from pymavlink import mavutil
import threading
import queue
import time
from mission import Mission, Waypoint

class Vehicle:
    def __init__(self, connection_string, baud=115200, source_system=255):
        self.master = mavutil.mavlink_connection(
            connection_string, baud=baud, source_system=source_system
        )
        self.send_queue = queue.Queue()
        self.recv_queue = queue.Queue()
        self._stop_event = threading.Event()
        self.last_heartbeat_time = -1  # -1 means no heartbeat received yet
        self._msg_registrations = {}   # {msg_type: [Queue, ...]}
        self._msg_reg_lock = threading.Lock()

        # Start threads for sending, receiving, and heartbeats
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._recv_thread.start()
        self._send_thread.start()
        self._heartbeat_thread.start()

    def register_message_listener(self, msg_types):
        """
        Register to listen for specific MAVLink message types.
        Returns a queue.Queue object where matching messages will be put.
        """
        if isinstance(msg_types, str):
            msg_types = [msg_types]
        q = queue.Queue()
        with self._msg_reg_lock:
            for t in msg_types:
                self._msg_registrations.setdefault(t, []).append(q)
        return q

    def unregister_message_listener(self, msg_types, q):
        if isinstance(msg_types, str):
            msg_types = [msg_types]
        with self._msg_reg_lock:
            for t in msg_types:
                if t in self._msg_registrations:
                    self._msg_registrations[t].remove(q)
                    if not self._msg_registrations[t]:
                        del self._msg_registrations[t]

    def _recv_loop(self):
        while not self._stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                # Put in registered queues first
                with self._msg_reg_lock:
                    for t, qs in self._msg_registrations.items():
                        if msg_type == t:
                            for q in qs:
                                q.put(msg)
                # Special handling for heartbeat
                if msg_type == "HEARTBEAT":
                    self.last_heartbeat_time = time.time()
                self.recv_queue.put(msg)

    def _send_loop(self):
        while not self._stop_event.is_set():
            try:
                msg = self.send_queue.get(timeout=1)
                self.master.mav.send(msg)
            except queue.Empty:
                continue

    def _heartbeat_loop(self):
        while not self._stop_event.is_set():
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(1)

    def send_message(self, msg):
        self.send_queue.put(msg)

    def get_message(self, timeout=None):
        try:
            return self.recv_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        self._stop_event.set()
        self._recv_thread.join()
        self._send_thread.join()
        self._heartbeat_thread.join()
        self.master.close()

    def get_home(self, timeout=5, target_system=1, target_component=1):
        """
        Request and return the current home position from the vehicle.
        Returns (lat, lon, alt) in degrees/meters, or None if not received.
        """
        # Register for HOME_POSITION message
        home_queue = self.register_message_listener('HOME_POSITION')
        # Request home position (ArduPilot responds automatically on boot/arm, but we can try)
        self.master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        home = None
        start = time.time()
        while time.time() - start < timeout:
            try:
                msg = home_queue.get(timeout=0.5)
                if msg:
                    # ArduPilot: lat/lon in 1e7, alt in mm
                    lat = msg.latitude / 1e7
                    lon = msg.longitude / 1e7
                    alt = msg.altitude / 1000.0
                    home = (lat, lon, alt)
                    break
            except queue.Empty:
                continue
        self.unregister_message_listener('HOME_POSITION', home_queue)
        return home

    def set_home(self, lat, lon, alt, target_system=1, target_component=1, timeout=5):
        """
        Set the home position on the vehicle using MAV_CMD_DO_SET_HOME.
        Waits for HOME_POSITION confirmation.
        Returns True if confirmed, False otherwise.
        """
        # Register for HOME_POSITION message before sending
        home_queue = self.register_message_listener('HOME_POSITION')
        self.master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,          # confirmation
            1,          # use specified location (1=yes)
            lat,
            lon,
            alt,
            0, 0, 0     # unused
        )
        confirmed = False
        start = time.time()
        while time.time() - start < timeout:
            try:
                msg = home_queue.get(timeout=0.5)
                if msg:
                    # ArduPilot: lat/lon in 1e7, alt in mm
                    lat_rcv = msg.latitude / 1e7
                    lon_rcv = msg.longitude / 1e7
                    alt_rcv = msg.altitude / 1000.0
                    # Confirm if matches what we set (with some tolerance)
                    if abs(lat_rcv - lat) < 1e-6 and abs(lon_rcv - lon) < 1e-6 and abs(alt_rcv - alt) < 0.5:
                        confirmed = True
                        break
            except queue.Empty:
                continue
        self.unregister_message_listener('HOME_POSITION', home_queue)
        return confirmed

    # --- Mission upload/download functionality ---
    # TODO: Fix uploading first mission item
    def upload_mission(self, mission, target_system=1, target_component=1, timeout=10):
        """
        Upload a Mission object to the vehicle.
        Only mission waypoints (not home) are uploaded.
        """
        waypoints = mission.waypoints 
        self.master.mav.mission_count_send(target_system, target_component, len(waypoints))
        Should_Increment = False
        for i, wp in enumerate(waypoints):
            # Register for the expected response before sending
            req_queue = self.register_message_listener(['MISSION_REQUEST_INT', 'MISSION_REQUEST'])
            start = time.time()
            got_req = None
            while time.time() - start < timeout:
                try:
                    got_req = req_queue.get(timeout=0.5)
                    if got_req.seq == i:
                        break
                except queue.Empty:
                    continue
            self.unregister_message_listener(['MISSION_REQUEST_INT', 'MISSION_REQUEST'], req_queue)
            if not got_req or got_req.seq != i:
                raise TimeoutError(f"Timeout waiting for MISSION_REQUEST(_INT) for seq {i}")
            # Always send MISSION_ITEM_INT
            self.master.mav.mission_item_int_send(
                target_system,
                target_component,
                wp.seq,
                wp.frame,
                wp.command,
                wp.current,
                int(wp.autocontinue),
                wp.param1,
                wp.param2,
                wp.param3,
                wp.param4,
                int(wp.x * 1e7),  # lat or x * 1e7
                int(wp.y * 1e7),  # lon or y * 1e7
                wp.z
            )
        # Register for MISSION_ACK before waiting
        ack_queue = self.register_message_listener('MISSION_ACK')
        ack = None
        start = time.time()
        while time.time() - start < timeout:
            try:
                ack = ack_queue.get(timeout=0.5)
                break
            except queue.Empty:
                continue
        self.unregister_message_listener('MISSION_ACK', ack_queue)
        if not ack:
            raise TimeoutError("Timeout waiting for MISSION_ACK")

    def download_mission(self, target_system=1, target_component=1, timeout=10):
        """
        Download mission from the vehicle and return a Mission object.
        The first waypoint (home) is always managed by ArduPilot and is included in the download.
        Uses MISSION_REQUEST_INT for requesting waypoints.
        """
        # Register for MISSION_COUNT before sending request
        count_queue = self.register_message_listener('MISSION_COUNT')
        self.master.mav.mission_request_list_send(target_system, target_component)
        msg = None
        start = time.time()
        while time.time() - start < timeout:
            try:
                msg = count_queue.get(timeout=0.5)
                break
            except queue.Empty:
                continue
        self.unregister_message_listener('MISSION_COUNT', count_queue)
        if not msg:
            raise TimeoutError("Timeout waiting for MISSION_COUNT")
        count = msg.count
        mission = Mission()
        for i in range(count):
            # Register for MISSION_ITEM_INT before sending request
            item_queue = self.register_message_listener('MISSION_ITEM_INT')
            self.master.mav.mission_request_int_send(target_system, target_component, i)
            item = None
            start = time.time()
            while time.time() - start < timeout:
                try:
                    item = item_queue.get(timeout=0.5)
                    break
                except queue.Empty:
                    continue
            self.unregister_message_listener('MISSION_ITEM_INT', item_queue)
            if not item:
                raise TimeoutError(f"Timeout waiting for MISSION_ITEM_INT {i}")
            wp = Waypoint(
                seq=item.seq,
                frame=item.frame,
                command=item.command,
                x=item.x / 1e7,
                y=item.y / 1e7,
                z=item.z,
                autocontinue=bool(item.autocontinue),
                current=item.current,
                param1=item.param1,
                param2=item.param2,
                param3=item.param3,
                param4=item.param4
            )
            mission.add_waypoint(wp)
        return mission

