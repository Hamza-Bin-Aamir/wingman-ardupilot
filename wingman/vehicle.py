from pymavlink import mavutil
import threading
import queue
import time
import math
from wingman.mission import Mission, Waypoint

class Vehicle:
    def __init__(self, connection_string, baud=115200, source_system=255):
        self.master = mavutil.mavlink_connection(
            connection_string, baud=baud, source_system=source_system
        )
        
        self.send_queue = queue.Queue()
        self.recv_queue = queue.Queue()
        self._stop_event = threading.Event()
        self.last_heartbeat_time = -1               # -1 means no heartbeat received yet
        self.last_heartbeat_send_time = -1          # -1 means no heartbeats sent yet
        self._msg_registrations = {}    # {msg_type: [Queue, ...]}
        self._msg_reg_lock = threading.Lock()
        self._heartbeat_lock = threading.Lock()     # Lock for heartbeat time variables
        
        # EKF and vibration monitoring
        self._ekf_vibe_lock = threading.Lock()
        self.last_vibration_data = None             # Latest VIBRATION message
        self.last_ekf_status = None                 # Latest EKF_STATUS_REPORT message
        self.last_sys_status = None                 # Latest SYS_STATUS message for EKF flags
        
        # Attitude data monitoring
        self._attitude_lock = threading.Lock()
        self.last_attitude = None                   # Latest ATTITUDE message (pitch, roll, yaw)
        
        # Flight data monitoring
        self._flight_data_lock = threading.Lock()
        self.last_vfr_hud = None                    # Latest VFR_HUD message (airspeed, heading, alt)
        self.last_global_position_int = None        # Latest GLOBAL_POSITION_INT message (GPS pos, AMSL alt)
        self.last_gps_raw_int = None                # Latest GPS_RAW_INT message (GPS status, HDOP, etc.)
        self.last_radio_status = None               # Latest RADIO_STATUS message (RSSI)
        self.last_nav_controller_output = None      # Latest NAV_CONTROLLER_OUTPUT message
        self.home_position = None                   # HOME_POSITION message
        self.last_heartbeat_msg = None              # Latest HEARTBEAT message for flight mode
        
        # Mission data monitoring
        self.last_mission_current = None            # Latest MISSION_CURRENT message (current mission item)
        self.last_mission_item_reached = None       # Latest MISSION_ITEM_REACHED message
        self.mission_items_cache = {}               # Cache for MISSION_ITEM responses {seq: mission_item_msg}
        self.pending_mission_item_requests = set()  # Track pending MISSION_ITEM requests

        # STATUSTEXT message queue for autopilot messages
        self.statustext_messages = queue.Queue(maxsize=50)  # Limited queue for autopilot text messages
        
        # Debug counters
        self._total_messages_received = 0
        self._statustext_messages_received = 0

        # Start threads for sending, receiving, and heartbeats
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        
        # Use wrapper function for heartbeat to ensure proper execution
        def heartbeat_wrapper():
            return self._heartbeat_loop()
        
        self._heartbeat_thread = threading.Thread(target=heartbeat_wrapper, daemon=True)
        
        # Start all threads
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
        print("🔄 _recv_loop started")
        while not self._stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                self._total_messages_received += 1
                msg_type = msg.get_type()
                
                # Debug: print every 100th message to avoid spam, but always print STATUSTEXT
                if msg_type == "STATUSTEXT" or self._total_messages_received % 100 == 0:
                    print(f"🔄 _recv_loop received: {msg_type} (total: {self._total_messages_received})")
                
                # Put in registered queues first
                with self._msg_reg_lock:
                    for t, qs in self._msg_registrations.items():
                        if msg_type == t:
                            for q in qs:
                                q.put(msg)
                # Special handling for heartbeat with thread safety
                if msg_type == "HEARTBEAT":
                    with self._heartbeat_lock:
                        self.last_heartbeat_time = time.time()
                    with self._flight_data_lock:
                        self.last_heartbeat_msg = msg
                        # Update vehicle_type on every new heartbeat
                        mav_type = getattr(msg, 'type', None)
                        if mav_type == 1:
                            self.vehicle_type = 'plane'
                        elif mav_type in [2, 3, 14, 17]:
                            self.vehicle_type = 'copter'
                        else:
                            self.vehicle_type = 'unknown'
                # Special handling for EKF and vibration data
                elif msg_type == "VIBRATION":
                    with self._ekf_vibe_lock:
                        self.last_vibration_data = msg
                elif msg_type == "EKF_STATUS_REPORT":
                    with self._ekf_vibe_lock:
                        self.last_ekf_status = msg
                elif msg_type == "SYS_STATUS":
                    with self._ekf_vibe_lock:
                        self.last_sys_status = msg
                # Special handling for attitude data
                elif msg_type == "ATTITUDE":
                    with self._attitude_lock:
                        self.last_attitude = msg
                # Special handling for flight data
                elif msg_type == "VFR_HUD":
                    # VFR_HUD contains both attitude (heading) and flight data (airspeed, alt)
                    with self._attitude_lock:
                        self.last_vfr_hud = msg
                    with self._flight_data_lock:
                        self.last_vfr_hud = msg
                elif msg_type == "GLOBAL_POSITION_INT":
                    with self._flight_data_lock:
                        self.last_global_position_int = msg
                elif msg_type == "GPS_RAW_INT":
                    with self._flight_data_lock:
                        self.last_gps_raw_int = msg
                elif msg_type == "RADIO_STATUS":
                    with self._flight_data_lock:
                        self.last_radio_status = msg
                elif msg_type == "NAV_CONTROLLER_OUTPUT":
                    with self._flight_data_lock:
                        self.last_nav_controller_output = msg
                elif msg_type == "HOME_POSITION":
                    with self._flight_data_lock:
                        self.home_position = msg
                elif msg_type == "MISSION_CURRENT":
                    with self._flight_data_lock:
                        self.last_mission_current = msg
                elif msg_type == "MISSION_ITEM_REACHED":
                    with self._flight_data_lock:
                        self.last_mission_item_reached = msg
                elif msg_type == "MISSION_ITEM":
                    with self._flight_data_lock:
                        # Cache the mission item and mark request as complete
                        self.mission_items_cache[msg.seq] = msg
                        self.pending_mission_item_requests.discard(msg.seq)
                # Handle STATUSTEXT messages for autopilot text messages
                elif msg_type == "STATUSTEXT":
                    self._statustext_messages_received += 1
                    # Debug logging for STATUSTEXT messages
                    text = getattr(msg, 'text', '')
                    # Handle both string and bytes cases
                    if isinstance(text, bytes):
                        text = text.decode('utf-8', errors='ignore').strip()
                    elif isinstance(text, str):
                        text = text.strip()
                    else:
                        text = str(text).strip()
                    
                    severity = getattr(msg, 'severity', 6)
                    print(f"🔤 STATUSTEXT #{self._statustext_messages_received} received: severity={severity}, text='{text}'")
                    
                    try:
                        self.statustext_messages.put_nowait(msg)
                        print(f"📥 STATUSTEXT queued. Queue size: {self.statustext_messages.qsize()}")
                    except queue.Full:
                        # Remove oldest message if queue is full
                        try:
                            self.statustext_messages.get_nowait()
                            self.statustext_messages.put_nowait(msg)
                            print(f"📥 STATUSTEXT queue was full, replaced oldest. Queue size: {self.statustext_messages.qsize()}")
                        except queue.Empty:
                            pass
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
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                current_time = time.time()
                with self._heartbeat_lock:
                    self.last_heartbeat_send_time = current_time
            except Exception as e:
                # Continue the loop even if there's an error
                pass
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
            print(f"🌐 Sending MAVLink MISSION_ITEM_INT: seq={wp.seq}, frame={wp.frame}, cmd={wp.command}, current={wp.current}, auto={int(wp.autocontinue)}")
            print(f"🌐   params=[{wp.param1}, {wp.param2}, {wp.param3}, {wp.param4}], coords=[{int(wp.x * 1e7)}, {int(wp.y * 1e7)}], alt={wp.z}")
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
        
        print(f"✅ Mission upload completed! Uploaded {len(waypoints)} waypoints, received MISSION_ACK type={ack.type}")
        
        # Store mission count for status tracking (from upload)
        with self._flight_data_lock:
            pass  # Mission count no longer tracked

    def set_current_waypoint(self, waypoint_seq, target_system=1, target_component=1, timeout=5):
        """
        Set the current waypoint in the mission to the specified sequence number.
        This tells ArduPilot which waypoint to execute next when in AUTO mode.
        """
        print(f"🎯 Setting current waypoint to seq={waypoint_seq}")
        
        # Send MISSION_SET_CURRENT command
        print(f"🌐 Sending MISSION_SET_CURRENT: target_system={target_system}, target_component={target_component}, seq={waypoint_seq}")
        self.master.mav.mission_set_current_send(
            target_system,
            target_component,
            waypoint_seq
        )
        print(f"🌐 MISSION_SET_CURRENT sent successfully")
        
        # Wait for MISSION_CURRENT response to confirm
        print(f"🔄 Waiting for MISSION_CURRENT confirmation...")
        current_queue = self.register_message_listener('MISSION_CURRENT')
        start = time.time()
        received_messages = 0
        while time.time() - start < timeout:
            try:
                current_msg = current_queue.get(timeout=0.5)
                received_messages += 1
                print(f"📥 Received MISSION_CURRENT: seq={current_msg.seq} (expected={waypoint_seq}) - message #{received_messages}")
                if current_msg.seq == waypoint_seq:
                    print(f"✅ Current waypoint set to seq={waypoint_seq}")
                    self.unregister_message_listener('MISSION_CURRENT', current_queue)
                    return True
            except queue.Empty:
                print(f"⏳ No MISSION_CURRENT received yet... ({time.time() - start:.1f}s elapsed)")
                continue
        
        self.unregister_message_listener('MISSION_CURRENT', current_queue)
        print(f"⚠️  Timeout waiting for MISSION_CURRENT confirmation for seq={waypoint_seq} (received {received_messages} messages)")
        return False

    def set_current_waypoint_alternative(self, waypoint_seq, target_system=1, target_component=1):
        """
        Alternative method to set current waypoint using DO_SET_MISSION_CURRENT command.
        This is sometimes more reliable than MISSION_SET_CURRENT.
        """
        print(f"🎯 Alternative: Setting current waypoint to seq={waypoint_seq} using DO_SET_MISSION_CURRENT")
        
        # Send MAV_CMD_DO_SET_MISSION_CURRENT command
        self.master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,  # command
            0,  # confirmation
            waypoint_seq,  # param1: sequence number
            0,  # param2: unused
            0,  # param3: unused
            0,  # param4: unused
            0,  # param5: unused
            0,  # param6: unused
            0   # param7: unused
        )
        print(f"🌐 DO_SET_MISSION_CURRENT command sent for seq={waypoint_seq}")
        return True

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
        
        # Store mission count for status tracking (from download)
        with self._flight_data_lock:
            pass  # Mission count no longer tracked
            
        return mission

    def get_heartbeat_times(self):
        """
        Thread-safe method to get heartbeat times.
        Returns (last_heartbeat_time, last_heartbeat_send_time)
        """
        with self._heartbeat_lock:
            return self.last_heartbeat_time, self.last_heartbeat_send_time

    def get_ekf_vibe_status(self):
        """
        Thread-safe method to get EKF and vibration status.
        Returns a dictionary with EKF and VIBE status information.
        """
        with self._ekf_vibe_lock:
            status = {
                "ekf_ok": True,
                "ekf_status": "healthy",  # healthy, warning, critical
                "vibe_x": 0.0,
                "vibe_y": 0.0, 
                "vibe_z": 0.0,
                "vibe_max": 0.0,
                "vibe_status": "healthy",  # healthy, warning, critical
                "has_data": False
            }
            
            # Process vibration data
            if self.last_vibration_data:
                status["has_data"] = True
                status["vibe_x"] = self.last_vibration_data.vibration_x
                status["vibe_y"] = self.last_vibration_data.vibration_y
                status["vibe_z"] = self.last_vibration_data.vibration_z
                
                # Calculate max vibration magnitude
                vibe_max = max(abs(status["vibe_x"]), abs(status["vibe_y"]), abs(status["vibe_z"]))
                status["vibe_max"] = vibe_max
                
                # Determine vibration status based on thresholds
                if vibe_max > 30.0:
                    status["vibe_status"] = "critical"
                elif vibe_max > 20.0:
                    status["vibe_status"] = "warning"
                else:
                    status["vibe_status"] = "healthy"
            
            # Process EKF status - check SYS_STATUS for EKF flags
            if self.last_sys_status:
                status["has_data"] = True
                # EKF flags from SYS_STATUS
                ekf_flags = self.last_sys_status.onboard_control_sensors_health
                
                # ArduPilot EKF health flags (bit positions)
                EKF_ATTITUDE = 0x00000008     # bit 3
                EKF_VELOCITY = 0x00000010     # bit 4  
                EKF_POSITION = 0x00000020     # bit 5
                
                ekf_healthy = (ekf_flags & EKF_ATTITUDE) and (ekf_flags & EKF_VELOCITY) and (ekf_flags & EKF_POSITION)
                
                if not ekf_healthy:
                    status["ekf_ok"] = False
                    status["ekf_status"] = "critical"
                else:
                    status["ekf_ok"] = True
                    status["ekf_status"] = "healthy"
            
            # If we have EKF_STATUS_REPORT, use it for more detailed status
            if self.last_ekf_status:
                status["has_data"] = True
                # EKF_STATUS_REPORT has velocity_variance, pos_horiz_variance, etc.
                # We can use these for more nuanced status
                vel_variance = getattr(self.last_ekf_status, 'velocity_variance', 0)
                pos_variance = getattr(self.last_ekf_status, 'pos_horiz_variance', 0)
                
                # Thresholds for EKF variance (these are typical values)
                if vel_variance > 1.0 or pos_variance > 1.0:
                    status["ekf_status"] = "critical"
                    status["ekf_ok"] = False
                elif vel_variance > 0.5 or pos_variance > 0.5:
                    status["ekf_status"] = "warning"
                    status["ekf_ok"] = True
                else:
                    status["ekf_status"] = "healthy"
                    status["ekf_ok"] = True
            
            return status

    def get_attitude_data(self):
        """
        Thread-safe method to get attitude data for virtual horizon.
        Returns a dictionary with pitch, roll, yaw (heading) data.
        """
        with self._attitude_lock:
            attitude_data = {
                "pitch": 0.0,       # Pitch in degrees
                "roll": 0.0,        # Roll in degrees  
                "yaw": 0.0,         # Yaw in degrees
                "heading": 0.0,     # Compass heading in degrees
                "has_data": False
            }
            
            # Process ATTITUDE message for pitch, roll, yaw
            if self.last_attitude:
                attitude_data["has_data"] = True
                # Convert from radians to degrees
                attitude_data["pitch"] = math.degrees(self.last_attitude.pitch)
                attitude_data["roll"] = math.degrees(self.last_attitude.roll)
                attitude_data["yaw"] = math.degrees(self.last_attitude.yaw)
                
                # Normalize yaw to 0-360 degrees
                attitude_data["yaw"] = attitude_data["yaw"] % 360
                if attitude_data["yaw"] < 0:
                    attitude_data["yaw"] += 360
            
            # Process VFR_HUD message for heading (compass)
            if self.last_vfr_hud:
                attitude_data["has_data"] = True
                attitude_data["heading"] = self.last_vfr_hud.heading
                
                # Normalize heading to 0-360 degrees
                attitude_data["heading"] = attitude_data["heading"] % 360
                if attitude_data["heading"] < 0:
                    attitude_data["heading"] += 360
            
            # If we don't have VFR_HUD but have ATTITUDE, use yaw as heading
            if not self.last_vfr_hud and self.last_attitude:
                attitude_data["heading"] = attitude_data["yaw"]
            
            return attitude_data

    def get_flight_data(self):
        """
        Thread-safe method to get flight data including airspeed, altitude, GPS, RSSI, flight mode.
        Returns a dictionary with all flight parameters.
        """
        with self._flight_data_lock:
            # ArduPlane flight modes mapping
            FLIGHT_MODES = {
                0: "Manual", 1: "Circle", 2: "Stabilize", 3: "Training",
                4: "Acro", 5: "FBW-A", 6: "FBW-B", 7: "Cruise",
                8: "Autotune", 10: "Auto", 11: "RTL", 12: "Loiter",
                13: "Takeoff", 14: "Avoid ADSB", 15: "Guided",
                16: "Initialising", 17: "QStabilize", 18: "QHover",
                19: "QLoiter", 20: "QLand", 21: "QRTL", 22: "QAutotune",
                23: "QAcro", 24: "Thermal"
            }
            
            # GPS Fix types
            GPS_FIX_TYPE = {
                0: "No GPS", 1: "No Fix", 2: "2D Fix",
                3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"
            }
            
            data = {
                # Time to target (placeholder for now)
                "time_to_target": None,
                
                # Airspeed data
                "airspeed_ias": None,
                "airspeed_ground": None,
                "airspeed_value": 0.0,
                "airspeed_type": "Unknown",
                
                # Altitude data
                "altitude_amsl": None,
                "altitude_relative": None,
                "altitude_value": 0.0,
                "altitude_type": "Unknown",
                
                # GPS data
                "gps_fix_type": 0,
                "gps_status": "No GPS",
                "gps_hdop": None,
                "gps_vdop": None,
                "gps_satellites": 0,
                
                # RSSI data
                "rssi_value": 0,
                "rssi_status": "unknown",
                
                # Flight mode
                "flight_mode_num": 0,
                "flight_mode": "Unknown",
                
                "has_data": False
            }
            
            # Process VFR_HUD data (airspeed, altitude, etc.)
            if self.last_vfr_hud:
                data["has_data"] = True
                data["airspeed_ias"] = self.last_vfr_hud.airspeed  # m/s
                data["airspeed_ground"] = self.last_vfr_hud.groundspeed  # m/s
                data["altitude_relative"] = self.last_vfr_hud.alt  # meters above home
                
                # Prefer IAS if available, otherwise use ground speed
                if data["airspeed_ias"] > 0:
                    data["airspeed_value"] = data["airspeed_ias"]
                    data["airspeed_type"] = "IAS"
                else:
                    data["airspeed_value"] = data["airspeed_ground"]
                    data["airspeed_type"] = "GPS"
            
            # Process GLOBAL_POSITION_INT data (AMSL altitude, position, and terrain if available)
            if self.last_global_position_int:
                data["has_data"] = True
                data["altitude_amsl"] = self.last_global_position_int.alt / 1000.0  # mm to meters
                # Terrain altitude (if available)
                terrain_alt = getattr(self.last_global_position_int, 'terrain_alt', None)
                if terrain_alt is not None and terrain_alt != 0 and terrain_alt != 2147483647:
                    data["altitude_terrain"] = terrain_alt / 1000.0  # mm to meters
                else:
                    data["altitude_terrain"] = None

                # Prefer Terrain > AMSL > Above Home
                if data["altitude_terrain"] is not None:
                    data["altitude_value"] = data["altitude_terrain"]
                    data["altitude_type"] = "Terrain"
                elif data["altitude_amsl"] is not None:
                    data["altitude_value"] = data["altitude_amsl"]
                    data["altitude_type"] = "AMSL"
                elif data["altitude_relative"] is not None:
                    data["altitude_value"] = data["altitude_relative"]
                    data["altitude_type"] = "Above Home"
                else:
                    data["altitude_value"] = 0.0
                    data["altitude_type"] = "Unknown"
            
            # Process GPS_RAW_INT data (GPS status, HDOP, satellites)
            if self.last_gps_raw_int:
                data["has_data"] = True
                data["gps_fix_type"] = self.last_gps_raw_int.fix_type
                data["gps_status"] = GPS_FIX_TYPE.get(data["gps_fix_type"], "Unknown")
                data["gps_hdop"] = self.last_gps_raw_int.eph / 100.0 if self.last_gps_raw_int.eph != 65535 else None
                data["gps_vdop"] = self.last_gps_raw_int.epv / 100.0 if self.last_gps_raw_int.epv != 65535 else None
                data["gps_satellites"] = self.last_gps_raw_int.satellites_visible
            
            # Process RADIO_STATUS data (RSSI)
            if self.last_radio_status:
                data["has_data"] = True
                data["rssi_value"] = self.last_radio_status.rssi
                
                # Determine RSSI status (typical values: 0-255, higher is better)
                if data["rssi_value"] > 150:
                    data["rssi_status"] = "good"
                elif data["rssi_value"] > 100:
                    data["rssi_status"] = "poor"
                else:
                    data["rssi_status"] = "very_poor"
            
            # Process HEARTBEAT for flight mode
            if self.last_heartbeat_msg:
                data["has_data"] = True
                # For ArduPilot, the custom_mode field contains the flight mode
                custom_mode = getattr(self.last_heartbeat_msg, 'custom_mode', 0)
                data["flight_mode_num"] = custom_mode
                data["flight_mode"] = FLIGHT_MODES.get(custom_mode, f"Mode {custom_mode}")
            
            return data

    def _request_mission_item(self, seq, target_system=1, target_component=1):
        """Request a specific mission item from the autopilot."""
        if seq not in self.pending_mission_item_requests and seq not in self.mission_items_cache:
            self.pending_mission_item_requests.add(seq)
            # Send MISSION_REQUEST_INT message
            self.master.mav.mission_request_int_send(
                target_system,
                target_component,
                seq,
                0  # mission_type: mission items
            )

    def get_mission_data(self):
        """
        Get current mission status data using only real MAVLink messages.
        Returns mission data only when current mission item information is available from MAVLink.
        """
        with self._flight_data_lock:
            # Mission command mapping
            MISSION_COMMANDS = {
                0: "Waypoint",
                16: "Waypoint",
                17: "Loiter Unlimited",
                18: "Loiter Turns",
                19: "Loiter Time",
                20: "Return to Launch",
                21: "Land",
                22: "Takeoff",
                84: "VTOL Takeoff",
                85: "VTOL Land",
                176: "Do Set Servo",
                177: "Do Repeat Servo",
                178: "Do Set Relay",
                179: "Do Repeat Relay",
                183: "Do Set CAM Trigg Dist",
                2000: "Image Start Capture",
                2001: "Image Stop Capture"
            }
            
            mission_data = {
                "current_item": 0,
                "total_items": 0,
                "item_type": "Waypoint",  # Command 0 maps to Waypoint
                "item_command": 0,
                "mission_status": "pending",
                "mission_state": "Not Started",
                "has_data": False
            }
            
            # Determine mission status based on flight mode even without mission items
            if self.last_heartbeat_msg:
                custom_mode = getattr(self.last_heartbeat_msg, 'custom_mode', 0)
                
                if custom_mode == 10:  # Auto mode
                    mission_data["mission_state"] = "Active"
                    mission_data["mission_status"] = "Active"
                elif custom_mode == 11:  # RTL mode
                    mission_data["mission_state"] = "Returning Home"
                    mission_data["mission_status"] = "Active"
                    mission_data["item_type"] = "Return to Launch"
                    mission_data["item_command"] = 20
                elif custom_mode in [0, 2, 5, 6]:  # Manual modes
                    mission_data["mission_state"] = "Manual Control"
                    mission_data["mission_status"] = "pending"
                else:
                    mission_data["mission_state"] = "Standby"
                    mission_data["mission_status"] = "pending"
                
                mission_data["has_data"] = True
            
            # Map item_command to item_type using MISSION_COMMANDS
            if mission_data["item_command"] in MISSION_COMMANDS:
                mission_data["item_type"] = MISSION_COMMANDS[mission_data["item_command"]]
            elif mission_data["item_command"] != 0:
                mission_data["item_type"] = f"Command {mission_data['item_command']}"
            
            # Only proceed with mission item details if we have current mission item
            if not self.last_mission_current:
                return mission_data
            
            current_seq = self.last_mission_current.seq
            mission_data["current_item"] = current_seq
            
            # Check if we have the mission item data for the current sequence
            if current_seq not in self.mission_items_cache:
                # Request the mission item if not already pending
                self._request_mission_item(current_seq)
                return mission_data
            
            # We have all required data - populate the response with actual mission item
            mission_item = self.mission_items_cache[current_seq]
            mission_data["item_command"] = mission_item.command
            
            # Map command number to readable type
            if mission_item.command in MISSION_COMMANDS:
                mission_data["item_type"] = MISSION_COMMANDS[mission_item.command]
            else:
                mission_data["item_type"] = f"Command {mission_item.command}"
            
            # Update mission status for Active mission execution
            if self.last_heartbeat_msg:
                custom_mode = getattr(self.last_heartbeat_msg, 'custom_mode', 0)
                
                if custom_mode == 10:  # Auto mode with Active mission item
                    mission_data["mission_state"] = "Executing"
                    mission_data["mission_status"] = "Active"
            
            mission_data["has_data"] = True
            return mission_data

    def arm(self, arm=True):
        """Arm or disarm the vehicle using MAVLink command_long_send."""
        # ArduPilot: MAV_CMD_COMPONENT_ARM_DISARM = 400
        # arm=True to arm, arm=False to disarm
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            0,    # confirmation
            1 if arm else 0,  # param1: 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0
        )
        return True

    def detect_vehicle_type(self):
        """Detect vehicle type (copter, plane, etc.) from the first heartbeat, opportunistically grabbing a heartbeat if needed."""
        if self.last_heartbeat_msg is None:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                with self._flight_data_lock:
                    self.last_heartbeat_msg = msg
        if self.last_heartbeat_msg:
            mav_type = self.last_heartbeat_msg.type
            if mav_type == 1:
                self.vehicle_type = 'plane'
            elif mav_type in [2, 3, 14, 17]:
                self.vehicle_type = 'copter'
            else:
                self.vehicle_type = 'unknown'
        else:
            self.vehicle_type = 'unknown'

    def get_vehicle_type(self):
        """Return detected vehicle type as string: 'plane', 'copter', or 'unknown'"""
        return getattr(self, 'vehicle_type', 'unknown')

    def set_mode(self, mode):
        """Set the vehicle mode using MAVLink command_long_send. Supports ArduPlane and ArduCopter."""
        # Detect vehicle type if not already set
        if not hasattr(self, 'vehicle_type') or self.vehicle_type == 'unknown':
            self.detect_vehicle_type()
        # Mode mappings
        PLANE_MODES = {
            'MANUAL': 0, 'CIRCLE': 1, 'STABILIZE': 2, 'TRAINING': 3, 'ACRO': 4, 'FBWA': 5, 'FBWB': 6,
            'CRUISE': 7, 'AUTOTUNE': 8, 'AUTO': 10, 'RTL': 11, 'LOITER': 12, 'TAKEOFF': 13, 'AVOID_ADSB': 14,
            'GUIDED': 15, 'INITIALISING': 16, 'QSTABILIZE': 17, 'QHOVER': 18, 'QLOITER': 19, 'QLAND': 20,
            'QRTL': 21, 'QAUTOTUNE': 22, 'THERMAL': 23
        }
        COPTER_MODES = {
            'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5, 'RTL': 6,
            'CIRCLE': 7, 'LAND': 9, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14, 'AUTOTUNE': 15, 'POSHOLD': 16,
            'BRAKE': 17, 'THROW': 18, 'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21, 'FLOWHOLD': 22,
            'FOLLOW': 23, 'ZIGZAG': 24, 'SYSTEMID': 25, 'AUTOROTATE': 26, 'AUTO_RTL': 27
        }
        mode_upper = mode.upper()
        if self.vehicle_type == 'plane':
            mode_num = PLANE_MODES.get(mode_upper)
        elif self.vehicle_type == 'copter':
            mode_num = COPTER_MODES.get(mode_upper)
        else:
            # Try both if unknown
            mode_num = PLANE_MODES.get(mode_upper)
            if mode_num is None:
                mode_num = COPTER_MODES.get(mode_upper)
        if mode_num is None:
            raise ValueError(f"Unknown mode: {mode} for vehicle type {self.vehicle_type}")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            176,  # MAV_CMD_DO_SET_MODE
            0,    # confirmation
            1,    # param1: mode (1=custom)
            mode_num,  # param2: custom mode number
            0, 0, 0, 0, 0
        )
        return True

    def is_armed(self):
        """Return True if the vehicle is currently armed, False otherwise. Uses last_heartbeat_msg.base_mode."""
        # MAVLink base_mode bit 7 (128) is MAV_MODE_FLAG_SAFETY_ARMED
        msg = self.last_heartbeat_msg
        if msg is not None and hasattr(msg, 'base_mode'):
            return bool(msg.base_mode & 0b10000000)
        return False

    def get_statustext_messages(self):
        """
        Get pending STATUSTEXT messages from the autopilot.
        Returns a list of message dictionaries containing type, text, and timestamp.
        """
        messages = []
        queue_size_before = self.statustext_messages.qsize()
        print(f"📤 get_statustext_messages called. Queue size: {queue_size_before}")
        
        # Non-blocking retrieval of all available messages
        while True:
            try:
                msg = self.statustext_messages.get_nowait()
                if msg:
                    # MAVLink STATUSTEXT severity levels: 0=emergency, 1=alert, 2=critical, 3=error, 4=warning, 5=notice, 6=info, 7=debug
                    severity_level = getattr(msg, 'severity', 6)  # Default to info
                    text = getattr(msg, 'text', '')
                    
                    # Handle both string and bytes cases
                    if isinstance(text, bytes):
                        text = text.decode('utf-8', errors='ignore').strip()
                    elif isinstance(text, str):
                        text = text.strip()
                    else:
                        text = str(text).strip()
                    
                    # Map MAVLink severity to frontend message types
                    if severity_level <= 2:  # emergency, alert, critical
                        msg_type = 'error'
                    elif severity_level <= 4:  # error, warning
                        msg_type = 'warning' 
                    elif severity_level == 5:  # notice
                        msg_type = 'status'
                    else:  # info, debug
                        msg_type = 'info'
                    
                    if text:  # Only add non-empty messages
                        message_data = {
                            'type': msg_type,
                            'text': text,
                            'severity': severity_level,
                            'timestamp': time.time()
                        }
                        messages.append(message_data)
                        print(f"📝 Formatted message: {message_data}")
            except queue.Empty:
                break
        
        print(f"📤 Returning {len(messages)} messages")
        return messages
