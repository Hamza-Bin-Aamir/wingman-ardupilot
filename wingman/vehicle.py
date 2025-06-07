from pymavlink import mavutil
import threading
import queue
import time

class Vehicle:
    def __init__(self, connection_string, baud=115200, source_system=255):
        self.master = mavutil.mavlink_connection(
            connection_string, baud=baud, source_system=source_system
        )
        self.send_queue = queue.Queue()
        self.recv_queue = queue.Queue()
        self._stop_event = threading.Event()
        self.last_heartbeat_time = -1  # -1 means no heartbeat received yet

        # Start threads for sending, receiving, and heartbeats
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._recv_thread.start()
        self._send_thread.start()
        self._heartbeat_thread.start()

    def _recv_loop(self):
        while not self._stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                if msg.get_type() == "HEARTBEAT":
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

