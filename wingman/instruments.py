from pymavlink import mavutil
import threading
import queue
import time
import copy

class AttitudeMonitor:
    class Attitude:
        def __init__(self, roll, pitch, yaw):
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

        def __repr__(self):
            return f"Attitude(roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})"

    """
    Continuously monitors vehicle attitude using MAVLink ATTITUDE messages.
    Usage:
        monitor = AttitudeMonitor(mav)
        monitor.start()
        att = monitor.get_attitude()
        monitor.stop()
    """
    def __init__(self, vehicle):
        self.mav = vehicle.master
        self._attitude = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()

    def _run(self):
        while not self._stop_event.is_set():
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            if msg:
                with self._lock:
                    self._attitude = self.Attitude(msg.roll, msg.pitch, msg.yaw)
            time.sleep(0.01)

    def start(self):
        if not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def get_attitude(self):
        """
        Returns the Attitude(roll, pitch, yaw) in radians, or None if not yet received.
        """
        with self._lock:
            return self._attitude