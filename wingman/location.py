from pymavlink import mavutil
import threading
import queue
import time
import copy

class PositionMonitor:
    class Position:
        def __init__(self, lat, lon, alt):
            self.lat = lat
            self.lon = lon
            self.alt = alt

        def __repr__(self):
            return f"Position(lat={self.lat}, lon={self.lon}, alt={self.alt})"

    """
    Continuously monitors vehicle position using MAVLink GLOBAL_POSITION_INT messages.
    Usage:
        monitor = PositionMonitor(mav)
        monitor.start()
        pos = monitor.locate()
        monitor.stop()
    """
    def __init__(self, vehicle):
        self.mav = vehicle.master
        self._position = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()

    def _run(self):
        while not self._stop_event.is_set():
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                with self._lock:
                    self._position = self.Position(msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
            time.sleep(0.01)

    def start(self):
        if not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def locate(self):
        """
        Returns Position(lat, lon, alt) or None if not yet received.
        """
        with self._lock:
            return self._position