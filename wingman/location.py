from pymavlink import mavutil
import threading
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
        monitor = PositionMonitor(vehicle)
        monitor.start()
        pos = monitor.locate()
        monitor.stop()
    """
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self._position = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()
        self._position_queue = None

    def _run(self):
        while not self._stop_event.is_set():
            try:
                msg = self._position_queue.get(timeout=1)
                if msg:
                    with self._lock:
                        self._position = self.Position(msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
            except Exception:
                continue
            time.sleep(0.01)

    def start(self):
        if not self._thread.is_alive():
            self._stop_event.clear()
            self._position_queue = self.vehicle.register_message_listener('GLOBAL_POSITION_INT')
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        if self._position_queue:
            self.vehicle.unregister_message_listener('GLOBAL_POSITION_INT', self._position_queue)
            self._position_queue = None

    def locate(self):
        """
        Returns a copy of Position(lat, lon, alt) or None if not yet received.
        """
        with self._lock:
            return self._position