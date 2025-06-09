from pymavlink import mavutil
import threading
import time
import queue

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
        monitor = AttitudeMonitor(vehicle)
        monitor.start()
        att = monitor.get_attitude()
        monitor.stop()
    """
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self._attitude = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()
        self._attitude_queue = None

    def _run(self):
        while not self._stop_event.is_set():
            try:
                msg = self._attitude_queue.get(timeout=1)
                if msg:
                    with self._lock:
                        self._attitude = self.Attitude(msg.roll, msg.pitch, msg.yaw)
            except queue.Empty:
                continue
            time.sleep(0.01)

    def start(self):
        if not self._thread.is_alive():
            self._stop_event.clear()
            self._attitude_queue = self.vehicle.register_message_listener('ATTITUDE')
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        if self._attitude_queue:
            self.vehicle.unregister_message_listener('ATTITUDE', self._attitude_queue)
            self._attitude_queue = None

    def get_attitude(self):
        """
        Returns Attitude(roll, pitch, yaw) in radians, or None if not yet received.
        """
        with self._lock:
            return self._attitude

class CustomMonitor:
    """
    Monitors all fields of a specified MAVLink message type.
    Returns the latest message as a dictionary of field names to values.
    Usage:
        monitor = CustomMonitor(vehicle, 'NAMED_VALUE_FLOAT')
        monitor.start()
        data = monitor.get_data()
        monitor.stop()
    """
    def __init__(self, vehicle, msg_type):
        self.vehicle = vehicle
        self.msg_type = msg_type
        self._data = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lock = threading.Lock()
        self._queue = None

    def _run(self):
        while not self._stop_event.is_set():
            try:
                msg = self._queue.get(timeout=1)
                if msg:
                    with self._lock:
                        # Convert MAVLink message to dict (excluding private fields)
                        self._data = {k: v for k, v in msg.__dict__.items() if not k.startswith('_')}
            except queue.Empty:
                continue
            time.sleep(0.01)

    def start(self):
        if not self._thread.is_alive():
            self._stop_event.clear()
            self._queue = self.vehicle.register_message_listener(self.msg_type)
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()
        if self._queue:
            self.vehicle.unregister_message_listener(self.msg_type, self._queue)
            self._queue = None

    def get_data(self):
        """
        Returns a dictionary of the latest message fields, or None if not yet received.
        """
        with self._lock:
            return dict(self._data) if self._data is not None else None