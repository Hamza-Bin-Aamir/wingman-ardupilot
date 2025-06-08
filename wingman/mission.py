from pymavlink import mavutil

class Waypoint:
    def __init__(self, seq, frame, command, x, y, z, autocontinue=True, current=0, param1=0, param2=0, param3=0, param4=0):
        self.seq = seq
        self.frame = frame
        self.command = command
        self.x = x  # latitude (deg) or local X (m)
        self.y = y  # longitude (deg) or local Y (m)
        self.z = z  # altitude (m)
        self.autocontinue = autocontinue
        self.current = current
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4

    def __repr__(self):
        return (f"Waypoint(seq={self.seq}, frame={self.frame}, command={self.command}, "
                f"x={self.x}, y={self.y}, z={self.z}, autocontinue={self.autocontinue}, "
                f"current={self.current}, param1={self.param1}, param2={self.param2}, "
                f"param3={self.param3}, param4={self.param4})")

    def to_mavlink(self):
        return mavutil.mavlink.MAVLink_mission_item_message(
            0,  # target_system (set when sending)
            0,  # target_component (set when sending)
            self.seq,
            self.frame,
            self.command,
            self.current,
            int(self.autocontinue),
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            self.x,
            self.y,
            self.z
        )

class Mission:
    def __init__(self):
        self.waypoints = []

    def add_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def clear(self):
        self.waypoints = []

    def count(self):
        return len(self.waypoints)

    def get_waypoint(self, idx):
            return self.waypoints[idx]
    def to_mavlink(self):
        """Return a list of MAVLink mission_item messages (system/component must be set before sending)."""
        return [wp.to_mavlink() for wp in self.waypoints]

    def show(self):
        """Print all waypoints in the mission."""
        wps = self.waypoints
        if not wps:
            print("No waypoints in mission.")
        for wp in wps:
            print(wp)