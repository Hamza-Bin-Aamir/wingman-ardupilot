# Our unit tests will live here
import wingman
import sys
import time
import argparse

DEFAULT_CONN = "udp:127.0.0.1:14550"

# Parse arguments at the top-level so all functions use the same connection string and show flag
parser = argparse.ArgumentParser()
parser.add_argument("--show", action="store_true", help="Show test names and live data")
parser.add_argument("--vehicle", type=str, default=DEFAULT_CONN, help="Vehicle connection string (default: udp:127.0.0.1:14550)")
args = parser.parse_args()
conn_str = args.vehicle

def get_conn_string():
    return conn_str

# Global vehicle instance
global_vehicle = None

def setup_vehicle():
    global global_vehicle
    if global_vehicle is None:
        from wingman.vehicle import Vehicle
        global_vehicle = Vehicle(get_conn_string())

def free_vehicle():
    global global_vehicle
    if global_vehicle is not None:
        global_vehicle.close()
        global_vehicle = None

def test_version():
    assert wingman.__version__ == "0.0.1"

def test_vehicle_init():
    setup_vehicle()
    v = global_vehicle
    start = time.time()
    connected = False
    while time.time() - start < 5:
        if v.last_heartbeat_time != -1:
            connected = True
            break
        time.sleep(0.1)
    assert connected, "No heartbeat received from vehicle within 5 seconds"

def test_vehicle_send_and_receive():
    setup_vehicle()
    v = global_vehicle
    msg = v.get_message(timeout=5)
    assert v.master is not None

def test_vehicle_upload_and_download_mission():
    setup_vehicle()
    v = global_vehicle
    from wingman.mission import Mission, Waypoint

    # Wait for heartbeat
    start = time.time()
    while time.time() - start < 5:
        if v.last_heartbeat_time != -1:
            break
        time.sleep(0.1)

    # Create example mission
    mission = Mission()
    mission.add_waypoint(Waypoint(
        seq=0,
        frame=3,     # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command=16,  # MAV_CMD_NAV_WAYPOINT
        x=47.397742,  # lat
        y=8.545594,   # lon
        z=10,         # alt
        autocontinue=True,
        current=0
    ))
    mission.add_waypoint(Waypoint(
        seq=1,
        frame=3,     
        command=16,
        x=47.397842,
        y=8.545694,
        z=10,
        autocontinue=True,
        current=0
    ))
    mission.add_waypoint(Waypoint(
        seq=2,
        frame=3,
        command=16,
        x=47.397842,
        y=8.545694,
        z=15,
        autocontinue=True,
        current=0
    ))

    # Upload mission
    v.upload_mission(mission)

    # Download mission
    downloaded = v.download_mission()
    # downloaded.show()

def test_vehicle_set_and_get_home():
    setup_vehicle()
    v = global_vehicle
    # Wait for heartbeat
    start = time.time()
    while time.time() - start < 5:
        if v.last_heartbeat_time != -1:
            break
        time.sleep(0.1)

    # Set a new home location
    lat, lon, alt = 47.397742, 8.545594, 10
    v.set_home(lat, lon, alt)

    # Get home location and check values
    home = v.get_home()
    assert home is not None, "Failed to get home position"
    # lat_rcv, lon_rcv, alt_rcv = home
    # assert abs(lat_rcv - lat) < 1e-6, f"Latitude mismatch: {lat_rcv} vs {lat}"
    # assert abs(lon_rcv - lon) < 1e-6, f"Longitude mismatch: {lon_rcv} vs {lon}"
    # assert abs(alt_rcv - alt) < 0.5, f"Altitude mismatch: {alt_rcv} vs {alt}"

def test_location_monitor(show=False):
    setup_vehicle()
    v = global_vehicle
    from wingman.location import PositionMonitor

    monitor = PositionMonitor(v)
    monitor.start()
    updates = 0
    start = time.time()
    pos = None
    while updates < 5 and time.time() - start < 10:
        pos = monitor.locate()
        if pos:
            updates += 1
            if show:
                print(f"[LocationMonitor] update {updates}: lat={pos.lat:.7f}, lon={pos.lon:.7f}, alt={pos.alt:.2f}")
        time.sleep(0.1)
    monitor.stop()
    assert pos is not None and updates == 5, "Did not receive 5 position updates from vehicle"

def test_attitude_monitor(show=False):
    setup_vehicle()
    v = global_vehicle
    from wingman.instruments import AttitudeMonitor

    monitor = AttitudeMonitor(v)
    monitor.start()
    updates = 0
    start = time.time()
    att = None
    while updates < 5 and time.time() - start < 10:
        att = monitor.get_attitude()
        if att:
            updates += 1
            if show:
                print(f"[AttitudeMonitor] update {updates}: roll={att.roll:.3f}, pitch={att.pitch:.3f}, yaw={att.yaw:.3f}")
        time.sleep(0.1)
    monitor.stop()
    assert att is not None and updates == 5, "Did not receive 5 attitude updates from vehicle"

def test_custom_monitor(show=False):
    setup_vehicle()
    v = global_vehicle
    from wingman.instruments import CustomMonitor

    monitor = CustomMonitor(v, 'SYS_STATUS')
    monitor.start()
    updates = 0
    start = time.time()
    data = None
    while updates < 5 and time.time() - start < 10:
        data = monitor.get_data()
        if data:
            updates += 1
            if show:
                print(f"[CustomMonitor] update {updates}: {data}")
        time.sleep(0.1)
    monitor.stop()
    assert data is not None and updates == 5, "Did not receive 5 SYS_STATUS updates from vehicle"

# Add to main test runner
if __name__ == "__main__":
    def run_test(name, func):
        if args.show:
            print(f"Running {name}...")
        func(show=args.show) if 'show' in func.__code__.co_varnames else func()
        if args.show:
            print(f"{name} passed.\n")

    try:
        run_test("test_version", test_version)
        run_test("test_vehicle_init", test_vehicle_init)
        run_test("test_vehicle_send_and_receive", test_vehicle_send_and_receive)
        run_test("test_vehicle_upload_and_download_mission", test_vehicle_upload_and_download_mission)
        run_test("test_vehicle_set_and_get_home", test_vehicle_set_and_get_home)
        run_test("test_location_monitor", test_location_monitor)
        run_test("test_attitude_monitor", test_attitude_monitor)
        run_test("test_custom_monitor", test_custom_monitor)
        print("All tests passed!")
    finally:
        free_vehicle()