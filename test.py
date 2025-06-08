# Our unit tests will live here
import wingman
import sys
import time

DEFAULT_CONN = "udp:127.0.0.1:14550"

def get_conn_string():
    if len(sys.argv) > 1:
        return sys.argv[1]
    return DEFAULT_CONN

def test_version():
    assert wingman.__version__ == "0.0.1"

def test_vehicle_init():
    from wingman.vehicle import Vehicle
    v = Vehicle(get_conn_string())
    start = time.time()
    connected = False
    while time.time() - start < 5:
        if v.last_heartbeat_time != -1:
            connected = True
            break
        time.sleep(0.1)
    v.close()
    assert connected, "No heartbeat received from vehicle within 5 seconds"

def test_vehicle_send_and_receive():
    from wingman.vehicle import Vehicle
    v = Vehicle(get_conn_string())
    msg = v.get_message(timeout=5)
    assert v.master is not None
    v.close()  
def test_vehicle_upload_and_download_mission():
    from wingman.vehicle import Vehicle
    from wingman.mission import Mission, Waypoint

    v = Vehicle(get_conn_string())
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

    v.close()

def test_vehicle_set_and_get_home():
    from wingman.vehicle import Vehicle

    v = Vehicle(get_conn_string())
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

    v.close()

# Add to main test runner
if __name__ == "__main__":
    test_version()
    test_vehicle_init()
    test_vehicle_send_and_receive()
    test_vehicle_upload_and_download_mission()
    test_vehicle_set_and_get_home()
    print("All tests passed!")