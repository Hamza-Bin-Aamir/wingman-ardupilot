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

if __name__ == "__main__":
    test_version()
    test_vehicle_init()
    test_vehicle_send_and_receive()
    print("All tests passed!")