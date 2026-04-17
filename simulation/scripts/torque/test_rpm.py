"""Quick test: start a mediator + SITL and check RPM param value."""
import socket, json, struct, time, sys, subprocess, threading

sys.path.insert(0, '/rawes/.venv/lib/python3.10/site-packages')
from pymavlink import mavutil

# Start mediator in a thread
def run_mediator():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 9002))
    s.settimeout(0.01)
    addr = None
    t = 0.0
    end = time.monotonic() + 20
    while time.monotonic() < end:
        try:
            data, a = s.recvfrom(4096)
            addr = a
        except: pass
        if addr:
            msg = json.dumps({
                "timestamp": t,
                "imu": {"gyro": [0,0,0], "accel_body": [0,0,-9.81]},
                "position": [0,0,0], "attitude": [0,0,0], "velocity": [0,0,0],
                "rpm": [485.0, 0.0]
            }, separators=(',', ':')) + "\n"
            try: s.sendto(msg.encode(), addr)
            except: pass
        t += 0.0025
        time.sleep(0.0025)
    s.close()

t = threading.Thread(target=run_mediator, daemon=True)
t.start()
time.sleep(0.5)

# Start SITL
proc = subprocess.Popen([
    '/ardupilot/build/sitl/bin/arducopter-heli',
    '--home', '51.5,-0.1,50,0',
    '--model', 'JSON',
    '--defaults', 'Tools/autotest/default_params/copter-heli.parm',
    '-I0'
], cwd='/ardupilot', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

time.sleep(5)

# Connect and check RPM1_TYPE
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat(timeout=8)
print("Connected")

for pname in ['RPM1_TYPE', 'RPM1_MIN', 'SCR_ENABLE']:
    m.mav.param_request_read_send(m.target_system, m.target_component, pname.encode(), -1)
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    print(f'  {pname} = {msg.param_value if msg else "TIMEOUT"}')

proc.kill()
print("Done")
