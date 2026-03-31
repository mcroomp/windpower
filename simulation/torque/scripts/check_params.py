import sys, time
sys.path.insert(0, '/rawes/.venv/lib/python3.10/site-packages')
from pymavlink import mavutil

m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', autoreconnect=False)
m.wait_heartbeat(timeout=8)
print("Connected — fetching all params...")
time.sleep(2)   # let vehicle initialize

m.mav.param_request_list_send(m.target_system, m.target_component)
params = {}
t0 = time.monotonic()
while time.monotonic() - t0 < 10:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg:
        params[msg.param_id.rstrip('\x00')] = msg.param_value
    elif len(params) > 0:
        break

# Print SCR_* and RPM* and SERVO9*
for k in sorted(params):
    if any(k.startswith(p) for p in ('SCR', 'RPM', 'SERVO9')):
        print(f"  {k} = {params[k]}")
print(f"Total params: {len(params)}")
