"""
Example of how to send MAV_DISTANCE_SENSOR messages to integrate a custom distance sensor
to the autopilot using pymavlink
"""

import time

# Import mavutil
from pymavlink import mavutil

# Wait for server connection
# Connect to the default listening port for
# mavproxy on Blue Robotics companion computer
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def wait_heartbeat(m):
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_component))

starttime = time.time()
while True:
    master.mav.heartbeat_send(
            6, #MAVTYPE = MAV_TYPE_GCS
            8, #MAVAUTOPILOT = MAV_AUTOPILOT_INVALID
            128, # MAV_MODE = MAV_MODE_FLAG_SAFETY_ARMED, have also tried 0 here
            0,0)
    print('HB')
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg == None:
        continue
    else:
        print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_component))
    time.sleep(1.0 - ((time.time() - starttime) % 1.0))
