from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:127.0.0.1:14540')

starttime = time.time()
while True:
    master.mav.heartbeat_send(
            6, #MAVTYPE = MAV_TYPE_GCS
            8, #MAVAUTOPILOT = MAV_AUTOPILOT_INVALID
            128, # MAV_MODE = MAV_MODE_FLAG_SAFETY_ARMED, have also tried 0 here
            0,0)
    print('HB')
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg == None:
        pass
    else:
        print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_component))
    time.sleep(1.0 - ((time.time() - starttime) % 1.0))