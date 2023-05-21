#  ====================================================
# || 1. TX -> Tracker
# || 
# || 2. TX -> Tracker -> Komputer
# ||
# ||
# ||
# ||
# ||
# ||
# ||
#  ====================================================

from pymavlink import mavutil
import serial.tools.list_ports
baudRate = 9600

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_component))

# create a mavlink serial instance

ports = serial.tools.list_ports.comports()
# 'com_list' contains list of all com ports
def connect_serial(baudRate):
    ports = serial.tools.list_ports.comports()
    com_list = []
    for p in ports:
        print(p)
        com_list.append(p.device)
    if len(com_list) == 0:
        print('No serial devices connected')
        return 0
    else:
        which_serial = int(input("Choose from 1-" + str(len(com_list))+" -> "))
        adress = str(com_list[which_serial - 1])
        master = mavutil.mavlink_connection(adress, baud=baudRate)
        wait_heartbeat(master)
        print("Sending all message types")
        return 

connect_serial(baudRate)

