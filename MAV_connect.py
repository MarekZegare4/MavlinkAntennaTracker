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
from pick import pick
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
    name_list = []
    for p in ports:
        com_list.append(p.device)
        name_list.append(p.description)
    if len(com_list) == 0:
        print('No serial devices connected')
        return 0
    else:
        title = 'Select port'
        option, index = pick(name_list, title)
        print(index)
        adress = str(com_list[index])
        master = mavutil.mavlink_connection(adress, baud=baudRate)
        print(master)
        wait_heartbeat(master)
        print("Sending all message types")
        return 1

connect_serial(baudRate)

