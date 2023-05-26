#  ====================================================
# || 1. TX -> Tracker
# || 
# || 2. TX -> Tracker -> GCS
# ||
# || 3. TX -> Tracker -> GCS
# ||       <- RTK
# ||
# ||
# ||
# ||
#  ====================================================

from pymavlink import mavutil
import serial.tools.list_ports
from pick import pick
import time
import os.path

baudRate = 9600
ports = serial.tools.list_ports.comports()

tracker_lat = 0
tracker_lon = 0
tracker_alt = 0

def wait_heartbeat(m):
    print("Waiting for APM heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Connected")

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
        return master

def wait_for_fix():
    starttime = time.time()
    print('Waiting for GPS fix')
    fixed = False
    while not fixed:
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        fix = gps_data.fix_type
        if fix == 3:
            fixed = True
        time.sleep(1.0 - ((time.time() - starttime) % 1.0))
    
def set_tracker_pos(tracker_lat, tracker_lon, tracker_alt):
    path = './LastPos.txt'
    is_LastPos = os.path.isfile(path)
    if is_LastPos == True:
        title = ("Saved postion found. Do you want to use it?")
        options = ['Yes','No']
        option, index = pick(options, title)
        if index == 0:
            file = open("LastPos.txt", "r")
            tracker_lat = int(file.readline())
            tracker_lon = int(file.readline())
            tracker_alt = int(file.readline())
            file.close()
        else:
            wait_for_fix()
            input("Press any button to save tracker's location")
            gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
            tracker_lat = gps_data.lat
            tracker_lon = gps_data.lon
            tracker_alt = gps_data.alt
            LastPos = (str(tracker_lat)+'\n'+str(tracker_lon)+'\n'+str(tracker_alt)+'\n')
            file = open("LastPos.txt", "w")
            file.write(LastPos)
            file.close()
    else:
        wait_for_fix()
        input("Press any button to save tracker's location")
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        tracker_lat = gps_data.lat
        tracker_lon = gps_data.lon
        tracker_alt = gps_data.alt
        LastPos = (str(tracker_lat)+'\n'+str(tracker_lon)+'\n'+str(tracker_alt)+'\n')
        file = open("LastPos.txt", "w")
        file.write(LastPos)
        file.close()

connection = connect_serial(baudRate)
set_tracker_pos(tracker_lat, tracker_lon, tracker_alt)
   
