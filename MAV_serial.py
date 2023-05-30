#  ====================================================
# || 1. TX -> Tracker
# || 
# || 2. TX -> Tracker -> GCS
# ||
# || 3. TX -> Tracker -> GCS
# ||       <- RTK
#  ====================================================

from pymavlink import mavutil
import serial.tools.list_ports
from pick import pick
import time
import os.path
import math
from pyproj import Geod
import numpy as np

baudRate = 9600
ports = serial.tools.list_ports.comports()

tracker_lat = 53.481169
tracker_lon = 14.708257
tracker_alt = 0.0

def startup():
    print("   __  ___             __ _        __  ")
    print("  /  |/  /___ _ _  __ / /(_)___   / /__")
    print(" / /|_/ // _ `/| |/ // // // _ \ /  '_/")
    print("/_/  /_/ \_,_/ |___//_//_//_//_//_/\_\ ")
    print(" ______                 __             ")
    print("/_  __/____ ___ _ ____ / /__ ___  ____ ")
    print(" / /  / __// _ `// __//  '_// -_)/ __/ ")
    print("/_/  /_/   \_,_/ \__//_/\_\ \__//_/    ")
    time.sleep(2)
                                       
def wait_heartbeat(m):
    print("Waiting for heartbeat")
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
        adress = str(com_list[index])
        master = mavutil.mavlink_connection(adress, baud=baudRate)
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
    
def get_gps_data():
    global tracker_lat
    global tracker_lon
    global tracker_alt
    lat_buf = 0
    lon_buf = 0
    alt_buf = 0
    for i in range(10):
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        lat_buf += gps_data.lat
        lon_buf += gps_data.lon
        alt_buf += gps_data.alt
        print(i, gps_data)
    tracker_lat = int(lat_buf / 10)
    tracker_lon = int(lon_buf / 10)
    tracker_alt = int(alt_buf / 10)
    print(tracker_alt, tracker_lat, tracker_lon) # <- Debug

def save_tracker_pos():
    input("Press any button to save tracker's location")
    get_gps_data()
    LastPos = (str(tracker_lat)+'\n'+str(tracker_lon)+'\n'+str(tracker_alt)+'\n')
    file = open("LastPos.txt", "w")
    file.write(LastPos)
    file.close()

def set_tracker_pos():
    global tracker_lat
    global tracker_lon
    global tracker_alt
    path = './LastPos.txt'
    is_LastPos = os.path.isfile(path)
    if is_LastPos == True:
        title = ("Saved postion found. Do you want to use it?")
        options = ['Yes','No']
        picked = pick(options, title)
        if picked[1] == 0:
            file = open("LastPos.txt", "r")
            tracker_lat = int(file.readline())
            tracker_lon = int(file.readline())
            tracker_alt = int(file.readline())
            file.close()
            print("Position set")
        else:
            wait_for_fix()
            save_tracker_pos()
    else:
        wait_for_fix()
        save_tracker_pos()

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def cart(lat, lon, alt):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    flat_factor = 1/298.257223563
    e_squared = 2*flat_factor - flat_factor**2
    a = 6378137.0
    N = a/(math.sqrt(1-e_squared*math.sin(lat_rad)**2))
    x = (N + alt)*math.cos(lat_rad)*math.cos(lon_rad)
    y = (N + alt)*math.cos(lat_rad)*math.sin(lon_rad)
    z = ((1-e_squared)*N + alt)*math.sin(lat_rad)
    return x, y, z

def angles(lat, lon, alt):
    g = Geod(ellps='WGS84')
    az1, az2, distance = g.inv(lat, lon, tracker_lat, tracker_lon)
    azimuth = 90 - az2
    if (azimuth < 0.0):
        azimuth += 360
    if (azimuth > 360):
        azimuth -= 360
    mav_coord = cart(lat, lon, alt)
    track_coord = cart(tracker_lat, tracker_lon, tracker_alt)
    dist = math.sqrt((mav_coord[0] - track_coord[0])**2 + (mav_coord[1] - track_coord[1])**2 + (mav_coord[2] - track_coord[2])**2)
    h = alt - tracker_alt
    altitude = math.degrees(math.asin(h/dist))
    return azimuth, dist, altitude

def print_all():
    time.sleep(1)
    print(tracker_lat, end='\r')
    time.sleep(1)
    print(tracker_lon, end='\r')
    time.sleep(1)
    print(tracker_alt, end='\r')
    time.sleep(1)

#--------------------------------PROGRAM-LOOP--------------------------------

#startup()
#clear_screen()
#connection = connect_serial(baudRate)
#clear_screen()
#set_tracker_pos()
#print_all() # <- Debug
print(angles(53.597937, 14.663008, 200))
   