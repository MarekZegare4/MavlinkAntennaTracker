from geographiclib.geodesic import Geodesic
from math import radians, cos, sqrt, sin, acos, degrees
from pymavlink import mavutil
import serial.tools.list_ports
from pick import pick
import time
import os.path

class geo_coord():
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class cart_coord():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

baudRate = 9600
ports = serial.tools.list_ports.comports()

tracker_lat = 53.47421158
tracker_lon = 14.53649923
tracker_alt = 0
tracker_pos = geo_coord(tracker_lat, tracker_lon, tracker_alt)
precision = 10000000.0

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
    
def get_gps_data():
    global tracker_pos
    lat_buf = 0
    lon_buf = 0
    alt_buf = 0
    for i in range(10):
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        lat_buf += gps_data.lat
        lon_buf += gps_data.lon
        alt_buf += gps_data.alt
        print(i, gps_data)
    tracker_pos.lat = float(lat_buf / (10 * precision))                                     # convert to degrees
    tracker_pos.lon = float(lon_buf / (10 * precision)) 
    tracker_pos.alt = float(alt_buf / (10 * 1000))                                          # convert from milimeters
    print(tracker_pos.lat, tracker_pos.lon, tracker_pos.alt)                                        # <- Debug

def save_tracker_pos(): 
    input("Press any button to save tracker's location")
    get_gps_data()
    LastPos = (str(tracker_pos.lat)+'\n'+str(tracker_pos.lon)+'\n'+str(tracker_pos.alt)+'\n')
    file = open("LastPos.txt", "w")
    file.write(LastPos)
    file.close()

def set_tracker_pos():
    global tracker_pos
    path = './LastPos.txt'
    is_LastPos = os.path.isfile(path)
    if is_LastPos == True:
        title = ("Saved postion found. Do you want to use it?")
        options = ['Yes','No']
        picked = pick(options, title)
        if picked[1] == 0:
            file = open("LastPos.txt", "r")
            tracker_pos.lat = float(file.readline())
            tracker_pos.lon = float(file.readline())
            tracker_pos.alt = float(file.readline())
            file.close()
            print("Position set")
            time.sleep(2)
        else:
            wait_for_fix()
            save_tracker_pos()
    else:
        wait_for_fix()
        save_tracker_pos()

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

# Conversion from WGS84 to cartesian coordinates
def cart(geo_coord): 
    lat_rad = radians(geo_coord.lat)
    lon_rad = radians(geo_coord.lon)
    flat_factor = 1/298.257223563
    e_squared = 2 * flat_factor - flat_factor ** 2
    a = 6378137.0
    N = a/(sqrt(1 - e_squared * sin(lat_rad) ** 2))
    x = (N + geo_coord.alt) * cos(lat_rad) * cos(lon_rad)
    y = (N + geo_coord.alt) * cos(lat_rad) * sin(lon_rad)
    z = ((1 - e_squared) * N + geo_coord.alt) * sin(lat_rad)
    return x, y, z


startup()
clear_screen()
connection = connect_serial(baudRate)
clear_screen()
set_tracker_pos()
clear_screen()

buf_az = 0.0
buf_dist = 0.0
buf_alt = 0.0

while True:
    try:
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
    except:
        print("Connection lost")
        break
    drone_pos = geo_coord(gps_data.lat / precision, gps_data.lon / precision, gps_data.alt / 1000)
    track_data = Geodesic.WGS84.Inverse(tracker_pos.lat, tracker_pos.lon, drone_pos.lat, drone_pos.lon)
    drone_cart = cart_coord(*cart(drone_pos))
    tracker_cart = cart_coord(*cart(tracker_pos))  

    if track_data["azi12"] < 0:
        azimuth = track_data["azi12"] + 360
    else:
        azimuth = track_data["azi12"]
    distance = sqrt((tracker_cart.x - drone_cart.x) ** 2 + (tracker_cart.y - drone_cart.y) ** 2 + (tracker_cart.z - drone_cart.z) ** 2)
    alt_acos = track_data["s12"]/distance

    if track_data["s12"]/distance >= 1:
        alt_acos = 1
    inclination = acos(alt_acos)
    diff_az = azimuth - buf_az
    diff_alt = inclination - buf_dist
    buf_az = azimuth
    buf_alt = inclination

    if distance > 1000:
        #print(f'{str(round(diff_az, 2)):5} {str(round(diff_alt, 2)):5}')
        print(f'Azimuth(deg): {str(round(azimuth, 2)):7} Distance(km): {str(round(distance / 1000, 2)):7} Inclination(deg): {str(round(inclination, 2)):7}')
    else:
        #print(f'{str(round(diff_az, 2)):5} {str(round(diff_alt, 2)):5}')
        print(f'Azimuth(deg): {str(round(azimuth, 2)):7} Distance(m): {str(round(distance, 2)):7} Inclination(deg): {str(round(inclination, 2)):7}', end='\r')


"""
gora = geo_coord(53.476351, 14.537121, 0)
prawo = geo_coord(53.474346, 14.540168, 120)
lewo = geo_coord(53.474372, 14.533977, 168)
plane_pos = lewo

gora_cart = cart_coord(*cart(gora))
plane_cart = cart_coord(*cart(plane_pos))
tracker_cart = cart_coord(*cart(tracker_pos))

track_data = Geodesic.WGS84.Inverse(tracker_pos.lat, tracker_pos.lon, plane_pos.lat, plane_pos.lon)

distance = sqrt((tracker_cart.x - plane_cart.x) ** 2 + (tracker_cart.y - plane_cart.y) ** 2 + (tracker_cart.z - plane_cart.z) ** 2)

alt_acos = track_data["s12"]/distance
if track_data["s12"]/distance >= 1:
    alt_acos = 1

altitude = acos(alt_acos)

print("Azimuth: " + str(track_data["azi1"]))
print("Altitude: " + str(degrees(altitude)))
print("XY_Distance: " + str(track_data["s12"]) + " m")
print("XYZ_Distance: " + str(distance) + " m")
"""