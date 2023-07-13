from geographiclib.geodesic import Geodesic
from math import radians, cos, sqrt, sin, acos, degrees
from pymavlink import mavutil
import keyboard
import time
import os.path
from getch import getche, getch

# screen resolution 20x53 char

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

tracker_lat = 0.0
tracker_lon = 0.0
tracker_alt = 0
tracker_pos = geo_coord(tracker_lat, tracker_lon, tracker_alt)
precision = 10000000.0

def startup():
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("          __  ___             __ _        __         ")
    print("         /  |/  /___ _ _  __ / /(_)___   / /__       ")
    print("        / /|_/ // _ `/| |/ // // // _ \ /  '_/       ")
    print("       /_/  /_/ \_,_/ |___//_//_//_//_//_/\_\        ")
    print("        ______                 __                    ")
    print("       /_  __/____ ___ _ ____ / /__ ___  ____        ")
    print("        / /  / __// _ `// __//  '_// -_)/ __/        ")
    print("       /_/  /_/   \_,_/ \__//_/\_\ \__//_/           ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    
    time.sleep(2)

    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("                                                     ")
    print("          __  ___             __ _        __         ")
    print("         /  |/  /___ _ _  __ / /(_)___   / /__       ")
    print("        / /|_/ // _ `/| |/ // // // _ \ /  '_/       ")
    print("       /_/  /_/ \_,_/ |___//_//_//_//_//_/\_\        ")
    print("        ______                 __                    ")
    print("       /_  __/____ ___ _ ____ / /__ ___  ____        ")
    print("        / /  / __// _ `// __//  '_// -_)/ __/        ")
    print("       /_/  /_/   \_,_/ \__//_/\_\ \__//_/           ")
    print("                                                     ")
    print("                                                     ")
    print("                 Connect your GC now                 ")
    print("          and press any button to continue...        ")
    print("                                                     ")
    print("                                                     ")
    a = getch()
                                       
def wait_heartbeat(m):
    print("Waiting for heartbeat")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print("Connected")

def connect_serial():
    master = mavutil.mavlink_connection('tcp:0.0.0.0:5601')
    wait_heartbeat(master)
    return master

def wait_for_fix():
    starttime = time.time()
    print('Waiting for GPS fix')
    fixed = False
    while not fixed:
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        fix = gps_data.fix_type
        if fix >= 3:
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
    print("Press any key to save tracker's location")
    a = getch()
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
        print("Saved postion found. Do you want to use it?")
        print("A - Yes   B - No")
        selection = getch()
        if selection == 'a':
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

clear_screen()
startup()
clear_screen()
connection = connect_serial()
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
    if track_data["azi1"] < 0:
        azimuth = track_data["azi1"] + 360
    else:
        azimuth = track_data["azi1"]
    distance = sqrt((tracker_cart.x - drone_cart.x) ** 2 + (tracker_cart.y - drone_cart.y) ** 2 + (tracker_cart.z - drone_cart.z) ** 2)
    alt_acos = track_data["s12"]/distance

    if track_data["s12"]/distance >= 1:
        alt_acos = 1
    inclination = degrees(acos(alt_acos))
    diff_az = azimuth - buf_az
    diff_alt = inclination - buf_dist
    buf_az = azimuth
    buf_alt = inclination

    if distance > 1000:
        #print(f'{str(round(diff_az, 2)):5} {str(round(diff_alt, 2)):5}')
        print(f'Az.(deg): {str(round(azimuth, 2)):7} Dist.(km): {str(round(distance / 1000, 2)):7} Incl.(deg): {str(round(inclination, 2)):7}', end='\r')
    else:
        #print(f'{str(round(diff_az, 2)):5} {str(round(diff_alt, 2)):5}')
        print(f'Az.(deg): {str(round(azimuth, 2)):7} Dist.(m): {str(round(distance, 2)):7} Incl.(deg): {str(round(inclination, 2)):7}', end='\r')
