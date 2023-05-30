import math
import numpy as np

tracker_lat = 53.481254
tracker_lon = 14.708217
tracker_alt = 0.0

def EarthRadiusInMeters(lat):
    a = 6378137.0
    b = 6356752.3
    cos = math.cos(np.deg2rad(lat))
    sin = math.sin(np.deg2rad(lat))
    t1 = a * a * cos
    t2 = b * b * sin
    t3 = a * cos
    t4 = b * sin
    return math.sqrt((t1*t1 + t2*t2) / (t3*t3 + t4*t4))

def cart(lat, lon, alt):
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    flat_factor = 1/298.257223563
    e_squared = 2*flat_factor - math.pow(flat_factor, 2)
    a = 6378137.0
    N = a/(math.sqrt(1-e_squared*math.pow(math.sin(lat_rad), 2)))
    x = (N + alt)*math.cos(lat_rad)*math.cos(lon_rad)
    y = (N + alt)*math.cos(lat_rad)*math.sin(lon_rad)
    z = ((1-e_squared)*N + alt)*math.sin(lat_rad)
    return x, y, z

def rotate_globe(lat, lon, alt):
    brp = cart(lat, lon - tracker_lon, alt)
    acos = math.cos(tracker_lat)
    asin = math.sin(tracker_lat)
    bx = (brp[0] * acos) - (brp[2] * asin)
    by = brp[1]
    bz = (brp[0] * asin) + (brp[2] * acos)
    return bx, by, bz


def calculate_azimuth():
    a = cart(53.481254, 14.708217, 0.0)
    b = rotate_globe(53.481322, 14.709963, 100) #prawo
    #b = cart(53.482147, 14.708025, 0) #góra
    x = b[0] - a[0]
    y = b[1] - a[1]
    z = b[2] - a[2]
    azimuth = math.degrees(math.atan(y/x))
    print(azimuth)

calculate_azimuth()