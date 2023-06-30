from geographiclib.geodesic import Geodesic
from math import radians, cos, sqrt, sin, acos, degrees

tracker_lat = 53.47421158
tracker_lon = 14.53649923
tracker_alt = 0

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

class geo_data():
    def __init__(self, lat1, lon1, lat2, lon2, a12, s12, azi1, azi2):
        self.lat1 = lat1 #  latitude of the first point in degrees
        self.lon1 = lon1 #  longitude of the first point in degrees
        self.azi1 = azi1 #  azimuth at the first point in degrees
        self.lat2 = lat2 #  latitude of the second point in degrees
        self.lon2 = lon2 #  longitude of the second point in degrees
        self.azi2 = azi2 #  azimuth at the second point in degrees
        self.s12 = s12   #  the distance from the first point to the second in meters 
        self.a12 = a12   #  spherical arc length from the first point to the second in degrees

def cart(geo_coord): # Conversion to cartesian coordinates
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

tracker_pos = geo_coord(tracker_lat, tracker_lon, tracker_alt)
gora = geo_coord(53.476351, 14.537121, 0)
prawo = geo_coord(53.474346, 14.540168, 120)
plane_pos = prawo

gora_cart = cart_coord(*cart(gora))
plane_cart = cart_coord(*cart(plane_pos))
tracker_cart = cart_coord(*cart(tracker_pos))

track_data = Geodesic.WGS84.Inverse(tracker_pos.lat, tracker_pos.lon, prawo.lat, prawo.lon)

distance = sqrt((tracker_cart.x - plane_cart.x) ** 2 + (tracker_cart.y - plane_cart.y) ** 2 + (tracker_cart.z - plane_cart.z) ** 2)
altitude = acos(track_data["s12"]/distance)

print("Azimuth: " + str(track_data["azi1"]))
print("Altitude: " + str(degrees(altitude)))
print("XY_Distance: " + str(track_data["s12"]) + " m")
print("XYZ_Distance: " + str(distance) + " m")
