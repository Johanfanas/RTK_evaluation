#!/usr/bin/env python3

import math
import numpy as np

class NavConversion(object):

    kSemimajorAxis = 6378137.0
    kSemiminorAxis = 6356752.3142
    kFirstEccentricitySquared = 6.69437999014 * 0.001
    kSecondEccentricitySquared = 6.73949674228 * 0.001

    def __init__(self, latitude, longitude, altitude):
        
        self.init_geodetic_reference(latitude, longitude, altitude)

    def init_geodetic_reference(self, latitude, longitude, altitude):

        self.latitude0 = math.radians(latitude)
        self.longitude0 = math.radians(longitude)
        self.altitude0 = altitude

        (self.initial_ecef_x, self.initial_ecef_y, self.initial_ecef_z) = self.geodetic_to_ecef(latitude, longitude,
                                                                                                altitude)
        # Compute ECEF to NED.
        phiP = math.atan2(self.initial_ecef_z,
                            math.sqrt(math.pow(self.initial_ecef_x, 2) + math.pow(self.initial_ecef_y, 2)))
        self.ecef_to_ned_matrix = self.n_re(phiP, self.longitude0)

    def geodetic_to_enu(self, latitude, longitude, altitude):
        # Geodetic position to local ENU frame
        (x, y, z) = self.geodetic_to_ecef(latitude, longitude, altitude)
        (north, east, down) = self.ecef_to_ned(x, y, z)

        # Return East, North, Up coordinate.
        return east, north, -down

    def geodetic_to_ecef(self, latitude, longitude, altitude):
        # Convert geodetic coordinates to ECEF.
        # http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)
        xi = math.sqrt(1 - NavConversion.kFirstEccentricitySquared * math.sin(lat_rad) * math.sin(lat_rad))
        x = (NavConversion.kSemimajorAxis / xi + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (NavConversion.kSemimajorAxis / xi + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (NavConversion.kSemimajorAxis / xi * (1 - NavConversion.kFirstEccentricitySquared) + altitude) * math.sin(lat_rad)

        return x, y, z

    def ecef_to_ned(self, x, y, z):
        # Converts ECEF coordinate position into local-tangent-plane NED.
        # Coordinates relative to given ECEF coordinate frame.
        vect = np.array([0.0, 0.0, 0.0])
        vect[0] = x - self.initial_ecef_x
        vect[1] = y - self.initial_ecef_y
        vect[2] = z - self.initial_ecef_z
        ret = self.ecef_to_ned_matrix.dot(vect)
        n = ret[0]
        e = ret[1]
        d = -ret[2]

        return n, e, d

    def n_re(self, lat_radians, lon_radians):
        s_lat = math.sin(lat_radians)
        s_lon = math.sin(lon_radians)
        c_lat = math.cos(lat_radians)
        c_lon = math.cos(lon_radians)

        ret = np.eye(3)
        ret[0, 0] = -s_lat * c_lon
        ret[0, 1] = -s_lat * s_lon
        ret[0, 2] = c_lat
        ret[1, 0] = -s_lon
        ret[1, 1] = c_lon
        ret[1, 2] = 0.0
        ret[2, 0] = c_lat * c_lon
        ret[2, 1] = c_lat * s_lon
        ret[2, 2] = s_lat

        return ret