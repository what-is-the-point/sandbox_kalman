#!/usr/bin/env python3
from math import *
from datetime import datetime as date
import numpy as np

deg2rad = pi / 180.0
rad2deg = 180.0 / pi
c       = float(299792458)  #[m/s], speed of light
R_e     = 6378.137 				#Earth Radius, in kilometers
e_e     = 0.081819221456	    #Eccentricity of Earth

adsb_msg = {
    "icao": None,      # MLAT, MSG, CLK, STA, AIR, ID, SEL
    "tx_type": 0,
    "callsign":'',       # transmission type
    "azimuth":0.0,    # String. Database session record number.
    "elevation":0.0,   # String. Database aircraft record number.
    "range":0.0,     # String. 24-bit ICACO ID, in hex.
    "geo_alt":0.0,     # String. Database flight record number.
    "baro_alt":0.0,
    "vert_rate":0.0,
    "speed":0.0,# String. Date the message was generated.
    "track":0.0,# String. Time the message was generated.
    "msg_src":'',   # String. Date the message was logged.
    "msg_cnt":0,    # String. Time the message was logged.
    "date_last":'', #last date stamp
    "time_last":'', #last time stamp
    "pos_date":None, #last date stamp
    "pos_time":None, #last time stamp
    "age":0.0, #seconds since last time stamp
    "az_rate":0.0,
    "el_rate":0.0,
    "range_rate":0.0,
}

sonde_msg = {
    "software_name": "radiosonde_auto_rx",              #Software Name
    "software_version": "1.6.0",                        #Software Version
    "uploader_callsign": "KN4GDX_1_AUTO_RX",            #Uploader Callsign
    "uploader_position": "38.783,-77.774",              #Uploader Location, lat lon
    "uploader_antenna": "Vertical UHF Base",            #Uploader Antenna
    "time_received": "2023-02-12T17:40:21.838783Z",     #Time the packet was received at the uploader's station
    "datetime": "2023-02-12T17:40:20.000000Z",          #Timestamp of the content of the packet (source = sonde)
    "manufacturer": "Graw", 
    "type": "DFM",
    "subtype": "DFM17",
    "serial": "22049077", # Serial Number, this is what is filtered on when subscribing
    "dfmcode": "0xB",
    "frame": 1360258820,  # unique frame number, might be linux epoch of datetime field.
    "lat": 37.97852,    # latitude, decimal degrees
    "lon": -79.96687,   # longitude, decimal degrees
    "alt": 6396.2,      # altitude, meters
    "temp": -21.1,      # temperature, deg Celsius
    "vel_v": -1.94,     # vertical velocity, meters / second
    "vel_h": 6.35,      # horizontal velocity, meters / second
    "heading": 334.57,  # heading, degrees in azimuth relative to true north
    "sats": 20,         # number of satellites used in position fix from onboard GPS
    "batt": 4.5,        # battery voltage
    "frequency": 404.011, # received frequency, MHz
    "ref_position": "GPS",
    "ref_datetime": "UTC",
    "snr": 6.1, #snr in dB at uploader station
    "user-agent": "Amazon CloudFront",
    "position": "37.97852,-79.96687",
    "upload_time_delta": -0.888,
    "uploader_alt": 210.0
}

#--Range Calculations Functions------
def SpeedHeading_To_ECEF(lat, lon, vel_h, vel_v, heading, km = False):
    # INPUT:
    #     lat - latitude [radians]
    #     lon - longitude in [radians]
	#     vel_v - vertical velocity [m/s]
	#     vel_h - horizontal vocity [m/s]
    #     heading - heading relative to North
    #     km - False = m/s, True = km/s
    # INTERNAL:
    #     vel_mag - velocity magnitude, along trajectory
    #     gamma - climb angle
    
    vel_mag = sqrt(pow(vel_h,2)+pow(vel_v,2))
    gamma = asin(vel_v/vel_mag) #climb angle
    clat = cos(lat)
    slat = sin(lat)
    clon = cos(lon)
    slon = sin(lon)
    chead = cos(heading)
    shead = sin(heading)
    cgamma = cos(gamma)
    sgamma = sin(gamma)

    i_dot = vel_mag*(-1*slon*cgamma*shead - clon*slat*cgamma*chead + clon*clat*sgamma)
    j_dot = vel_mag*(clon*cgamma*shead    - slon*slat*cgamma*chead + slon*clat*sgamma)
    k_dot = vel_mag*(0                    + clat*cgamma*shead      + slat*sgamma)

    if km:
        return i_dot/1000.0, j_dot/1000.0, k_dot/1000.0
    else:
        return i_dot, j_dot, k_dot


def LLH_To_ECEF(lat, lon, h):
	#INPUT:
	#	h   - height above ellipsoid (MSL), km
	#	lat - geodetic latitude, in radians
	#	lon - longitude, in radians
    C_e = R_e / sqrt(1 - pow(e_e, 2) * pow(sin(lat),2))
    S_e = C_e * (1 - pow(e_e, 2))
    r_i = (C_e + h) * cos(lat) * cos(lon)
    r_j = (C_e + h) * cos(lat) * sin(lon)
    r_k = (S_e + h) * sin(lat)
    return r_i, r_j, r_k

def RAZEL(lat1, lon1, h1, lat2, lon2, h2):
	#Calculates Range, Azimuth, Elevation in SEZ coordinate frame from SITE to UAV
	#INPUT:
	# lat1, lon1, h1 - Site Location
	# lat2, lon2, h2 - UAV location
    # lats and lons in degrees
    # h in km
	#OUTPUT:
	# Slant Range, Azimuth, Elevation

    lat1 = lat1 * deg2rad
    lon1 = lon1 * deg2rad
    lat2 = lat2 * deg2rad
    lon2 = lon2 * deg2rad

    r_site   = np.array(LLH_To_ECEF(lat1, lon1, h1))
    r_uav    = np.array(LLH_To_ECEF(lat2, lon2, h2))
    rho_ecef = r_uav - r_site

    ECEF_2_SEZ_ROT = np.array([[sin(lat1) * cos(lon1), sin(lat1) * sin(lon1), -1 * cos(lat1)],
                               [-1 * sin(lon1)       , cos(lon1)            , 0             ],
                               [cos(lat1) * cos(lon1), cos(lat1) * sin(lon1), sin(lat1)     ]])

    rho_sez = np.dot(ECEF_2_SEZ_ROT ,rho_ecef)
    rho_mag = np.linalg.norm(rho_sez)
    el = asin(rho_sez[2]/rho_mag) * rad2deg
    az_asin = asin(rho_sez[1]/sqrt(pow(rho_sez[0],2)+pow(rho_sez[1], 2))) * rad2deg
    az_acos = acos(-1 * rho_sez[0]/sqrt(pow(rho_sez[0],2)+pow(rho_sez[1], 2))) * rad2deg
    #print az_asin, az_acos
    #Perform Quadrant Check:
    if (az_asin >= 0) and (az_acos >= 0): az = az_acos# First or Fourth Quadrant
    else: az = 360 - az_acos# Second or Third Quadrant
    #This is the Azimuth From the TARGET to the UAV
    #Must convert to Back Azimuth:
    back_az = az + 180
    if back_az >= 360:  back_az = back_az - 360
    #print az, back_az
    # rho_mag in kilometers, range to target
    # back_az in degrees, 0 to 360
    # el in degrees, negative = down tilt, positive = up tilt
    razel = {
        "rho": rho_mag,
        "az":az,
        "el":el
    }
    # return rho_mag, az, el
    return razel
