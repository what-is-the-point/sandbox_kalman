#!/usr/bin/env python3
"""
Aircraft data processing functions
"""

from math import *
import datetime
import numpy as np

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

