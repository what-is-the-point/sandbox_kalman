#!/usr/bin/env python3

"""
RadioSonde data processing functions
"""


import math
import datetime
import sondehub
import os, sys

import pandas as pd
import numpy as np
import sondehub

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

def ImportRadioSondeFile(data_path, serial):
    fn = ".".join([serial,'log'])
    fp = "/".join([data_path,fn])

    if not os.path.isfile(fp):
        print("Data Log does not exist: {:s}".format(fp))
        print("Check path, serial, file names")
        print("Exitting...")
        sys.exit()
    print("Importing Radio Sonde Log file: {:s}".format(fp))

    df = pd.read_json(fp, lines=True)
    original_count = len(df)
    df.drop_duplicates(subset=['datetime'], keep='first', inplace=True, ignore_index=True)
    filtered_count = len(df)
    print("Original Data Point Count: {:d}".format(original_count))
    print("Filtered Data Point Count: {:d}".format(filtered_count))
    return df


    # with open(fp, 'r') as f:
    #     data = f.read()
    #     # data = data.strip().split("\n")

    # print (data)
    # df = pd.read_json(data, lines=True)
    # print(df)
    # # return df




def on_message(msg):
    # print(type(msg), len(msg))
    # print(json.dumps(msg, indent=4))
    global cnt
    cnt = cnt + 1
    ts = datetime.datetime.utcnow()
    rae = razel.RAZEL(obs['lat'], obs['lon'], obs['alt']/1000.0,
                      msg['lat'], msg['lon'], msg['alt']/1000.0)
    
    print("MESSAGE INFO:", msg['serial'])
    print("        counter: {:d}".format(cnt))
    print("   Sonde Serial:", msg['serial'])
    print(" Sonde Datetime:", msg['datetime'])
    print("  Received Time:", msg['time_received'])
    print("            Now:", datetime.datetime.strftime(ts, "%Y-%m-%dT%H:%M:%S.%fZ"))
    print("      Sonde Loc:", msg['lat'], msg['lon'], msg['alt'])
    print("RECEIVER INFO")
    print("       Receiver:", msg['uploader_callsign'])
    print("   Receiver Loc:", msg['uploader_position'], msg['uploader_alt'])
    print("   Receiver Ant:", msg['uploader_antenna'])
    print("       SNR [dB]:", msg['snr'])
    print("RADIOSONDE INFO:")
    print("   Altitude [km]: {:3.3f}".format(msg['alt']/1000.0))
    # print("V Velocity [m/s]: {:+3.3f}".format(msg['vel_v']))
    if msg['vel_v'] < 0.0:
        print("V Velocity [m/s]: {:+3.3f}".format(msg['vel_v']), "DESCENDING")
    else:
        print("V Velocity [m/s]: {:+3.3f}".format(msg['vel_v']), "Ascending")
    print("H Velocity [m/s]: {:3.3f}".format(msg['vel_h']))
    print("   Heading [deg]: {:3.3f}".format(msg['heading']))
    if 'temp' in msg.keys(): 
        print(" Temperature [C]: {:3.3f}".format(msg['temp']))
    if 'batt' in msg.keys(): 
        print("     Battery [V]: {:3.3f}".format(msg['batt']))
    if 'sats' in msg.keys(): 
        print("  Satellites [#]: {:d}".format(msg['sats']))
    print("OBSERVER INFO: {:s}".format(obs['name']))
    print("    Observer Loc:", obs['lat'], obs['lon'], obs['alt'])
    print("     Range  [km]: {:3.3f}".format(rae['rho']))
    print("   Azimuth [deg]: {:3.3f}".format(rae['az']))
    print(" Elevation [deg]: {:3.3f}".format(rae['el']))
    
    print()