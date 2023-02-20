#!/usr/bin/env python3

import socket
import os
import string
import sys
import time
import argparse
import datetime
import json, csv, yaml
import subprocess
import sondehub
import math
import copy
from binascii import *
import pandas as pd

from utilities import RadioSonde as rs
from utilities import plotting
from utilities import astro

obs = {}
cnt = 0



def import_configs_yaml(args):
    ''' setup configuration data '''
    fp_cfg = '/'.join([args.cfg_path,args.cfg_file])
    print (fp_cfg)
    if not os.path.isfile(fp_cfg) == True:
        print('ERROR: Invalid Configuration File: {:s}'.format(fp_cfg))
        sys.exit()
    print('Importing configuration File: {:s}'.format(fp_cfg))
    with open(fp_cfg, 'r') as yaml_file:
        cfg = yaml.safe_load(yaml_file)
        yaml_file.close()

    if cfg['main']['base_path'] == 'cwd':
        cfg['main']['base_path'] = os.getcwd()
    cfg['serial'] = args.serial
    cfg['plot']=args.plot
    return cfg


if __name__ == '__main__':
    """ Main entry point to start the service. """
    startup_ts = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    #--------START Command Line argument parser----------------
    parser = argparse.ArgumentParser(description="Simple RadioSonde Tracker",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    cwd = os.getcwd()
    cfg_fp_default = '/'.join([cwd, 'config'])
    cfg = parser.add_argument_group('Configuration File')
    cfg.add_argument('--cfg_path',
                       dest='cfg_path',
                       type=str,
                       default='/'.join([os.getcwd(), 'config']),
                       help="Configuration File Path",
                       action="store")
    cfg.add_argument('--cfg_file',
                       dest='cfg_file',
                       type=str,
                       default="sonde_config.yaml",
                       help="Configuration File",
                       action="store")
    cfg.add_argument('--serial',
                       dest='serial',
                       type=str,
                       default=None,
                       help="Radiosonde Serial",
                       action="store")
    cfg.add_argument('--plot',
                       dest='plot',
                       type=bool,
                       default=False,
                       help="Plot Radiosonde Data",
                       action="store")
    args = parser.parse_args()
    #--------END Command Line argument parser-----------------
    #print(chr(27) + "[2J")
    subprocess.run(["reset"])
    #print(sys.path)
    # print(args.serial)
    # if not args.serial:
    #     print("Please enter a target Radiosonde Serial Number...")
    #     sys.exit()


    cfg = import_configs_yaml(args)
    print(cfg)
    obs = cfg['observer']

    #Configure Serial Number
    #  if no serial specified on command line, use the first file
    if cfg['serial'] == None:
        print("No Serial number specified, using first file in datapath...")
        filenames = next(os.walk(cfg['main']['data_path']), (None, None, []))[2]
        cfg['serial'] = filenames[0].split(".")[0]

    df = rs.ImportRadioSondeFile(cfg['main']['data_path'], cfg['serial'])
    df.name = cfg['serial']
    print(df.info())
    print(df.head(20))
    print(df.name)

    if cfg['plot']:
        #plotting.PlotAltitude(df,cfg, idx=1)
        #plotting.PlotTemperature(df,cfg, idx=2)
        # plotting.PlotAltitudeTemperature(df,cfg, idx=2)
        # plotting.PlotTemperatureVsAltitude(df,cfg, idx=2)
        # plotting.PlotAltitudeVsTemperature(df,cfg, idx=2)
        # plotting.PlotBattery(df,cfg, idx=2)
        # plotting.PlotBatteryTemperature(df,cfg, idx=2)
        plotting.PlotHorizontalVelocityVsAltitude(df,cfg, idx=2)

    keep_list = ['datetime', 'lat', 'lon', 'alt', 'heading', 'vel_h', 'vel_v']
    df_meas = df[keep_list]
    
    print(df_meas.info())

    deg2rad = math.pi / 180.0
    # result = razel.LLH_To_ECEF(df['lat'] * deg2rad, df['lon']*deg2rad, df['alt']/1000.0)

    df_ecef = pd.DataFrame(df_meas.apply(lambda x: astro.LLH_To_ECEF(x['lat']*deg2rad, 
                                                                     x['lon']*deg2rad, 
                                                                     x['alt']/1000.0), axis=1).to_list(), 
                                                                     columns=['i','j','k'])
  
    df_dot = pd.DataFrame(df_meas.apply(lambda x: astro.SpeedHeading_To_ECEF(x['lat']*deg2rad, 
                                                                    x['lon']*deg2rad, 
                                                                    x['vel_h'],
                                                                    x['vel_v'],
                                                                    x['heading']*deg2rad,
                                                                    km=True), axis=1).to_list(), 
                                                                    columns=['i_dot','j_dot','k_dot'])

    df_meas = pd.concat([df_meas, df_ecef, df_dot], axis=1, join='inner')
    df_meas.name = cfg['serial']

    print(df_meas)
    print(df_meas.name)
    # df_meas['i'] a[0]
    # df_measa['i']['j'], df_meas['k'])
    




    sys.exit()




