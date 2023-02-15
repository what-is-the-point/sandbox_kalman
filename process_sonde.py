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
from binascii import *

from utilities import RadioSonde as rs
from utilities import plotting

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
    print(df)
    df.drop_duplicates(subset=['datetime'], keep='first', inplace=True, ignore_index=True)
    print(df.info())
    print(df.head(20))
    print(df.name)

    #plotting.PlotAltitude(df,cfg, idx=1)
    #plotting.PlotTemperature(df,cfg, idx=2)
    plotting.PlotAltitudeTemperature(df,cfg, idx=2)
    plotting.PlotTemperatureVsAltitude(df,cfg, idx=2)
    plotting.PlotAltitudeVsTemperature(df,cfg, idx=2)
    plotting.PlotBattery(df,cfg, idx=2)
    plotting.PlotBatteryTemperature(df,cfg, idx=2)

    sys.exit()




