import sys
import os
import math
import string
import struct
import numpy as np
import scipy
import datetime as dt
import pytz
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from matplotlib.dates import DateFormatter
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import matplotlib.dates as mdates
import matplotlib.colors as colors


def PlotHorizontalVelocityVsAltitude(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    #Configure Labels and Title
    ax1.set_xlabel('Horizontal Velocity [m/s]')
    ax1.set_ylabel('Altitude [m]')
    title = "Horizontal Velocity Vs Altitude"
    ax1.set_title(title)
    ax1.plot(df['vel_h'], df['alt'], linestyle='-', color='b', label="Battery [V]")
    plt.show()


def PlotBatteryTemperature(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    ax2 = ax1.twinx()
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.set_ylim([3.0, 5.5])
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))

    #Configure Labels and Title
    ax1.set_xlabel('Time [UTC]')
    ax1.set_ylabel('Battery Voltage [V]')
    ax2.set_ylabel('Temperature [C]')
    title = "Battery & Temperature, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ln1 = ax1.plot(df['datetime'], df['batt'], linestyle='-', color='b', label="Battery [V]")
    ln2 = ax2.plot(df['datetime'], df['temp'], linestyle='-', color='r', label="Temperature [C]")
    lns = ln1+ln2
    lbls = [l.get_label() for l in lns]
    ax1.legend(lns, lbls)
    plt.show()

def PlotBattery(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.set_ylim([3.0, 5.5])
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))

    #Configure Labels and Title
    ax1.set_xlabel('Time [UTC]')
    ax1.set_ylabel('Battery Voltage [V]')
    title = "Battery, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ax1.plot(df['datetime'], df['batt'], linestyle='-')
    plt.show()


def PlotAltitudeVsTemperature(df, cfg, idx=0):
    xinch = 7
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    # ax2 = ax1.twinx()
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    # ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))

    #Configure Labels and Title
    ax1.set_ylabel('Altitude [m]')
    ax1.set_xlabel('Temperature [C]')
    title = "Altitude Vs Temperature, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ln1 = ax1.plot(df['temp'], df['alt'], linestyle='-', color='b')
    plt.show()

def PlotTemperatureVsAltitude(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    # ax2 = ax1.twinx()
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    # ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))

    #Configure Labels and Title
    ax1.set_xlabel('Altitude [m]')
    ax1.set_ylabel('Temperature [C]')
    title = "Temperature Vs Altitude, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ln1 = ax1.plot(df['alt'], df['temp'], linestyle='-', color='b')
    plt.show()

def PlotAltitudeTemperature(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    ax2 = ax1.twinx()
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))

    #Configure Labels and Title
    ax1.set_xlabel('Time [UTC]')
    ax1.set_ylabel('Altitude [m]')
    ax2.set_ylabel('Temperature [C]')
    title = "Altitude & Temperature, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ln1 = ax1.plot(df['datetime'], df['alt'], linestyle='-', color='b', label="Altitude [m]")
    ln2 = ax2.plot(df['datetime'], df['temp'], linestyle='-', color='r', label="Temperature [C]")
    lns = ln1+ln2
    lbls = [l.get_label() for l in lns]
    ax1.legend(lns, lbls)
    plt.show()

def PlotTemperature(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))

    #Configure Labels and Title
    ax1.set_xlabel('Time [UTC]')
    ax1.set_ylabel('Temperature [C]')
    title = "Temperature, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ax1.plot(df['datetime'], df['temp'], linestyle='-')
    plt.show()

def PlotAltitude(df, cfg, idx=0):
    xinch = 14
    yinch = 7
    fig1=plt.figure(idx, figsize=(xinch,yinch/.8))
    ax1 = fig1.add_subplot(1,1,1)
    #Configure Grids
    ax1.xaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.yaxis.grid(True,'major', linewidth=1)
    ax1.yaxis.grid(True,'minor')
    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d\n %H:%M:%S"))
    #Configure Labels and Title
    ax1.set_xlabel('Time [UTC]')
    ax1.set_ylabel('Altitude [m]')
    title = "Altitude, RadioSonde: {:s}".format(cfg['serial'])
    ax1.set_title(title)
    ax1.plot(df['datetime'], df['alt'], linestyle='-')
    plt.show()
