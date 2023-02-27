#!/usr/bin/env python3
"""
Plotting Utilities for Kalman Filtering
"""

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


def PlotErrorCovariance(df, cfg, idx=0):
    # plt.rcParams['text.usetex'] = True
    pts = 100
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
    #Configure Labels and Title
    ax1.set_xlabel('step')
    ax1.set_ylabel('Position Error Covariance')
    ax2.set_ylabel('Velocity Error Covariance')
    # title = "Error Covariance, {:s}".format(coord)
    title = df.name
    ax1.set_title(title)
    ax1.plot(df.index[0:pts], df['i'][0:pts], linestyle='-', color='r', label="i [m]")
    ax1.plot(df.index[0:pts], df['j'][0:pts], linestyle='-', color='g', label="j [m]")
    ax1.plot(df.index[0:pts], df['k'][0:pts], linestyle='-', color='b', label="k [m]")
    ax2.plot(df.index[0:pts], df['i_dot'][0:pts], linestyle='--', color='r', label="i_dot [m/s]")
    ax2.plot(df.index[0:pts], df['j_dot'][0:pts], linestyle='--', color='g', label="j_dot [m/s]")
    ax2.plot(df.index[0:pts], df['k_dot'][0:pts], linestyle='--', color='b', label="k_dot [m/s]")
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper right')
    plt.show()

def PlotVelocity(k_df, m_df, k1_df, coord, cfg, idx=0):
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
    ax1.set_xlabel('item')
    ax1.set_ylabel('velocity [m/s]')
    title = "Velocity, {:s}".format(coord)
    ax1.set_title(title)
    ax1.plot(k_df.index, k_df[coord], linestyle='-', color='r', label="estimate, {:s} [m]".format(coord))
    ax1.plot(m_df.index, m_df[coord], linestyle='-', color='g', label="measurement, {:s} [m]".format(coord))
    ax1.plot(k1_df.index, k1_df[coord], linestyle='-', color='b', label="prediction, {:s} [m]".format(coord))
    ax1.legend()
    plt.show()

def PlotPosition(k_df, m_df, k1_df, coord, cfg, idx=0):
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
    ax1.set_xlabel('item')
    ax1.set_ylabel('position [m]')
    title = "Position, {:s}".format(coord)
    ax1.set_title(title)
    ax1.plot(k_df.index, k_df[coord], linestyle='-', color='r', label="estimate, {:s} [m]".format(coord))
    ax1.plot(m_df.index, m_df[coord], linestyle='-', color='g', label="measurement, {:s} [m]".format(coord))
    ax1.plot(k1_df.index, k1_df[coord], linestyle='-', color='b', label="prediction, {:s} [m]".format(coord))
    ax1.legend()
    plt.show()


