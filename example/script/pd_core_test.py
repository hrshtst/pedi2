#!/usr/bin/env python
# -*- coding: utf-8 -*-

from datautil import Data
from plotutil import FigureCreator
import matplotlib.pyplot as plt

LABEL_LIST = [
    't', 'dt',
    'xd', 'yd', 'zd', 'thetad',
    'x', 'y', 'z', 'theta',
    'vx', 'vy', 'vz',
    'ax', 'ay', 'az',
    'xz', 'yz', 'zz',
    'ud', 'deltau', 'vu',
    'wd', 'deltaw', 'vw',
    'refx', 'refy', 'refz',
    'refvx', 'refvy', 'refvz',
    'refax', 'refay', 'refaz',
    'refxz', 'refyz', 'refzz',

    'lfpx', 'lfpy', 'lfpz',
    'lfpdx', 'lfpdy', 'lfpdz',
    'lfrefpx', 'lfrefpy', 'lfrefpz',
    'lfax', 'lfay', 'lfaz',
    'lfadx', 'lfady', 'lfadz',
    'lfrefax', 'lfrefay', 'lfrefaz',
    'lfphi', 'lfh',
    'lfpzre', 'lfpzim',
    'lfphase', 'lfzrefz',

    'rfpx', 'rfpy', 'rfpz',
    'rfpdx', 'rfpdy', 'rfpdz',
    'rfrefpx', 'rfrefpy', 'rfrefpz',
    'rfax', 'rfay', 'rfaz',
    'rfadx', 'rfady', 'rfadz',
    'rfrefax', 'rfrefay', 'rfrefaz',
    'rfphi', 'rfh',
    'rfpzre', 'rfpzim',
    'rfphase', 'rfzrefz'
]


def plotXY(data):
    fig = FigureCreator()
    fig.plot(data.x, data.y, label='COM')
    fig.plot(data.xz, data.yz, label='ZMP')
    fig.plot(data.xd, data.yd, label='Ref')
    fig.setTitleLabels(xlabel='x position [m]', ylabel='y position [m]')
    fig.makeAspectEqual()
    fig.legend()


def plotZ(data):
    fig = FigureCreator()
    fig.plot(data.t, data.z, label='COM')
    fig.plot(data.t, data.zd, label='Ref')
    fig.setTitleLabels(xlabel='time [s]', ylabel='z position [m]')
    fig.setYLimit([0.2, 0.35])
    fig.legend()


def usage(prog):
    print "%s [-hx] datafile" % prog


def main():
    import sys
    showflag = True
    datafile = None

    # parse arguments
    for arg in sys.argv:
        if arg.startswith('-'):
            if arg == '-h':
                usage(sys.argv[0])
                sys.exit(0)
            if arg == '-x':
                showflag = False
        else:
            datafile = arg
    if datafile is None:
        usage(sys.argv[0])
        sys.exit(1)

    # plot
    data = Data(datafile, LABEL_LIST)
    plotXY(data)
    plotZ(data)

    if showflag:
        plt.show()

if __name__ == '__main__':
    main()
