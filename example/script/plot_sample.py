#!/usr/bin/env python
# -*- coding: utf-8 -*-

from datautil import Data
from plotutil import FigureCreator
import matplotlib.pyplot as plt

LABEL_LIST = [
    't', 'dt',
    'xd', 'yd', 'zd', 'thetad',
    'x', 'y', 'z', 'theta'
]


def plotX(data):
    fig = FigureCreator()
    fig.plot(data.t, data.x, label='COM x')
    fig.setTitleLabels(xlabel='time [s]', ylabel='x position [m]')
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
    plotX(data)

    if showflag:
        plt.show()

if __name__ == '__main__':
    main()
