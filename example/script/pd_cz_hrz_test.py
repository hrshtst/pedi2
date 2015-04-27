#!/usr/bin/env python
# -*- coding: utf-8 -*-

from datautil import Data
from plotutil import FigureCreator
import matplotlib.pyplot as plt

LABEL_LIST = [
    't', 'xd', 'yd', 'x', 'y', 'vx', 'vy', 'xz', 'yz', 'theta'
]


def plot(data):
    fig = FigureCreator()
    fig.plot(data.x, data.y, label='COM')
    fig.plot(data.xz, data.yz, label='ZMP')
    fig.plot(data.xd, data.yd, label='Ref')
    fig.setTitleLabels(xlabel='x position', ylabel='y position')
    fig.makeAspectEqual()
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
    plot(data)

    if showflag:
        plt.show()

if __name__ == '__main__':
    main()
