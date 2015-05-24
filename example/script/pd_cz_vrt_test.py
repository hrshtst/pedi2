#!/usr/bin/env python
# -*- coding: utf-8 -*-

from datautil import Data
from plotutil import FigureCreator
import matplotlib.pyplot as plt

LABEL_LIST = [
    't', 'z', 'vz', 'az'
]


def plot(data):
    fig = FigureCreator()
    fig.plot(data.t, data.z)
    fig.setTitleLabels(xlabel='time', ylabel='z')


def usage(prog):
    print """\
usage:
    %s [-hx] datafile""" % prog


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
        elif arg != sys.argv[0]:
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
