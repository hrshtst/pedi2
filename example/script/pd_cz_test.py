from __future__ import annotations

import matplotlib.pyplot as plt

from datautil import Data
from plotutil import FigureCreator

LABEL_LIST = [
    "t",
    "dt",
    "xd",
    "yd",
    "zd",
    "thetad",
    "x",
    "y",
    "z",
    "theta",
    "vx",
    "vy",
    "vz",
    "ax",
    "ay",
    "az",
    "xz",
    "yz",
    "zz",
    "ud",
    "deltau",
    "vu",
    "wd",
    "deltaw",
    "vw",
    "refx",
    "refy",
    "refz",
    "refvx",
    "refvy",
    "refvz",
    "refax",
    "refay",
    "refaz",
    "refxz",
    "refyz",
    "refzz",
]


def plotXY(data):
    fig = FigureCreator()
    fig.plot(data.x, data.y, label="COM")
    fig.plot(data.xz, data.yz, label="ZMP")
    fig.plot(data.xd, data.yd, label="Ref")
    fig.setTitleLabels(xlabel="x position", ylabel="y position")
    fig.makeAspectEqual()
    fig.legend()


def plotX(data):
    fig = FigureCreator()
    fig.plot(data.t, data.x, label="COM")
    fig.plot(data.t, data.xd, label="Ref")
    fig.plot(data.t, data.xz, label="ZMP")
    fig.setTitleLabels(xlabel="time", ylabel="x position")
    # fig.setYLimit([0.2, 0.35])
    fig.legend()


def plotY(data):
    fig = FigureCreator()
    fig.plot(data.t, data.y, label="COM")
    fig.plot(data.t, data.yd, label="Ref")
    fig.plot(data.t, data.yz, label="ZMP")
    fig.setTitleLabels(xlabel="time", ylabel="y position")
    # fig.setYLimit([0.2, 0.35])
    fig.legend()


def plotZ(data):
    fig = FigureCreator()
    fig.plot(data.t, data.z, label="COM")
    fig.plot(data.t, data.zd, label="Ref")
    fig.setTitleLabels(xlabel="time", ylabel="z position")
    fig.setYLimit([0.2, 0.35])
    fig.legend()


def usage(prog):
    print(f"{prog} [-hx] datafile")


def main():
    import sys

    showflag = True
    datafile = None

    # parse arguments
    for arg in sys.argv:
        if arg.startswith("-"):
            if arg == "-h":
                usage(sys.argv[0])
                sys.exit(0)
            if arg == "-x":
                showflag = False
        else:
            datafile = arg
    if datafile is None:
        usage(sys.argv[0])
        sys.exit(1)

    # plot
    data = Data(datafile, LABEL_LIST)
    plotXY(data)
    plotX(data)
    plotY(data)
    # plotZ(data)

    if showflag:
        plt.show()


if __name__ == "__main__":
    main()
