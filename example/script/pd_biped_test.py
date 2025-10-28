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
    "lfpx",
    "lfpy",
    "lfpz",
    "lfpdx",
    "lfpdy",
    "lfpdz",
    "lfrefpx",
    "lfrefpy",
    "lfrefpz",
    "lfax",
    "lfay",
    "lfaz",
    "lfadx",
    "lfady",
    "lfadz",
    "lfrefax",
    "lfrefay",
    "lfrefaz",
    "lfphi",
    "lfh",
    "lfpzre",
    "lfpzim",
    "lfphase",
    "lfzrefz",
    "rfpx",
    "rfpy",
    "rfpz",
    "rfpdx",
    "rfpdy",
    "rfpdz",
    "rfrefpx",
    "rfrefpy",
    "rfrefpz",
    "rfax",
    "rfay",
    "rfaz",
    "rfadx",
    "rfady",
    "rfadz",
    "rfrefax",
    "rfrefay",
    "rfrefaz",
    "rfphi",
    "rfh",
    "rfpzre",
    "rfpzim",
    "rfphase",
    "rfzrefz",
]


def plotXY(data):
    fig = FigureCreator()
    fig.plot(data.x, data.y, label="COM")
    fig.plot(data.xz, data.yz, label="ZMP")
    fig.plot(data.xd, data.yd, label="Ref")
    fig.setTitleLabels(xlabel="x position [m]", ylabel="y position [m]")
    fig.makeAspectEqual()
    fig.legend()


def plotX(data):
    fig = FigureCreator()
    fig.plot(data.t, data.x, label="COM position")
    fig.plot(data.t, data.xz, label="ZMP position")
    fig.plot(data.t, data.xd, label="Ref COM position")
    fig.plot(data.t, data.lfpx, label="Left foot pos.")
    fig.plot(data.t, data.rfpx, label="Right foot pos.")
    fig.setTitleLabels(xlabel="time [s]", ylabel="x position [m]")
    fig.legend()


def plotY(data):
    fig = FigureCreator()
    fig.plot(data.t, data.y, label="COM position")
    fig.plot(data.t, data.yz, label="ZMP position")
    fig.plot(data.t, data.yd, label="Ref COM position")
    fig.plot(data.t, data.lfpy, label="Left foot pos.")
    fig.plot(data.t, data.rfpy, label="Right foot pos.")
    fig.setTitleLabels(xlabel="time [s]", ylabel="y position [m]")
    fig.legend()


def plotZ(data):
    fig = FigureCreator()
    fig.plot(data.t, data.z, label="COM")
    fig.plot(data.t, data.zd, label="Ref")
    fig.setTitleLabels(xlabel="time [s]", ylabel="z position [m]")
    fig.setYLimit([0.2, 0.35])
    fig.legend()


def plotFootX(data):
    fig = FigureCreator()
    fig.plot(data.t, data.lfpx, label="lf pos")
    fig.plot(data.t, data.rfpx, label="rf pos")
    fig.plot(data.t, data.lfpdx, label="desired lf pos")
    fig.plot(data.t, data.rfpdx, label="desired rf pos")
    fig.setTitleLabels(xlabel="time [s]", ylabel="position x [m]")
    fig.legend()


def plotFootY(data):
    fig = FigureCreator()
    fig.plot(data.t, data.lfpy, label="lf pos")
    fig.plot(data.t, data.rfpy, label="rf pos")
    fig.plot(data.t, data.lfpdy, label="desired lf pos")
    fig.plot(data.t, data.rfpdy, label="desired rf pos")
    fig.setTitleLabels(xlabel="time [s]", ylabel="position y [m]")
    fig.legend()


def plotFootZ(data):
    fig = FigureCreator()
    fig.plot(data.t, data.lfpz, label="lf pos")
    fig.plot(data.t, data.rfpz, label="rf pos")
    fig.plot(data.t, data.lfpdz, label="desired lf pos")
    fig.plot(data.t, data.rfpdz, label="desired rf pos")
    fig.setTitleLabels(xlabel="time [s]", ylabel="position z [m]")
    fig.legend()


def plotFootXZ(data):
    fig = FigureCreator()
    fig.plot(data.lfpx, data.lfpz, label="lf pos")
    fig.plot(data.rfpx, data.rfpz, label="rf pos")
    fig.plot(data.lfpdx, data.lfpdz, label="desired lf pos")
    fig.plot(data.rfpdx, data.rfpdz, label="desired rf pos")
    fig.setTitleLabels(xlabel="position x [m]", ylabel="position z [m]")
    fig.setLimits(xlim=[0.1, 0.3])
    fig.legend()
    fig.save("foot-xz.svg")


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
    # plotFootX(data)
    # plotFootY(data)
    # plotFootZ(data)
    # plotFootXZ(data)

    if showflag:
        plt.show()


if __name__ == "__main__":
    main()
