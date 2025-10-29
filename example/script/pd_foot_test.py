from __future__ import annotations

import matplotlib.pyplot as plt

from datautil import Data
from plotutil import FigureCreator

LABEL_LIST = [
    "t",
    "dt",
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
    "lfsign",
    "lfkappa",
    "lfdist",
    "lfphi",
    "lfzmpu",
    "lfzmpw",
    "lfrefu",
    "lfrefw",
    "lfzsign",
    "lfh",
    "lfrho",
    "lfzdist",
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
    "rfsign",
    "rfkappa",
    "rfdist",
    "rfphi",
    "rfzmpu",
    "rfzmpw",
    "rfrefu",
    "rfrefw",
    "rfzsign",
    "rfh",
    "rfrho",
    "rfzdist",
    "rfpzre",
    "rfpzim",
    "rfphase",
    "rfzrefz",
]


def plotFootX(data):
    fig = FigureCreator()
    fig.plot(data.t, data.lfpx, label="lf pos")
    fig.plot(data.t, data.rfpx, label="rf pos")
    fig.plot(data.t, data.lfpdx, label="desired lf pos")
    fig.plot(data.t, data.rfpdx, label="desired rf pos")
    fig.setTitleLabels(xlabel="time [s]", ylabel="x position [m]")
    fig.legend()


def plotFootY(data):
    fig = FigureCreator()
    fig.plot(data.t, data.lfpy, label="lf pos")
    fig.plot(data.t, data.rfpy, label="rf pos")
    fig.plot(data.t, data.lfpdy, label="desired lf pos")
    fig.plot(data.t, data.rfpdy, label="desired rf pos")
    fig.setTitleLabels(xlabel="time [s]", ylabel="y position [m]")
    fig.legend()


def plotFootZ(data):
    fig = FigureCreator()
    fig.plot(data.t, data.lfpz, label="lf pos")
    fig.plot(data.t, data.rfpz, label="rf pos")
    fig.plot(data.t, data.lfpdz, label="desired lf pos")
    fig.plot(data.t, data.rfpdz, label="desired rf pos")
    fig.setTitleLabels(xlabel="time [s]", ylabel="z position [m]")
    fig.legend()


def plotFootXY(data):
    fig = FigureCreator()
    fig.plot(data.lfpx, data.lfpy, label="lf pos")
    fig.plot(data.rfpx, data.rfpy, label="rf pos")
    fig.plot(data.lfpdx, data.lfpdy, label="desired lf pos")
    fig.plot(data.rfpdx, data.rfpdy, label="desired rf pos")
    fig.setTitleLabels(xlabel="x position [m]", ylabel="y position [m]")
    fig.makeAspectEqual()
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
    plotFootX(data)
    plotFootY(data)
    plotFootZ(data)
    plotFootXY(data)

    if showflag:
        plt.show()


if __name__ == "__main__":
    main()
