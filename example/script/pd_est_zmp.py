from __future__ import annotations

import matplotlib.pyplot as plt

from datautil import Data
from plotutil import FigureCreator

LABEL_LIST = [
    "fx",
    "fy",
    "fz",
    "xz",
    "yz",
    "zz",
]

DT = 0.002


def plotFX(data):
    fig = FigureCreator()
    t = data.makeTimeSeq(DT)
    fig.plot(t, data.fx, label="Force x")
    fig.setTitleLabels(xlabel="time [s]", ylabel="force [N]")
    fig.legend()


def plotFY(data):
    fig = FigureCreator()
    t = data.makeTimeSeq(DT)
    fig.plot(t, data.fy, label="Force y")
    fig.setTitleLabels(xlabel="time [s]", ylabel="force [N]")
    fig.legend()


def plotFZ(data):
    fig = FigureCreator()
    t = data.makeTimeSeq(DT)
    fig.plot(t, data.fz, label="Force z")
    fig.setTitleLabels(xlabel="time [s]", ylabel="force [N]")
    fig.legend()


def plotXZ(data):
    fig = FigureCreator()
    t = data.makeTimeSeq(DT)
    fig.plot(t, data.xz, label="ZMP x")
    fig.setTitleLabels(xlabel="time [s]", ylabel="position [m]")
    fig.legend()


def plotYZ(data):
    fig = FigureCreator()
    t = data.makeTimeSeq(DT)
    fig.plot(t, data.yz, label="ZMP y")
    fig.setTitleLabels(xlabel="time [s]", ylabel="position [m]")
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
    # plotFX(data)
    # plotFY(data)
    # plotFZ(data)
    # plotXZ(data)
    plotYZ(data)

    if showflag:
        plt.show()


if __name__ == "__main__":
    main()
