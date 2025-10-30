from __future__ import annotations

import matplotlib.pyplot as plt

from datautil import Data
from plotutil import FigureCreator

LABEL_LIST = ["input", "output"]


def plot(data):
    fig = FigureCreator()
    fig.plot(data.input, label="ipnut")
    fig.plot(data.output, label="output")
    fig.setTitleLabels(xlabel="step", ylabel="value")
    fig.setLimits(xlim=[1000, 3000])
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
    plot(data)

    if showflag:
        plt.show()


if __name__ == "__main__":
    main()
