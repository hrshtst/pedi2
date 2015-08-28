from matplotlib import rcParams
rcParams['ps.usedistiller'] = 'xpdf'
import matplotlib.pyplot as plt


class FigureCreator(object):

    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.plot = self.ax.plot
        self.vline = self.ax.axvline
        self.hline = self.ax.axhline

    def resize(self, figsize=(8, 6)):
        self.fig.set_size_inches(figsize[0], figsize[1], forward=True)

    def setTitle(self, title):
        self.ax.set_title(title)

    def setXLabel(self, xlabel):
        self.ax.set_xlabel(xlabel)

    def setYLabel(self, ylabel):
        self.ax.set_ylabel(ylabel)

    def setTitleLabels(self, title=None, xlabel=None, ylabel=None):
        if title is not None:
            self.setTitle(title)
        if xlabel is not None:
            self.setXLabel(xlabel)
        if ylabel is not None:
            self.setYLabel(ylabel)

    def setXLimit(self, xlim):
        self.ax.set_xlim(xlim)

    def setYLimit(self, ylim):
        self.ax.set_ylim(ylim)

    def setLimits(self, xlim=None, ylim=None):
        if xlim is not None:
            self.setXLimit(xlim)
        if ylim is not None:
            self.setYLimit(ylim)

    def makeAspectEqual(self):
        self.ax.set_aspect('equal', 'datalim')

    def legend(self, loc='best'):
        self.ax.legend(loc=loc)

    def save(self, filename):
        self.fig.savefig(filename,
                         bbox_inches='tight', pad_inches=0.0)
