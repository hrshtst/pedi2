#!/usr/bin/env python
# -*- coding: utf-8 -*-

from datautil import Data
from plotutil import FigureCreator
import numpy as np
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


class SRDataElem(object):

    def __init__(self, n_vert=0, vert_list=[]):
        self.set(n_vert, vert_list)

    def __eq__(self, other):
        return (isinstance(other, self.__class__)
                and self._n_vert == other._n_vert
                # and np.allclose(self._vert, other._vert, atol=1e-03))
                and np.allclose(self._vert, other._vert, atol=1e-02))

    def __ne__(self, other):
        return not self.__eq__(other)

    def set(self, n_vert, vert_list):
        self._n_vert = n_vert
        self._vert = np.array(vert_list, dtype=float)
        if self._n_vert > 0:
            self._vert = self._vert.reshape(self._n_vert, 3)
            self._vert = np.vstack((self._vert, self._vert[0]))

    def get2d(self):
        if self.is_off():
            raise UserWarning('supporting region data is none')
        else:
            return self._vert[:, 0], self._vert[:, 1]

    def is_on(self):
        if self._n_vert > 0:
            return True
        else:
            return False

    def is_off(self):
        return not self.is_on()


class SRDataOne(object):

    def __init__(self, raw_data=[0, 0, 0]):
        self.read(raw_data)

    @staticmethod
    def _read(raw_data, pos):
        n_vert = int(raw_data[pos])
        pos += 1
        sr = SRDataElem(n_vert, raw_data[pos:pos + 3 * n_vert])
        pos += 3 * n_vert
        return pos, sr

    def read(self, raw_data):
        pos = 0
        pos, self.sr = self._read(raw_data, pos)
        pos, self.sr_lf = self._read(raw_data, pos)
        pos, self.sr_rf = self._read(raw_data, pos)

    def is_both_feet_on(self):
        return self.sr_lf.is_on() and self.sr_rf.is_on()

    def is_either_foot_off(self):
        return self.sr_lf.is_off() or self.sr_rf.is_off()

    def is_left_foot_on(self):
        return self.sr_lf.is_on()

    def is_right_foot_on(self):
        return self.sr_rf.is_on()


class SRDataIterator(object):

    def __init__(self, sr_data, tag):
        self._i = 0
        self._sr_data = sr_data
        self._tag = tag
        self._sr = SRDataElem()

    def __iter__(self):
        return self

    def next(self):
        while(True):
            if self._i == self._sr_data.num():
                raise StopIteration()
            sr_data = self._sr_data.get(self._i)
            self._i += 1
            if (self._tag == 'left' and
                    sr_data.is_left_foot_on() and self._sr != sr_data.sr_lf):
                self._sr = sr_data.sr_lf
                return self._sr.get2d()
            if (self._tag == 'right' and
                    sr_data.is_right_foot_on() and self._sr != sr_data.sr_rf):
                self._sr = sr_data.sr_rf
                return self._sr.get2d()


class SRData(object):

    def __init__(self, datapath):
        self._sr_data_list = self.read_file(datapath)

    def read_file(self, filename):
        file_obj = open(filename)
        sr_data = []
        for line in file_obj:
            raw_data = map(float, line.strip().split())
            sr_data.append(SRDataOne(raw_data))
        file_obj.close()
        return sr_data

    def num(self):
        return len(self._sr_data_list)

    def get(self, i):
        return self._sr_data_list[i]

    def iterate_left(self):
        return SRDataIterator(self, 'left')

    def iterate_right(self):
        return SRDataIterator(self, 'right')


def plot_sr(fig, sr_data):
    for x, y in sr_data.iterate_left():
        fig.plot(x, y, 'k-')
    for x, y in sr_data.iterate_right():
        fig.plot(x, y, 'k-')


def plot_traj(fig, data):
    fig.plot(data.x, data.y, 'r-', label='actual COM position')
    fig.plot(data.xz, data.yz, 'g-', label='actual ZMP position')
    # fig.plot(data.xd, data.yd, label='referential COM pos.')
    # fig.setTitleLabels(xlabel='x [m]', ylabel='y [m]')
    fig.makeAspectEqual()
    fig.legend(frameaplha=0.0)
    fig.save('simulation_result.svg')
    fig.save('simulation_result.png')


def usage(prog):
    print "Usage:"
    print "  %s [-hx] data.log sr.log" % prog


def main():
    import sys
    showflag = True
    datalog = None
    srlog = None

    # parse arguments
    for arg in sys.argv[1:]:
        if arg.startswith('-'):
            if arg == '-h':
                usage(sys.argv[0])
                sys.exit(0)
            if arg == '-x':
                showflag = False
        else:
            if datalog is None:
                datalog = arg
            else:
                srlog = arg
    if datalog is None or srlog is None:
        usage(sys.argv[0])
        sys.exit(1)

    # plot
    fig = FigureCreator()
    sr_data = SRData(srlog)
    plot_sr(fig, sr_data)
    data = Data(datalog, LABEL_LIST)
    plot_traj(fig, data)

    if showflag:
        plt.show()

if __name__ == '__main__':
    main()
