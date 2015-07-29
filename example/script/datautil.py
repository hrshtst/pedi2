import numpy as np


class Data(object):

    def __init__(self, datapath, labellist):
        self._datapath = datapath
        self._labellist = labellist
        self._rawdata = np.loadtxt(self._datapath)
        self._data = self.make_data(self._rawdata, self._labellist)

    def __getattribute__(self, name):
        if name == '_data':
            try:
                return object.__getattribute__(self, name)
            except AttributeError:
                return {}
        if name in self._data.keys():
            return self._data[name]
        else:
            return object.__getattribute__(self, name)

    @staticmethod
    def make_data(rawdata, labellist):
        data = {}
        rawdata = np.atleast_2d(rawdata)
        if len(labellist) != rawdata.shape[1]:
            msg = 'num of data entries does not match with that of labels'
            raise UserWarning(msg)
        for i, l in enumerate(labellist):
            if l in data.keys():
                msg = 'label [%s] already exists' % l
                raise UserWarning(msg)
            data[l] = rawdata[:, i]
        return data

    def getDataPath(self):
        return self._datapath

    def getDataNum(self):
        return self._rawdata.shape[0]

    def getDataLabelNum(self):
        return self._rawdata.shape[1]
