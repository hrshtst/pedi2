from math import exp
import numpy as np


class Param(object):

    def __init__(self):
        self.vd = 0
        self.q1 = 0
        self.q2 = 0
        self.rho = 0
        self.kr = 0
        self.dist = 0
        self.kappa = 0
        self.zeta = 0

    def make_str(self, label, value):
        return label + ' = ' + str(value) + '\n'

    def __str__(self):
        res = ''
        res += self.make_str('vd', self.vd)
        res += self.make_str('q1', self.q1)
        res += self.make_str('q2', self.q2)
        res += self.make_str('rho', self.rho)
        res += self.make_str('kr', self.kr)
        res += self.make_str('dist', self.dist)
        res += self.make_str('kappa', self.kappa)
        res += self.make_str('zeta', self.zeta)


def gamma(p, dw, vw):
    d_ = 0.5 * p.dist
    d2 = dw ** 2 + (vw / p.zeta) ** 2 / (p.q1 * p.q2)
    act = 1.0 - p.rho * \
        np.exp(p.kr * (1. - ((p.q1 * p.q2 + 1.) / d_) ** 2 * d2))
    # print 'd_ = ', d_
    # print 'd2 = ', d2
    # print 'act = ', act
    # print 'arg exp = ', p.kr * (1. - ((p.q1 * p.q2 + 1.) / d_) ** 2 * d2)
    # print 'sqr = ', (1.0 - (p.q1 * p.q2 + 1.0) / d_) ** 2
    return act


def simwz(p, du, vu, dw, vw):
    r = 1. + p.kappa * dw
    k1 = p.q1 * p.q2
    k2 = (p.q1 + p.q2) / p.zeta
    return k1 * dw + k2 * gamma(p, dw, vw) * vw - \
                               (p.kappa / r) * (vu / p.zeta) ** 2


def regwz(p, du, vu, dw, vw):
    r = 1. + p.kappa * dw
    k1 = p.q1 * p.q2
    k2 = (p.q1 + p.q2) / p.zeta
    return k1 * dw + k2 * vw - (p.kappa / r) * (vu / p.zeta) ** 2

# parameter setting
p = Param()
p.vd = 0
p.q1 = 1
p.q2 = 1
p.rho = 1
p.kr = 1
p.dist = 1
p.zeta = 1
p.kappa = 1

print 'simwz'
print simwz(p, 0, 0, 0, 0)
print simwz(p, 0.1, 0.1, 0.1, 0.1)
print simwz(p, 0.2, -0.2, 0.2, -0.2)
print 'regwz'
print regwz(p, 0, 0, 0, 0)
print regwz(p, 0.1, 0.1, 0.1, 0.1)
print regwz(p, 0.2, -0.2, 0.2, -0.2)
