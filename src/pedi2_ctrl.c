#include <pedi2/pedi2_ctrl.h>

void pedi2VertUpdate(pedi2Vert *vert)
{
  vert->zeta = sqrt( RK_G / vert->z );
}

void pedi2CtrlInit(pedi2Ctrl *c)
{
  c->qu1 = 0.0;
  c->qu1 = 0.0;
  c->qu2 = 0.0;
  c->qw1 = 0.0;
  c->qw2 = 0.0;
  c->kappa = 0.0;
  c->rho = 0.0;
  c->kr = 0.0;
  c->vd = 0.0;
  c->w = 0.0;
}

static double _pedi2CtrlAct(pedi2Ctrl *c, double dw, double vw)
{
  double d_, d2;

  d_ = 0.5 * c->w;
  d2 = zSqr( dw ) + zSqr( vw/c->vert->zeta ) / ( c->qw1*c->qw2 );
  return 1.0 - c->rho * exp( c->kr * ( 1.0 - zSqr((c->qw1*c->qw2+1.0)/d_)*d2 ) );
}

void pedi2CtrlZMP(pedi2Ctrl *c, double du, double vu, double dw, double vw)
{
  double r = 1.0 - c->kappa * dw;
  c->uz = (c->qu1+c->qu2) * (vu-r*c->vd) / c->vert->zeta - c->qu1*c->qu2*r*du + 2.0*c->kappa*vu*vw/(zSqr(c->vert->zeta)*r);
  /* c->uz = (c->qu1+c->qu2) * (vu-r*c->vd) / c->vert->zeta - c->qu1*c->qu2*r*du - 2.0*c->kappa*vu*vw/(zSqr(c->vert->zeta)*r); */
  c->wz = (c->qw1+c->qw2) * _pedi2CtrlAct(c,dw,vw) * vw / c->vert->zeta + c->qw1*c->qw2*dw - (c->kappa/r)*zSqr(vu/c->vert->zeta);
}
