#include <pedi2/pd_cz_hrz_u.h>

void pdCZHrzUInit(pdCZHrzU *u, double *kappa, double *zeta)
{
  u->_vd = 0;
  u->_q1 = 0;
  u->_q2 = 0;
  u->_kappa = kappa;
  u->_zeta = zeta;
}

void pdCZHrzUDestroy(pdCZHrzU *u)
{
  u->_vd = 0;
  u->_q1 = 0;
  u->_q2 = 0;
  u->_kappa = NULL;
  u->_zeta = NULL;
}

static double _pdCZHrzUK1(pdCZHrzU *u);
static double _pdCZHrzUK2(pdCZHrzU *u);

double _pdCZHrzUK1(pdCZHrzU *u)
{
  return pdCZHrzUQ1(u) * pdCZHrzUQ2(u);
}

double _pdCZHrzUK2(pdCZHrzU *u)
{
  return ( pdCZHrzUQ1(u) + pdCZHrzUQ2(u) ) / pdCZHrzUZeta(u);
}

double pdCZHrzUCalcSimZMP(pdCZHrzU *u, double du, double vu, double dw, double vw)
{
  double r;

  r = 1.0 + pdCZHrzUKappa(u) * dw;
  return -_pdCZHrzUK1(u)*r*du + _pdCZHrzUK2(u)*(vu-r*pdCZHrzURefVel(u)) + 2*pdCZHrzUKappa(u)*vu*vw/(zSqr(pdCZHrzUZeta(u))*r);
}

double pdCZHrzUCalcRegZMP(pdCZHrzU *u, double du, double vu, double dw, double vw)
{
  double r;

  r = 1.0 + pdCZHrzUKappa(u) * dw;
  return -_pdCZHrzUK1(u)*r*du + _pdCZHrzUK2(u)*vu + 2*pdCZHrzUKappa(u)*vu*vw/(zSqr(pdCZHrzUZeta(u))*r);
}
