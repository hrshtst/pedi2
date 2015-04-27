#include <pedi2/pd_cz_hrz_w.h>

void pdCZHrzWInit(pdCZHrzW *w, double *kappa, double *zeta)
{
  w->_vd = 0;
  w->_q1 = 0;
  w->_q2 = 0;
  w->_rho = 0;
  w->_kr = 0;
  w->_dist = 0;
  w->_kappa = kappa;
  w->_zeta = zeta;
}

void pdCZHrzWDestroy(pdCZHrzW *w)
{
  w->_vd = 0;
  w->_q1 = 0;
  w->_q2 = 0;
  w->_rho = 0;
  w->_kr = 0;
  w->_dist = 0;
  w->_kappa = NULL;
  w->_zeta = NULL;
}

static double _pdCZHrzWK1(pdCZHrzW *w);
static double _pdCZHrzWK2(pdCZHrzW *w);
static double _pdCZHrzWAct(pdCZHrzW *w, double dw, double vw);

double _pdCZHrzWK1(pdCZHrzW *w)
{
  return pdCZHrzWQ1(w) * pdCZHrzWQ2(w);
}

double _pdCZHrzWK2(pdCZHrzW *w)
{
  return ( pdCZHrzWQ1(w) + pdCZHrzWQ2(w) ) / pdCZHrzWZeta(w);
}

double _pdCZHrzWAct(pdCZHrzW *w, double dw, double vw)
{
  double d_, d2;
  double act;

  d_ = 0.5 * pdCZHrzWDist(w);
  d2 = zSqr( dw ) + zSqr( vw/pdCZHrzWZeta(w) ) / ( pdCZHrzWQ1(w)*pdCZHrzWQ2(w) );
  act = 1.0 - pdCZHrzWRho(w)*exp( pdCZHrzWKr(w)*( 1.0 - zSqr((pdCZHrzWQ1(w)*pdCZHrzWQ2(w)+1.0)/d_)*d2 ) );
  return act;
}

double pdCZHrzWCalcSimZMP(pdCZHrzW *w, zVec2D delta, zVec2D vel)
{
  double r;

  r = 1.0 + pdCZHrzWKappa(w) * delta[pdW];
  return -_pdCZHrzWK1(w)*delta[pdW] + _pdCZHrzWK2(w)*_pdCZHrzWAct(w,delta[pdW],vel[pdW])*vel[pdW] - (pdCZHrzWKappa(w)/r)*zSqr(vel[pdU]/pdCZHrzWZeta(w));
}

double pdCZHrzWCalcRegZMP(pdCZHrzW *w, zVec2D delta, zVec2D vel)
{
  double r;

  r = 1.0 + pdCZHrzWKappa(w) * delta[pdW];
  return -_pdCZHrzWK1(w)*delta[pdW] + _pdCZHrzWK2(w)*vel[pdW] - (pdCZHrzWKappa(w)/r)*zSqr(vel[pdU]/pdCZHrzWZeta(w));
}
