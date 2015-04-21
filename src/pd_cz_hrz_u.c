#include <pedi2/pd_cz_hrz_u.h>

void pdCZHrzUInit(pdCZHrzU *u, double *kappa, double *zeta)
{
  u->_vd = 0;
  u->_q1 = 0;
  u->_q2 = 0;
  u->_kappa = kappa;
  u->_zeta = zeta;
  u->refzmp = 0;
  u->refacc = 0;
}

void pdCZHrzUDestroy(pdCZHrzU *u)
{
  u->_vd = 0;
  u->_q1 = 0;
  u->_q2 = 0;
  u->_kappa = NULL;
  u->_zeta = NULL;
  u->refzmp = 0;
  u->refacc = 0;
}
