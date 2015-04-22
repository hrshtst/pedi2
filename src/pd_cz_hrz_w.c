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
