#include <pedi2/pd_hrz.h>

void pdHrzInit(pdHrz *hrz)
{
  hrz->q1 = 0;
  hrz->q2 = 0;
  hrz->kappa = 0;
  hrz->rho = 0;
  hrz->kr = 0;
  hrz->vrt = NULL;
}

void pdHrzSetup(pdHrz *hrz, pdVrt *vrt)
{
  hrz->vrt = vrt;
}

double pdHrzK1(pdHrz *hrz)
{
  return hrz->q1 * hrz->q2;
}

double pdHrzK2(pdHrz *hrz)
{
  return ( hrz->q1 + hrz->q2 ) / hrz->vrt->zeta;
}
