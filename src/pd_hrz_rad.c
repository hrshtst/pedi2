#include <pedi2/pd_hrz.h>

#define _pdc(p) ((pdHrzPrmRad *)p)

static pdHrzCom pd_hrz_rad = {
};

pdHrz *pdHrzSetupRad(pdHrz *h)
{
  if( !( h->prm = zAlloc( pdHrzPrmRad, 1 ) ) )
    return NULL;
  _pdc(h->prm)->vd = 0;
  _pdc(h->prm)->q1 = 0;
  _pdc(h->prm)->q2 = 0;
  _pdc(h->prm)->kappa = 0;
  _pdc(h->prm)->rho = 0;
  _pdc(h->prm)->kr = 0;
  _pdc(h->prm)->dist = 0;
  h->com = &pd_hrz_rad;
  return h;
}
