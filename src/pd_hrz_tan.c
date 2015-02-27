#include <pedi2/pd_hrz.h>

#define _pdc(p) ((pdHrzPrmTan *)p)

static pdHrzCom pd_hrz_tan = {
};

pdHrz *pdHrzSetupTan(pdHrz *h)
{
  if( !( h->prm = zAlloc( pdHrzPrmTan, 1 ) ) )
    return NULL;
  _pdc(h->prm)->vd = 0;
  _pdc(h->prm)->q1 = 0;
  _pdc(h->prm)->q2 = 0;
  _pdc(h->prm)->dist = 0;
  h->com = &pd_hrz_tan;
  return h;
}
