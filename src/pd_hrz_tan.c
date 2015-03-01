#include <pedi2/pd_hrz.h>

static void _pdHrzSetPrmTan(void *dst, void *src);
/* static double _pdHrzK1Tan(void *prm); */
/* static double _pdHrzK2Tan(void *prm); */

#define _pdc(p) ((pdHrzPrmTan *)p)

void _pdHrzSetPrmTan(void *dst, void *src)
{
  _pdc(dst)->vd = _pdc(src)->vd;
  _pdc(dst)->q1 = _pdc(src)->q1;
  _pdc(dst)->q2 = _pdc(src)->q2;
  _pdc(dst)->kappa = _pdc(src)->kappa;
}

/* double _pdHrzK1Tan(void *prm) */
/* { */
/*   return 0.5; */
/* } */

/* double _pdHrzK2Tan(void *prm) */
/* { */
/*   return 1.5; */
/* } */

static pdHrzCom pd_hrz_tan = {
  _pdHrzSetPrmTan,
  /* _pdHrzK1Tan, */
  /* _pdHrzK2Tan, */
};

pdHrz *pdHrzSetupTan(pdHrz *h, pdVrt *v)
{
  if( !( h->prm = zAlloc( pdHrzPrmTan, 1 ) ) )
    return NULL;
  _pdc(h->prm)->vd = 0;
  _pdc(h->prm)->q1 = 0;
  _pdc(h->prm)->q2 = 0;
  _pdc(h->prm)->kappa = 0;
  _pdc(h->prm)->vrt = v;
  h->com = &pd_hrz_tan;
  return h;
}
