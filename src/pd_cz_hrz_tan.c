#include <pedi2/pd_cz_hrz.h>

static void _pdCZHrzSetPrmTan(void *dst, void *src);
static double _pdCZHrzK1Tan(void *prm);
static double _pdCZHrzK2Tan(void *prm);
static void _pdCZHrzUpdateTan(void *prm, double du, double vu, double dw, double vw);
static double _pdCZHrzZMPTan(void *prm);

#define _pdc(p) ((pdCZHrzPrmTan *)p)

void _pdCZHrzSetPrmTan(void *dst, void *src)
{
  _pdc(dst)->vd = _pdc(src)->vd;
  _pdc(dst)->q1 = _pdc(src)->q1;
  _pdc(dst)->q2 = _pdc(src)->q2;
  _pdc(dst)->kappa = _pdc(src)->kappa;
}

double _pdCZHrzK1Tan(void *prm)
{
  return _pdc(prm)->q1 * _pdc(prm)->q2;
}

double _pdCZHrzK2Tan(void *prm)
{
  return ( _pdc(prm)->q1 + _pdc(prm)->q2 ) / _pdc(prm)->vrt->zeta;
}

void _pdCZHrzUpdateTan(void *prm, double du, double vu, double dw, double vw)
{
  double r;

  r = 1.0 - _pdc(prm)->kappa * dw;
  _pdc(prm)->uz = -_pdCZHrzK1Tan(prm)*r*du + _pdCZHrzK2Tan(prm)*(vu-r*_pdc(prm)->vd) + 2*_pdc(prm)->kappa*vu*vw/(zSqr(_pdc(prm)->vrt->zeta)*r);
}

double _pdCZHrzZMPTan(void *prm)
{
  return _pdc(prm)->uz;
}

static pdCZHrzCom pd_hrz_tan = {
  _pdCZHrzSetPrmTan,
  _pdCZHrzK1Tan,
  _pdCZHrzK2Tan,
  _pdCZHrzUpdateTan,
  _pdCZHrzZMPTan,
};

pdCZHrz *pdCZHrzSetupTan(pdCZHrz *h, pdCZVrt *v)
{
  if( !( h->prm = zAlloc( pdCZHrzPrmTan, 1 ) ) )
    return NULL;
  _pdc(h->prm)->vd = 0;
  _pdc(h->prm)->q1 = 0;
  _pdc(h->prm)->q2 = 0;
  _pdc(h->prm)->kappa = 0;
  _pdc(h->prm)->vrt = v;
  h->com = &pd_hrz_tan;
  return h;
}
