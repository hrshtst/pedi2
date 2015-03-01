#include <pedi2/pd_hrz.h>

static void _pdHrzSetPrmRad(void *dst, void *src);
static double _pdHrzK1Rad(void *prm);
static double _pdHrzK2Rad(void *prm);
static double _pdHrzZMPRad(void *prm);

#define _pdc(p) ((pdHrzPrmRad *)p)

void _pdHrzSetPrmRad(void *dst, void *src)
{
  _pdc(dst)->vd    = _pdc(src)->vd;
  _pdc(dst)->q1    = _pdc(src)->q1;
  _pdc(dst)->q2    = _pdc(src)->q2;
  _pdc(dst)->kappa = _pdc(src)->kappa;
  _pdc(dst)->rho   = _pdc(src)->rho;
  _pdc(dst)->kr    = _pdc(src)->kr;
  _pdc(dst)->dist  = _pdc(src)->dist;
}

double _pdHrzK1Rad(void *prm)
{
  return _pdc(prm)->q1 * _pdc(prm)->q2;
}

double _pdHrzK2Rad(void *prm)
{
  return ( _pdc(prm)->q1 + _pdc(prm)->q2 ) / _pdc(prm)->vrt->zeta;
}

double _pdHrzZMPRad(void *prm)
{
  return _pdc(prm)->wz;
}

static pdHrzCom pd_hrz_rad = {
  _pdHrzSetPrmRad,
  _pdHrzK1Rad,
  _pdHrzK2Rad,
  _pdHrzZMPRad,
};

pdHrz *pdHrzSetupRad(pdHrz *h, pdVrt *v)
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
  _pdc(h->prm)->vrt = v;
  h->com = &pd_hrz_rad;
  return h;
}
