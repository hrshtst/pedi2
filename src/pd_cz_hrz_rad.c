#include <pedi2/pd_cz_hrz.h>

static void _pdCZHrzSetPrmRad(void *dst, void *src);
static double _pdCZHrzK1Rad(void *prm);
static double _pdCZHrzK2Rad(void *prm);
static double _pdCZHrzAct(void *prm, double dw, double vw);
static double _pdCZHrzZMPRad(void *prm);
static double _pdCZHrzAccRad(void *prm);
static void _pdCZHrzUpdateRad(void *prm, double du, double vu, double dw, double vw);

#define _pdc(p) ((pdCZHrzPrmRad *)p)

void _pdCZHrzSetPrmRad(void *dst, void *src)
{
  _pdc(dst)->xd    = _pdc(src)->xd;
  _pdc(dst)->vd    = _pdc(src)->vd;
  _pdc(dst)->q1    = _pdc(src)->q1;
  _pdc(dst)->q2    = _pdc(src)->q2;
  _pdc(dst)->kappa = _pdc(src)->kappa;
  _pdc(dst)->rho   = _pdc(src)->rho;
  _pdc(dst)->kr    = _pdc(src)->kr;
  _pdc(dst)->dist  = _pdc(src)->dist;
}

double _pdCZHrzK1Rad(void *prm)
{
  return _pdc(prm)->q1 * _pdc(prm)->q2;
}

double _pdCZHrzK2Rad(void *prm)
{
  return ( _pdc(prm)->q1 + _pdc(prm)->q2 ) / _pdc(prm)->vrt->zeta;
}

double _pdCZHrzAct(void *prm, double dw, double vw)
{
  double d_, d2;
  double ret;

  d_ = 0.5 * _pdc(prm)->dist;
  d2 = zSqr( dw ) + zSqr( vw/_pdc(prm)->vrt->zeta ) / ( _pdc(prm)->q1 * _pdc(prm)->q2 );
  ret = 1.0 - _pdc(prm)->rho * exp( _pdc(prm)->kr * ( 1.0 - zSqr((_pdc(prm)->q1*_pdc(prm)->q2+1.0)/d_)*d2 ) );
  return ret;
}

double _pdCZHrzZMPRad(void *prm)
{
  return _pdc(prm)->zmp;
}

double _pdCZHrzAccRad(void *prm)
{
  return _pdc(prm)->acc;
}

void _pdCZHrzUpdateRad(void *prm, double du, double vu, double dw, double vw)
{
  double r;

  r = 1.0 - _pdc(prm)->kappa * dw;
  _pdc(prm)->zmp = _pdCZHrzK1Rad(prm)*dw + _pdCZHrzK2Rad(prm)*_pdCZHrzAct(prm,dw,vw)*vw - (_pdc(prm)->kappa/r)*zSqr(vu/_pdc(prm)->vrt->zeta);
  _pdc(prm)->acc = zSqr(_pdc(prm)->vrt->zeta) * _pdCZHrzZMPRad(prm) + (_pdc(prm)->kappa/r)*zSqr(vu);
}

static pdCZHrzCom pd_hrz_rad = {
  _pdCZHrzSetPrmRad,
  _pdCZHrzK1Rad,
  _pdCZHrzK2Rad,
  _pdCZHrzZMPRad,
  _pdCZHrzAccRad,
  _pdCZHrzUpdateRad,
};

pdCZHrz *pdCZHrzSetupRad(pdCZHrz *h, pdCZVrt *v)
{
  if( !( h->prm = zAlloc( pdCZHrzPrmRad, 1 ) ) )
    return NULL;
  _pdc(h->prm)->xd = 0;
  _pdc(h->prm)->vd = 0;
  _pdc(h->prm)->q1 = 0;
  _pdc(h->prm)->q2 = 0;
  _pdc(h->prm)->kappa = 0;
  _pdc(h->prm)->rho = 0;
  _pdc(h->prm)->kr = 0;
  _pdc(h->prm)->dist = 0;
  _pdc(h->prm)->zmp = 0;
  _pdc(h->prm)->acc = 0;
  _pdc(h->prm)->vrt = v;
  h->com = &pd_hrz_rad;
  return h;
}
