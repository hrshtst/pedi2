#include <pedi2/pd_cz_hrz_mov.h>

void pdCZHrzMovInit(pdCZHrzMov *hrz, pdCZVrt *vrt)
{
  pdCZHrzUInit( &hrz->_u, &pdCZHrzMovKappa(hrz), &pdCZVrtZeta(vrt) );
  pdCZHrzWInit( &hrz->_w, &pdCZHrzMovKappa(hrz), &pdCZVrtZeta(vrt) );
  hrz->_vrt = vrt;
  pdCZHrzMovSetKappa( hrz, 0 );
  pdCZHrzMovSetSR( hrz, NULL );
  hrz->refuz = 0;
  hrz->refwz = 0;
  hrz->refddu = 0;
  hrz->refddw = 0;
}

void pdCZHrzMovDestroy(pdCZHrzMov *hrz)
{
  pdCZHrzUDestroy( &hrz->_u );
  pdCZHrzWDestroy( &hrz->_w );
  hrz->_vrt = NULL;
  pdCZHrzMovSetKappa( hrz, 0 );
  pdCZHrzMovSetSR( hrz, NULL );
}

void pdCZHrzMovCalcZMP(pdCZHrzMov *hrz, double du, double vu, double dw, double vw)
{
  hrz->refuz = pdCZHrzMovCalcSimZMPU( hrz, du, vu, dw, vw );
  hrz->refwz = pdCZHrzMovCalcSimZMPW( hrz, du, vu, dw, vw );
}

void pdCZHrzMovCalcAcc(pdCZHrzMov *hrz, double du, double vu, double dw, double vw)
{
  hrz->refddu = -zSqr( pdCZHrzMovZeta( hrz ) ) * pdCZHrzMovZMPU( hrz );
  hrz->refddw = -zSqr( pdCZHrzMovZeta( hrz ) ) * pdCZHrzMovZMPW( hrz );
}
