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
  hrz->_sr_vert = NULL;
}

void pdCZHrzMovDestroy(pdCZHrzMov *hrz)
{
  pdCZHrzUDestroy( &hrz->_u );
  pdCZHrzWDestroy( &hrz->_w );
  hrz->_vrt = NULL;
  pdCZHrzMovSetKappa( hrz, 0 );
  pdCZHrzMovSetSR( hrz, NULL );
  zFree( pdCZHrzMovSRVert( hrz ) );
}

void pdCZHrzMovCalcZMP(pdCZHrzMov *hrz, double du, double vu, double dw, double vw, double *uz, double *wz)
{
  zVec3D p, cp;

  zVec3DCreate( &p,
                pdCZHrzMovCalcSimZMPU( hrz, du, vu, dw, vw ),
                pdCZHrzMovCalcSimZMPW( hrz, du, vu, dw, vw ),
                pdCZVrtCalcZMP( hrz->_vrt ) );
  if( hrz->_sr ){
    zCH2DClosest( hrz->_sr, &p, &cp );
    *uz = zVec3DElem( &cp, zX );
    *wz = zVec3DElem( &cp, zY );
  } else {
    *uz = zVec3DElem( &p, zX );
    *wz = zVec3DElem( &p, zY );
  }
}

void pdCZHrzMovCalcAcc(pdCZHrzMov *hrz, double uz, double wz, double *ddu, double *ddw)
{
  *ddu = -zSqr( pdCZHrzMovZeta( hrz ) ) * uz;
  *ddw = -zSqr( pdCZHrzMovZeta( hrz ) ) * wz;
}

void pdCZHrzMovUpdate(pdCZHrzMov *hrz, double du, double vu, double dw, double vw)
{
  pdCZHrzMovCalcZMP( hrz, du, vu, dw, vw, &pdCZHrzMovZMPU(hrz), &pdCZHrzMovZMPW(hrz) );
  pdCZHrzMovCalcAcc( hrz,  pdCZHrzMovZMPU(hrz),  pdCZHrzMovZMPW(hrz),
                          &pdCZHrzMovAccU(hrz), &pdCZHrzMovAccW(hrz) );
}
