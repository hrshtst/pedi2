#include <pedi2/pd_cz_hrz.h>

void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt)
{
  pdCZHrzSetRefPosX( hrz, 0 );
  pdCZHrzSetRefPosY( hrz, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPosX( hrz, 0 );
  pdCZHrzSetPosY( hrz, 0 );
  pdCZHrzSetVelX( hrz, 0 );
  pdCZHrzSetVelY( hrz, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
  pdCZHrzMovInit( &hrz->_hrzm, vrt );
  hrz->refxz = 0;
  hrz->refyz = 0;
  hrz->refddx = 0;
  hrz->refddy = 0;
}

void pdCZHrzDestroy(pdCZHrz *hrz)
{
  pdCZHrzMovDestroy( &hrz->_hrzm );
  pdCZHrzSetRefPosX( hrz, 0 );
  pdCZHrzSetRefPosY( hrz, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPosX( hrz, 0 );
  pdCZHrzSetPosY( hrz, 0 );
  pdCZHrzSetVelX( hrz, 0 );
  pdCZHrzSetVelY( hrz, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
}

void pdCZHrzXformMtoW(pdCZHrz *hrz, double u, double w, double *X, double *Y)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  *X = pdCZHrzPosX(hrz) + c * u - s * w;
  *Y = pdCZHrzPosY(hrz) + s * u + c * w;
}

void pdCZHrzXformWtoM(pdCZHrz *hrz, double X, double Y, double *u, double *w)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  *u =  c * ( X - pdCZHrzPosX(hrz) ) + s * ( Y - pdCZHrzPosY(hrz) );
  *w = -s * ( X - pdCZHrzPosX(hrz) ) + c * ( Y - pdCZHrzPosY(hrz) );
}

void pdCZHrzRotMtoW(pdCZHrz *hrz, double vu, double vw, double *vx, double *vy)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  *vx = c * vu - s * vw;
  *vy = s * vu + c * vw;
}

void pdCZHrzRotWtoM(pdCZHrz *hrz, double vx, double vy, double *vu, double *vw)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  *vu =  c * vx + s * vy;
  *vw = -s * vx + c * vy;
}

void pdCZHrzXformSRWtoM(pdCZHrz *hrz, zVec3DList *sr)
{
  zVec3D *p, *pp;
  zVec3DListCell *cp;
  double u, w;

  if( !( p = zAlloc( zVec3D, zListNum( sr ) ) ) ){
    ZALLOCERROR();
    zFree( p );
    exit( EXIT_FAILURE );
  }
  pp = p;
  zListForEach( sr, cp ){
    pdCZHrzXformWtoM( hrz, zVec3DElem(cp->data,zX), zVec3DElem(cp->data,zY), &u, &w );
    zVec3DCreate( pp++, u, w, zVec3DElem(cp->data,zZ) );
  }
  pdCZHrzMovSetSR( &hrz->_hrzm, p, zListNum(sr) );
  zFree( p );
}

void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, double ud, double wd, double *deltau, double *deltaw)
{
  if( zIsTiny( pdCZHrzKappa(hrz) ) ){
    *deltau = ud;
    *deltaw = wd;
  } else {
    *deltau = asin( pdCZHrzKappa(hrz)*ud ) / pdCZHrzKappa(hrz);
    *deltaw = wd - ( 1.0 - sqrt( 1.0 - zSqr(pdCZHrzKappa(hrz)*ud) ) ) / pdCZHrzKappa(hrz);
  }
}

void pdCZHrzUpdate(pdCZHrz *hrz, double x, double y, double theta, double vx, double vy, double xd, double yd, double thetad, zVec3DList *sr)
{
  double ud, wd, du, vu, dw, vw;

  pdCZHrzSetPosX( hrz, x );
  pdCZHrzSetPosY( hrz, y );
  pdCZHrzSetTheta( hrz, theta );
  pdCZHrzSetVelX( hrz, vx );
  pdCZHrzSetVelY( hrz, vy );
  pdCZHrzSetRefPosX( hrz, xd );
  pdCZHrzSetRefPosY( hrz, yd );
  pdCZHrzSetRefTheta( hrz, thetad );
  if( sr )
    pdCZHrzXformSRWtoM( hrz, sr );
  pdCZHrzXformWtoM( hrz, pdCZHrzRefPosX(hrz), pdCZHrzRefPosY(hrz), &ud, &wd );
  pdCZHrzCalcDiffToRefPos( hrz, ud, wd, &du, &dw );
  pdCZHrzRotWtoM( hrz, pdCZHrzVelX(hrz), pdCZHrzVelY(hrz), &vu, &vw);
  pdCZHrzUpdateM( hrz, du, vu, dw, vw );
  pdCZHrzXformMtoW( hrz, pdCZHrzZMPU(hrz), pdCZHrzZMPW(hrz), &pdCZHrzZMPX(hrz), &pdCZHrzZMPY(hrz) );
  pdCZHrzRotMtoW( hrz, pdCZHrzAccU(hrz), pdCZHrzAccW(hrz), &pdCZHrzAccX(hrz), &pdCZHrzAccY(hrz) );
}
