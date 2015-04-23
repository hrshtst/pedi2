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
