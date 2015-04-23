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

zVec3D *pdCZHrzXformSRWtoM(pdCZHrz *hrz, zVec3DList *sr_w, zVec3DList *sr_m, zVec3D *sr_m_vert)
{
  zVec3DListCell *cp;
  zVec3D *traversep;
  double u, w;

  if( sr_m_vert == NULL || zListNum( sr_w ) != zListNum( sr_m ) ){
    zFree( sr_m_vert );
    /* printf("reallocated!\n"); */
    if( !( sr_m_vert = zAlloc( zVec3D, zListNum( sr_w ) ) ) ){
      ZALLOCERROR();
      zFree( sr_m_vert );
      exit( EXIT_FAILURE );
    }
  }
  traversep = sr_m_vert;
  zListForEach( sr_w, cp ){
    pdCZHrzXformWtoM( hrz, zVec3DElem(cp->data,zX), zVec3DElem(cp->data,zY), &u, &w );
    zVec3DCreate( traversep++, u, w, zVec3DElem(cp->data,zZ) );
  }
  zCH2D( sr_m, sr_m_vert, zListNum(sr_w) );
  return sr_m_vert;
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
