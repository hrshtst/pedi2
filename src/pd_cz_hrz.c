#include <pedi2/pd_cz_hrz.h>

void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt)
{
  pdCZHrzSetRefPos( hrz, 0, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPos( hrz, 0, 0 );
  pdCZHrzSetVel( hrz, 0, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
  pdCZHrzMovInit( &pdCZHrzM(hrz), vrt );
  pdCZHrzZMPX( hrz ) = 0;
  pdCZHrzZMPY( hrz ) = 0;
  pdCZHrzAccX( hrz ) = 0;
  pdCZHrzAccY( hrz ) = 0;
}

void pdCZHrzDestroy(pdCZHrz *hrz)
{
  pdCZHrzMovDestroy( &pdCZHrzM(hrz) );
  pdCZHrzSetRefPos( hrz, 0, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPos( hrz, 0, 0 );
  pdCZHrzSetVel( hrz, 0, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
}

double *pdCZHrzRotMtoW(pdCZHrz *hrz, zVec2D vm, zVec2D vw)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  return zVec2DCreate( vw, -s*vm[pdU]-c*vm[pdW], c*vm[pdU]-s*vm[pdW]);
}

double *pdCZHrzRotWtoM(pdCZHrz *hrz, zVec2D vw, zVec2D vm)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  return zVec2DCreate( vm, -s*vw[zX]+c*vw[zY], -c*vw[zX]-s*vw[zY] );
}

double *pdCZHrzXformMtoW(pdCZHrz *hrz, zVec2D pm, zVec2D pw)
{
  zVec2D p;

  pdCZHrzRotMtoW( hrz, pm, p );
  return zVec2DAdd( p, pdCZHrzPos(hrz), pw );
}

double *pdCZHrzXformWtoM(pdCZHrz *hrz, zVec2D pw, zVec2D pm)
{
  zVec2D p;

  zVec2DSub( pw, pdCZHrzPos(hrz), p );
  return pdCZHrzRotWtoM( hrz, p, pm );
}

void pdCZHrzXformSRWtoM(pdCZHrz *hrz, zVec3DList *sr)
{
  zVec3D *p, *pp;
  zVec3DListCell *cp;
  zVec2D pw, pm;

  if( !( p = zAlloc( zVec3D, zListNum( sr ) ) ) ){
    ZALLOCERROR();
    zFree( p );
    exit( EXIT_FAILURE );
  }
  pp = p;
  zListForEach( sr, cp ){
    zVec2DCreate( pw, zVec3DElem(cp->data,zX), zVec3DElem(cp->data,zY) );
    pdCZHrzXformWtoM( hrz, pw, pm );
    zVec3DCreate( pp++, pm[pdU], pm[pdW], zVec3DElem(cp->data,zZ) );
  }
  pdCZHrzMovSetSR( &pdCZHrzM(hrz), p, zListNum(sr) );
  zFree( p );
}

void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, zVec2D pdm, zVec2D deltam)
{
  if( zIsTiny( pdCZHrzKappa(hrz) ) ){
    zVec2DCopy( pdm, deltam );
  } else {
    zVec2DCreate( deltam, asin( pdCZHrzKappa(hrz)*pdm[pdU] ) / pdCZHrzKappa(hrz),
                  pdm[pdW] - ( 1.0 - sqrt( 1.0 - zSqr(pdCZHrzKappa(hrz)*pdm[pdU]) ) ) / pdCZHrzKappa(hrz) );
  }
}

void pdCZHrzUpdate(pdCZHrz *hrz, zVec2D p, double theta, zVec2D v, zVec2D pd, double thetad, zVec3DList *sr)
{
  pdCZHrzSetPosVec( hrz, p );
  pdCZHrzSetTheta( hrz, theta );
  pdCZHrzSetVelVec( hrz, v );
  pdCZHrzSetRefPosVec( hrz, pd );
  pdCZHrzSetRefTheta( hrz, thetad );
  if( sr )
    pdCZHrzXformSRWtoM( hrz, sr );
  pdCZHrzXformWtoM( hrz, pdCZHrzRefPos(hrz), pdCZHrzRefPosM(hrz) );
  pdCZHrzCalcDiffToRefPos( hrz, pdCZHrzRefPosM(hrz), pdCZHrzDeltaM(hrz) );
  pdCZHrzRotWtoM( hrz, pdCZHrzVel(hrz), pdCZHrzVelM(hrz) );
  pdCZHrzUpdateM( hrz, pdCZHrzDeltaM(hrz), pdCZHrzVelM(hrz) );
  pdCZHrzXformMtoW( hrz, pdCZHrzZMPM(hrz), pdCZHrzZMP(hrz) );
  pdCZHrzRotMtoW( hrz, pdCZHrzAccM(hrz), pdCZHrzAcc(hrz) );
}

void pdCZHrzFWrite(FILE *fp, pdCZHrz *hrz)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "xd:%f, yd:%f, thetad:%f\n",
           pdCZHrzRefPosX(hrz), pdCZHrzRefPosY(hrz), pdCZHrzRefTheta(hrz) );
  fprintf( fp, "x: %f, y :%f, theta :%f\n",
           pdCZHrzPosX(hrz), pdCZHrzPosY(hrz), pdCZHrzTheta(hrz) );
  fprintf( fp, "vx:%f, vy:%f\n", pdCZHrzVelX(hrz), pdCZHrzVelY(hrz) );
  fprintf( fp, "ud:%f, deltau:%f, vu:%f\n",
           pdCZHrzRefPosU(hrz), pdCZHrzDeltaU(hrz), pdCZHrzVelU(hrz) );
  fprintf( fp, "wd:%f, deltaw:%f, vw:%f\n",
           pdCZHrzRefPosW(hrz), pdCZHrzDeltaW(hrz), pdCZHrzVelW(hrz) );
  fprintf( fp, "refxz:%f, refyz:%f\n", pdCZHrzZMPX(hrz), pdCZHrzZMPY(hrz) );
  fprintf( fp, "refddx:%f, refddy:%f\n", pdCZHrzAccX(hrz), pdCZHrzAccY(hrz) );
}
