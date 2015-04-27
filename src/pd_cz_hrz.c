#include <pedi2/pd_cz_hrz.h>

void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt)
{
  pdCZHrzSetRefPos( hrz, 0, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPos( hrz, 0, 0 );
  pdCZHrzSetVel( hrz, 0, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
  pdCZHrzUWInit( pdCZHrzUWPtr(hrz), vrt );
  pdCZHrzSetRefPosUW( hrz, 0, 0 );
  pdCZHrzSetDelta( hrz, 0, 0 );
  pdCZHrzSetVelUW( hrz, 0, 0 );
  pdCZHrzZMPX( hrz ) = 0;
  pdCZHrzZMPY( hrz ) = 0;
  pdCZHrzAccX( hrz ) = 0;
  pdCZHrzAccY( hrz ) = 0;
}

void pdCZHrzDestroy(pdCZHrz *hrz)
{
  pdCZHrzUWDestroy( pdCZHrzUWPtr(hrz) );
  pdCZHrzSetRefPos( hrz, 0, 0 );
  pdCZHrzSetRefTheta( hrz, 0 );
  pdCZHrzSetPos( hrz, 0, 0 );
  pdCZHrzSetVel( hrz, 0, 0 );
  pdCZHrzSetTheta( hrz, 0 );
  pdCZHrzSetSR( hrz, NULL );
  pdCZHrzSetRefPosUW( hrz, 0, 0 );
  pdCZHrzSetDelta( hrz, 0, 0 );
  pdCZHrzSetVelUW( hrz, 0, 0 );
}

double *pdCZHrzRotUWtoXY(pdCZHrz *hrz, zVec2D vuw, zVec2D vxy)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  return zVec2DCreate( vxy, -s*vuw[pdU]-c*vuw[pdW], c*vuw[pdU]-s*vuw[pdW]);
}

double *pdCZHrzRotXYtoUW(pdCZHrz *hrz, zVec2D vxy, zVec2D vuw)
{
  double s, c;

  zSinCos( pdCZHrzTheta(hrz), &s, &c );
  return zVec2DCreate( vuw, -s*vxy[zX]+c*vxy[zY], -c*vxy[zX]-s*vxy[zY] );
}

double *pdCZHrzXformUWtoXY(pdCZHrz *hrz, zVec2D uw, zVec2D xy)
{
  zVec2D p;

  pdCZHrzRotUWtoXY( hrz, uw, p );
  return zVec2DAdd( p, pdCZHrzPos(hrz), xy );
}

double *pdCZHrzXformXYtoUW(pdCZHrz *hrz, zVec2D xy, zVec2D uw)
{
  zVec2D p;

  zVec2DSub( xy, pdCZHrzPos(hrz), p );
  return pdCZHrzRotXYtoUW( hrz, p, uw );
}

void pdCZHrzXformSRXYtoUW(pdCZHrz *hrz, zVec3DList *sr)
{
  zVec3D *p, *pp;
  zVec3DListCell *cp;
  zVec2D xy, uw;

  if( !( p = zAlloc( zVec3D, zListNum( sr ) ) ) ){
    ZALLOCERROR();
    zFree( p );
    exit( EXIT_FAILURE );
  }
  pp = p;
  zListForEach( sr, cp ){
    zVec2DCreate( xy, zVec3DElem(cp->data,zX), zVec3DElem(cp->data,zY) );
    pdCZHrzXformXYtoUW( hrz, xy, uw );
    zVec3DCreate( pp++, uw[pdU], uw[pdW], zVec3DElem(cp->data,zZ) );
  }
  pdCZHrzUWSetSR( pdCZHrzUWPtr(hrz), p, zListNum(sr) );
  zFree( p );
}

void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, zVec2D uwd, zVec2D delta)
{
  if( zIsTiny( pdCZHrzKappa(hrz) ) ){
    zVec2DCopy( uwd, delta );
  } else {
    zVec2DCreate( delta, asin( pdCZHrzKappa(hrz)*uwd[pdU] ) / pdCZHrzKappa(hrz),
                  uwd[pdW] - ( 1.0 - sqrt( 1.0 - zSqr(pdCZHrzKappa(hrz)*uwd[pdU]) ) ) / pdCZHrzKappa(hrz) );
  }
}

void pdCZHrzUpdate(pdCZHrz *hrz, zVec2D xy, double theta, zVec2D vxy, zVec2D xyd, double thetad, zVec3DList *sr)
{
  pdCZHrzSetPosVec( hrz, xy );
  pdCZHrzSetTheta( hrz, theta );
  pdCZHrzSetVelVec( hrz, vxy );
  pdCZHrzSetRefPosVec( hrz, xyd );
  pdCZHrzSetRefTheta( hrz, thetad );
  if( sr )
    pdCZHrzXformSRXYtoUW( hrz, sr );
  pdCZHrzXformXYtoUW( hrz, pdCZHrzRefPos(hrz), pdCZHrzRefPosUW(hrz) );
  pdCZHrzCalcDiffToRefPos( hrz, pdCZHrzRefPosUW(hrz), pdCZHrzDelta(hrz) );
  pdCZHrzRotXYtoUW( hrz, pdCZHrzVel(hrz), pdCZHrzVelUW(hrz) );
  pdCZHrzUpdateUW( hrz, pdCZHrzDelta(hrz), pdCZHrzVelUW(hrz) );
  pdCZHrzXformUWtoXY( hrz, pdCZHrzZMPUW(hrz), pdCZHrzZMP(hrz) );
  pdCZHrzRotUWtoXY( hrz, pdCZHrzAccUW(hrz), pdCZHrzAcc(hrz) );
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
