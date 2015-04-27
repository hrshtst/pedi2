#include <zm/zm_ode.h>
#include <pedi2/pd_cz_hrz.h>

/* state vector
 * p = [ x  y ]
 */

zVec ddp(double t, zVec x, zVec dx, void *dummy, zVec ddx)
{
  pdCZHrz *hrz;

  hrz = (pdCZHrz *)dummy;
  zVecSetElem( ddx, 0, pdCZHrzAccX( hrz ) );
  zVecSetElem( ddx, 1, pdCZHrzAccY( hrz ) );
  return ddx;
}

void output(double t, zVec2D pd, zVec2D p, zVec2D v, zVec2D pz, double theta)
{
  printf( "%f %f %f %f %f %f %f %f %f %f\n",
/* 0   */ t,
/* 1-2 */ pd[zX], pd[zY],
/* 3-4 */ p[zX], p[zY],
/* 5-6 */ v[zX], v[zY],
/* 7-8 */ pz[zX], pz[zY],
/* 9   */ theta);
}

void auto_update_ref(pdCZHrz *hrz, zVec2D refxy, zVec2D refxyd, double *theta)
{
  zVec2D refuw;
  double refdw;
  double delta_theta;
  zVec2D refuwd;


  pdCZHrzXformXYtoUW( hrz, refxy, refuw );
  delta_theta = atan2( pdCZHrzKappa(hrz)*refuw[pdU], 1.0+pdCZHrzKappa(hrz)*(pdCZHrzDeltaW(hrz)-refuw[pdW]) );
  refdw = ( pdCZHrzDeltaW(hrz) - refuw[pdW] ) / cos(delta_theta);
  if( !zIsTiny(pdCZHrzKappa(hrz)) )
    refdw += ( 1.0 - cos(delta_theta) ) / ( pdCZHrzKappa(hrz)*cos(delta_theta) );
  zVec2DCreate( refuwd, refuw[pdU] - refdw * sin(delta_theta),
                        refuw[pdW] + refdw * cos(delta_theta) );
  pdCZHrzXformUWtoXY( hrz, refuwd, refxyd );
  *theta += delta_theta;
}


#define DT 0.01
#define T 20.0
int main(void)
{
  pdCZVrt vrt;
  pdCZHrz hrz;
  zVec2D xy, vxy, zmp, xyd;
  zODE2 ode;
  zVec p, dp;
  double t;
  double theta, thetad;

  /* init of pdCZHrz */
  pdCZVrtInit( &vrt );
  pdCZVrtSetPrm( &vrt, 0.26, 1.0, 0.5 );
  pdCZHrzInit( &hrz, &vrt );
  pdCZHrzSetRefVelU( &hrz, 0.25 );
  pdCZHrzSetQ1U( &hrz, 1.0 );
  pdCZHrzSetQ2U( &hrz, 0.0 );
  pdCZHrzSetRefVelW( &hrz, 0.0 );
  pdCZHrzSetQ1W( &hrz, 1.0 );
  pdCZHrzSetQ2W( &hrz, 1.5 );
  pdCZHrzSetRho( &hrz, 1.0 );
  pdCZHrzSetKr( &hrz, 1.0 );
  pdCZHrzSetDist( &hrz, 0.1 );
  pdCZHrzSetKappa( &hrz, 0.0 );
  /* init states */
  theta = thetad = -zPI_2;
  zVec2DCreate( xy, 0, 0 );
  zVec2DCreate( vxy, 0, 0 );
  zVec2DClear( zmp );
  zVec2DClear( xyd );
  /* init of ODE solver */
  zODE2Assign( &ode, Regular, NULL, NULL, NULL, NULL );
  zODE2AssignRegular( &ode, RK4 );
  zODE2Init( &ode, 2, 0, ddp );
  /* init of state vector */
  p  = zVecCreateList( 2, xy[zX], xy[zY] );
  dp = zVecCreateList( 2, vxy[zX], vxy[zY] );
  /* loop */
  for( t=0; t<T; t+=DT ){
    /* modify commands */
    if( t < 0.4*T )
      pdCZHrzSetKappa( &hrz, 1.0 );
    else if( t < 0.6*T )
      pdCZHrzSetKappa( &hrz, 0.0 );
    else
      pdCZHrzSetKappa( &hrz, -1.0 );
    /* update */
    pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
    pdCZHrzUpdate( &hrz, xy, theta, vxy, xyd, thetad, NULL );
    zODE2Update( &ode, t, p, dp, DT, &hrz );
    zVec2DCreate( xy, zVecElem(p,0), zVecElem(p,1) );
    zVec2DCreate( vxy, zVecElem(dp,0), zVecElem(dp,1) );
    zVec2DCreate( zmp, pdCZHrzZMPX(&hrz), pdCZHrzZMPY(&hrz) );
    /* output */
    output( t, xyd, xy, vxy, zmp, theta );
    /* auto reference update */
    if( !zIsTiny( pdCZHrzRefVelU(&hrz) ) )
      auto_update_ref( &hrz, xy, xyd, &theta );
  }
  /* destroy */
  zVecFree( p );
  zVecFree( dp );
  zODE2Destroy( &ode );
  pdCZHrzDestroy( &hrz );
  pdCZVrtDestroy( &vrt );
  return 0;
}
