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

void output(double t, zVec p, zVec dp, double xz, double yz, double xd, double yd, double theta)
{
  printf( "%f %f %f %f %f %f %f %f %f %f\n",
/* 0   */ t,
/* 1-2 */ zVecElem(p, 0), zVecElem(p, 1),
/* 3-4 */ zVecElem(dp, 0), zVecElem(dp, 1),
/* 5-6 */ xz, yz,
/* 7-8 */ xd, yd,
/* 9   */ theta);
}

void auto_update_ref(pdCZHrz *hrz, double refx, double refy, double *xd, double *yd, double *theta)
{
  double refu, refw, refdw;
  double delta_theta;
  double ud, wd;

  pdCZHrzXformWtoM( hrz, refx, refy, &refu, &refw );
  delta_theta = atan2( pdCZHrzKappa(hrz)*refu, 1.0+pdCZHrzKappa(hrz)*(pdCZHrzDeltaW(hrz)-refw) );
  refdw = ( pdCZHrzDeltaW(hrz) - refw ) / cos(delta_theta);
  if( !zIsTiny(pdCZHrzKappa(hrz)) )
    refdw += ( 1.0 - cos(delta_theta) ) / ( pdCZHrzKappa(hrz)*cos(delta_theta) );
  ud = refu - refdw * sin(delta_theta);
  wd = refw + refdw * cos(delta_theta);
  pdCZHrzXformMtoW( hrz, ud, wd, xd, yd );
  *theta += delta_theta;
}


#define DT 0.01
#define T 20.0
int main(void)
{
  pdCZVrt vrt;
  pdCZHrz hrz;
  zODE2 ode;
  zVec p, dp;
  double t;
  double theta, thetad;
  double xd, yd;

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
  theta = thetad = -zPI_2;
  xd = yd = 0;
  /* init of ODE solver */
  zODE2Assign( &ode, Regular, NULL, NULL, NULL, NULL );
  zODE2AssignRegular( &ode, RK4 );
  zODE2Init( &ode, 2, 0, ddp );
  /* init of state vector */
  p = zVecCreateList( 2, 0.0, 0.0 );
  dp = zVecCreateList( 2, 0.0, 0.0 );
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
    pdCZHrzUpdate( &hrz, zVecElem(p,0), zVecElem(p,1), theta, zVecElem(dp,0), zVecElem(dp,1), xd, yd, thetad, NULL );
    zODE2Update( &ode, t, p, dp, DT, &hrz );
    /* output */
    output( t, p, dp, pdCZHrzZMPX(&hrz), pdCZHrzZMPY(&hrz), xd, yd, theta );
    /* auto reference update */
    if( !zIsTiny( pdCZHrzRefVelU(&hrz) ) )
      auto_update_ref( &hrz, zVecElem(p,0), zVecElem(p,1), &xd, &yd, &theta );
  }
  /* destroy */
  zVecFree( p );
  zVecFree( dp );
  zODE2Destroy( &ode );
  pdCZHrzDestroy( &hrz );
  pdCZVrtDestroy( &vrt );
  return 0;
}
