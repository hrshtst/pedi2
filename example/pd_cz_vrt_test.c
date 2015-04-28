#include <zm/zm_vec.h>
#include <zm/zm_ode.h>
#include <pedi2/pd_cz_vrt.h>

/* state vector
 * p = [ z ]
 */

zVec ddp(double t, zVec x, zVec dx, void *dummy, zVec ddx)
{
  pdCZVrt *vrt;

  vrt = (pdCZVrt *)dummy;
  zVecSetElem( ddx, 0, pdCZVrtAcc( vrt ) );
  return ddx;
}

void output(double t, double z, double vz, double az)
{
  printf( "%f %f %f %f\n", t, z, vz, az );
}

#define DT 0.01
#define T 5.0

int main(void)
{
  pdCZVrt vrt;
  zODE2 ode;
  zVec x, dx;
  double t;

  /* init of pdCZVrt */
  pdCZVrtInit( &vrt );
  pdCZVrtSetPrm( &vrt, 0.3, 1.0, 0.5 );
  /* init of ODE solver */
  zODE2Assign( &ode, Regular, NULL, NULL, NULL, NULL );
  zODE2AssignRegular( &ode, RK4 );
  zODE2Init( &ode, 1, 0, ddp );
  /* init of state vector */
  x = zVecCreateList( 1, 0.35 );
  dx = zVecAlloc( 1 );
  /* loop */
  output( 0, zVecElem(x,0), zVecElem(dx,0), pdCZVrtAcc(&vrt) );
  for( t=0; t<T; t+=DT ){
    pdCZVrtUpdateAcc( &vrt, zVecElem(x,0), zVecElem(dx,0) );
    zODE2Update( &ode, t, x, dx, DT, &vrt );
    output( t, zVecElem(x,0), zVecElem(dx,0), pdCZVrtAcc(&vrt) );
  }
  /* destroy */
  zVecFree( x );
  zVecFree( dx );
  zODE2Destroy( &ode );
  pdCZVrtDestroy( &vrt );
  return 0;
}
