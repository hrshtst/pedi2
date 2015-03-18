#include <zm/zm_ode.h>
#include <zeo/zeo_vec3d.h>
#include <pedi2/pd_ctrl.h>

/* state vector
 * p = [ x  dx/dt  y  dy/dt ]^T
 */
zVec3D com;
zVec3D vel;
zVec3D ref;
zVec3D zmp;
double theta;

void measure(double *du, double *vu, double *dw, double *vw)
{
  double dx, dy;
  double vx, vy;
  double s, c;

  dx = zVec3DElem(&ref,zX) - zVec3DElem(&com,zX);
  dy = zVec3DElem(&ref,zY) - zVec3DElem(&com,zY);
  vx = zVec3DElem(&vel,zX);
  vy = zVec3DElem(&vel,zY);
  zSinCos( theta, &s, &c );
  *du = 0;
  *vu = -vx*s + vy*c;
  *dw = dx*c + dy*s;
  *vw = -vx*c - vy*s;
}

void world_zmp(double uz, double wz)
{
  double s, c;

  zSinCos( theta, &s, &c );
  zVec3DElem(&zmp,zX) = zVec3DElem(&com,zX) - uz*s - wz*c;
  zVec3DElem(&zmp,zY) = zVec3DElem(&com,zY) + uz*c - wz*s;
}

zVec dp(double t, zVec p, void *dummy, zVec v)
{
  double du, vu, dw, vw;
  pdCtrl *ctrl;

  ctrl = (pdCtrl *)dummy;
  measure( &du, &vu, &dw, &vw );
  /* printf("du: %g, vu: %g, dw: %g, vw: %g\n", du, vu, dw, vw); */
  pdCtrlUpdate( ctrl, du, vu, dw, vw );
  /* printf("uz: %g, wz: %g\n", pdCtrlZMPTan(ctrl), pdCtrlZMPRad(ctrl)); */
  world_zmp( pdCtrlZMPTan(ctrl), pdCtrlZMPRad(ctrl) );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCtrlZeta(ctrl)) * ( zVecElem(p,0) - zVec3DElem(&zmp,zX) );
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCtrlZeta(ctrl)) * ( zVecElem(p,2) - zVec3DElem(&zmp,zY) );
  return v;
}

void state_update(zVec p)
{
  zVec3DElem(&com,zX) = zVecElem(p,0);
  zVec3DElem(&com,zY) = zVecElem(p,2);
  zVec3DElem(&vel,zX) = zVecElem(p,1);
  zVec3DElem(&vel,zY) = zVecElem(p,3);
}

void ref_update(pdCtrl *ctrl, double xs, double ys)
{
  double x, y;
  double xd, yd;
  double kappa;
  double dw, dws;
  double s, c;
  double cos_d;
  double kx, ky, kw;

  x  = zVec3DElem(&com,zX);
  y  = zVec3DElem(&com,zY);
  xd = zVec3DElem(&ref,zX);
  yd = zVec3DElem(&ref,zY);
  kappa = pdCtrlKappa(ctrl);
  zSinCos( theta, &s, &c );
  dw = -( xd - x ) * c - ( yd - y ) * s;

  kx = kappa * (xs - x);
  ky = kappa * (ys - y);
  cos_d = ( kx*c + ky*s + 1 ) / sqrt( zSqr(kx+c) + zSqr(ky+s) );
  dws = ( dw + (xs-x)*c + (ys-y)*s ) / cos_d;
  if( !zIsTiny(kappa) )
    dws += ( 1 - cos_d ) / ( kappa * cos_d );
  kw = 1.0 + kappa * dws;
  zVec3DElem(&ref,zX) = xd + (xs-xd)/kw - dws*c/kw;
  zVec3DElem(&ref,zY) = yd + (ys-yd)/kw - dws*s/kw;
  theta = atan2( kappa*(ys-yd)/kw+s/kw, kappa*(xs-xd)/kw+c/kw );
}

void print_data(double t)
{
  printf( "%g %g %g %g %g %g %g %g %g %g\n", t,
          zVec3DElem(&com,zX), zVec3DElem(&com,zY),
          zVec3DElem(&vel,zX), zVec3DElem(&vel,zY),
          zVec3DElem(&zmp,zX), zVec3DElem(&zmp,zY),
          zVec3DElem(&ref,zX), zVec3DElem(&ref,zY),
          theta );
}

#define DT   0.01
#define STEP 1500
/* #define STEP 5 */
int main(void)
{
  zODE ode;
  zVec p;
  pdCtrl ctrl;
  double t;
  register int i;

  zODEAssign( &ode, RKF45, NULL, NULL );
  zODEInit( &ode, 4, 0, dp );
  pdCtrlInit( &ctrl );
  pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 2.0, 1.0, 1.0 );
  pdCtrlSetRefVrt( &ctrl, 0.26 );
  pdCtrlSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  p = zVecCreateList( 4, 0.55, 0.0, 0.0, 0.0 );
  zVec3DCreate( &com, zVecElem(p,0), zVecElem(p,2), 0.26 );
  zVec3DCreate( &vel, zVecElem(p,1), zVecElem(p,3), 0 );
  zVec3DCreate( &zmp, zVecElem(p,0), zVecElem(p,2), 0 );
  zVec3DCreate( &ref, 0.5, 0, 0 );
  theta = 0;
  for( i=0; i<=STEP; i++ ){
    t = DT * i;
    if( i == 500 )
      pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
    if( i == 1000 )
      pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, -2.0, 1.0, 1.0 );
    state_update( p );
    zODEUpdate( &ode, t, p, DT, &ctrl );
    print_data( t );
    ref_update( &ctrl, zVecElem(p,0), zVecElem(p,2) );
    /* getchar(); */
  }
  zVecFree( p );
  zODEDestroy( &ode );

  return 0;
}
