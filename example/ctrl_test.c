#include <zm/zm_ode.h>
#include <pedi2/pedi2_ctrl.h>

/* state vector
 * p = [ x  dx/dt  y  dy/dt ]^T
 */

void polar(zVec p, double *r, double *theta)
{
  *r = sqrt( zSqr(zVecElem(p,0)) + zSqr(zVecElem(p,2)) );
  *theta = atan2( zVecElem(p,2), zVecElem(p,0) );
}

void measure(pedi2Ctrl *ctrl, zVec p, double *du, double *vu, double *dw, double *vw)
{
  double r, rd;
  double theta, thetad;
  double s, c;

  polar( p, &r, &theta );
  rd = 1.0 / ctrl->kappa;
  thetad = theta;
  zSinCos( theta, &s, &c );
  *du = rd * ( thetad - theta );
  *vu = -s*zVecElem(p,1) + c*zVecElem(p,3);
  *dw = rd - r;
  *vw = -c*zVecElem(p,1) - s*zVecElem(p,3);
}

void world_zmp(zVec p, double uz, double wz, double *xz, double *yz)
{
  double r, theta;
  double s, c;

  polar( p, &r, &theta );
  zSinCos( theta, &s, &c );
  *xz = zVecElem(p,0) - uz*s - wz*c;
  *yz = zVecElem(p,2) + uz*c - wz*s;
}

zVec dp(double t, zVec p, void *dummy, zVec v)
{
  double du, vu, dw, vw;
  double xz, yz;
  pedi2Ctrl *ctrl;

  ctrl = (pedi2Ctrl *)dummy;
  measure( ctrl, p, &du, &vu, &dw, &vw );
  pedi2CtrlZMP( ctrl, du, vu, dw, vw );
  world_zmp( p, ctrl->uz, ctrl->wz, &xz, &yz );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(ctrl->vert->zeta) * ( zVecElem(p,0) - xz );
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(ctrl->vert->zeta) * ( zVecElem(p,2) - yz );
  return v;
}

#define DT   0.01
#define STEP 1000
int main(void)
{
  zODE ode;
  zVec p;
  pedi2Ctrl ctrl;
  pedi2Vert vert;
  double t, xz, yz;
  register int i;

  zODEAssign( &ode, RKF45, NULL, NULL );
  zODEInit( &ode, 4, 0, dp );
  pedi2CtrlInit( &ctrl );
  ctrl.vert = &vert;
  ctrl.vert->z = 0.26;
  pedi2VertUpdate( ctrl.vert );
  ctrl.qu1 = 1.0;
  ctrl.qu2 = 0.0;
  ctrl.qw1 = 1.0;
  ctrl.qw2 = 1.5;
  ctrl.kappa = 2.0;
  ctrl.rho = 1.0;
  ctrl.kr = 1.0;
  /* ctrl.vd = 0.25; */
  ctrl.vd = 0.5;
  ctrl.w = 0.1;
  p = zVecCreateList( 4, 0.6, 0.0, 0.0, 0.0 );
  for( i=0; i<=STEP; i++ ){
    t = DT * i;
    zODEUpdate( &ode, t, p, DT, &ctrl );
    world_zmp( p, ctrl.uz, ctrl.wz, &xz, &yz );
    printf( "%g %g %g ", t, xz, yz );
    zVecDataWrite( p );
    /* getchar(); */
  }
  zVecFree( p );
  zODEDestroy( &ode );

  return 0;
}
