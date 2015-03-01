#include <zm/zm_ode.h>
#include <pedi2/pd_ctrl.h>

/* state vector
 * p = [ x  dx/dt  y  dy/dt ]^T
 */

void polar(zVec p, double *r, double *theta)
{
  *r = sqrt( zSqr(zVecElem(p,0)) + zSqr(zVecElem(p,2)) );
  *theta = atan2( zVecElem(p,2), zVecElem(p,0) );
}

void measure(pdCtrl *ctrl, zVec p, double *du, double *vu, double *dw, double *vw)
{
  double r, rd;
  double theta, thetad;
  double s, c;

  polar( p, &r, &theta );
  rd = 1.0 / pdCtrlKappa(ctrl);
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
  pdCtrl *ctrl;

  ctrl = (pdCtrl *)dummy;
  measure( ctrl, p, &du, &vu, &dw, &vw );
  pdCtrlUpdate( ctrl, du, vu, dw, vw );
  world_zmp( p, pdCtrlZMPTan(ctrl), pdCtrlZMPRad(ctrl), &xz, &yz );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCtrlZeta(ctrl)) * ( zVecElem(p,0) - xz );
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCtrlZeta(ctrl)) * ( zVecElem(p,2) - yz );
  return v;
}

#define DT   0.01
#define STEP 1000
int main(void)
{
  zODE ode;
  zVec p;
  pdCtrl ctrl;
  double t, xz, yz;
  register int i;

  zODEAssign( &ode, RKF45, NULL, NULL );
  zODEInit( &ode, 4, 0, dp );
  pdCtrlInit( &ctrl );
  pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 2.0, 1.0, 1.0 );
  pdCtrlSetRefVrt( &ctrl, 0.26 );
  pdCtrlSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  p = zVecCreateList( 4, 0.6, 0.0, 0.0, 0.0 );
  for( i=0; i<=STEP; i++ ){
    t = DT * i;
    zODEUpdate( &ode, t, p, DT, &ctrl );
    world_zmp( p, pdCtrlZMPTan(&ctrl), pdCtrlZMPRad(&ctrl), &xz, &yz );
    printf( "%g %g %g ", t, xz, yz );
    zVecDataWrite( p );
    /* getchar(); */
  }
  zVecFree( p );
  zODEDestroy( &ode );

  return 0;
}
