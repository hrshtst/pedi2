#include <unistd.h>
#include <zm/zm_ode.h>
#include <zeo/zeo_vec3d.h>
#include <zx11/zxutil.h>
#include <pedi2/pd_ctrl.h>
#include <liw/liw_time.h>
#include "util/scene.h"

zVec3D com;
zVec3D zmp;

void polar(double *r, double *theta)
{
  *r = sqrt( zSqr(zVec3DElem(&com,zX)) + zSqr(zVec3DElem(&com,zY)) );
  *theta = atan2( zVec3DElem(&com,zY), zVec3DElem(&com,zX) );
}

void measure(pdCtrl *ctrl, double vx, double vy, double *du, double *vu, double *dw, double *vw)
{
  double r, rd;
  double theta, thetad;
  double s, c;

  polar( &r, &theta );
  rd = 1.0 / pdCtrlKappa( ctrl );
  thetad = theta;
  zSinCos( theta, &s, &c );
  *du = rd * ( thetad - theta );
  *vu = -s*vx + c*vy;
  *dw = rd - r;
  *vw = -c*vx - s*vy;
}

void world_zmp(double uz, double wz)
{
  double r, theta;
  double s, c;

  polar( &r, &theta );
  zSinCos( theta, &s, &c );
  zVec3DElem(&zmp,zX) = zVec3DElem(&com,zX) - uz*s - wz*c;
  zVec3DElem(&zmp,zY) = zVec3DElem(&com,zY) + uz*c - wz*s;
}

zVec dp(double t, zVec p, void *dummy, zVec v)
{
  double du, vu, dw, vw;
  pdCtrl *ctrl;

  ctrl = (pdCtrl *)dummy;
  zVec3DElem(&com,zX) = zVecElem(p,0);
  zVec3DElem(&com,zY) = zVecElem(p,2);
  measure( ctrl, zVecElem(p,1), zVecElem(p,3), &du, &vu, &dw, &vw );
  pdCtrlUpdate( ctrl, du, vu, dw, vw );
  world_zmp( pdCtrlZMPTan(ctrl), pdCtrlZMPRad(ctrl) );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCtrlZeta(ctrl)) * ( zVecElem(p,0) - zVec3DElem(&zmp,zX) );
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCtrlZeta(ctrl)) * ( zVecElem(p,2) - zVec3DElem(&zmp,zY) );
  return v;
}

void init_ctrl(pdCtrl *ctrl)
{
  pdCtrlInit( ctrl );
  /*                qu1, qu2, qw1, qw2, kappa, rho, kr */
  pdCtrlSetPrm( ctrl, 1,   0,   1, 1.5,     2,   1,  1 );
  /*                     zd */
  pdCtrlSetRefVrt( ctrl, 0.26 );
  /*                      vud, vwd, dist */
  pdCtrlSetRefHrz( ctrl, 0.25, 0.0, 0.1 );
  zVec3DCreate( &com, 0.6, 0, 0.26 );
  zVec3DCreate( &zmp, 0.6, 0, 0 );
}

void resize(zxWindow *win, Scene *sc)
{
  short w,h;

  zxWindowClear( win );
  w = zxWindowWidth( win );
  h = zxWindowHeight( win );
  resizeScene( sc, 0, 0, w, h );
}

#define DT     0.01
void mainloop(zxWindow *win, Scene *sc, zODE *ode, pdCtrl *ctrl)
{
  int count = 0;
  int t;
  zVec p;

  p = zVecCreateList( 4, zVec3DElem(&com,zX), 0, zVec3DElem(&com,zY), 0 );
  while( 1 ){
    switch( zxGetEvent() ){
    case KeyPress:
      switch( zxKeySymbol() ){
      case XK_q: return;
      }
      break;
    case ConfigureNotify:
      zxWindowUpdateRegion( win );
      resize( win, sc );
      break;
    default: ;
    }
    t = count*DT;
    resize( win, sc );
    zODEUpdate( ode, t, p, DT, ctrl );
    lookAt( sc, 0.0, -5.5, 3, 0.0, 0.0, 0.0 );
    drawScene( sc, &com, &zmp );
    usleep( sec2usec(DT) );
    count++;
  }
}



#define WIDTH  480
#define HEIGHT 480
int main(int argc, char *argv[])
{
  zxWindow win;
  zODE ode;
  Scene sc;
  pdCtrl ctrl;

  glrkInitGLX();
  zxWindowCreate( &win, 0, 0, WIDTH, HEIGHT );
  zxWindowSetBG( &win, (char *)"lightgray" );
  zxWindowClear( &win );
  zxKeyEnable( &win );
  zxWindowSetTitle( &win, (char *)"ctrl view test" );
  zxWindowOpen( &win );

  zODEAssign( &ode, RKF45, NULL, NULL );
  zODEInit( &ode, 4, 0, dp );
  init_ctrl( &ctrl );
  initScene( &sc, &win );

  mainloop( &win, &sc, &ode, &ctrl );

  exitScene( &sc );
  pdCtrlDestroy( &ctrl );
  return 0;
}
