#include <zm/zm_ode.h>
#include <zx11/zxwidget.h>
#include <zx11/zximage_dib.h>
#include <pedi2/pd_ctrl.h>

#include "util/dm_scene.h"
#include "util/dm_console.h"
#include "util/dm_foot.h"

typedef struct{
  double qu1, qu2;
  double qw1, qw2;
  double kappa, rho, kr;
  double zd;
  double vud, vwd, dist;
  double lfh, rfh;
} dmCommand;

typedef struct{
  double x[2], y[2];
  double nx[2], ny[2];
  pdCtrl c;
  dmFoot lf, rf;
  double xz, yz;
  double xzmin, xzmax;
  double yzmin, yzmax;
  double xd, yd;
  double theta;
  double adx, ady;
  dmCommand com;
} dmSystem;

typedef struct{
  double t;
  zVec p;
  zODE ode;
} dmODESolver;

void dmSystemUpdateRobot(dmSystem *sys)
{
  dmRobotSolveIK();
  dmSupportRegion();
  dmSupportRegionBorder( &sys->xzmin, &sys->xzmax, &sys->yzmin, &sys->yzmax );
}

void dmSystemObserve(dmSystem *sys, double *du, double *vu, double *dw, double *vw)
{
  double s, c;

  zSinCos( sys->theta, &s, &c );
  *du = 0;
  *vu = -sys->x[1]*s + sys->y[1]*c;
  *dw = ( sys->xd - sys->x[0] )*c +( sys->yd - sys->y[0] )*s;
  *vw = -sys->x[1]*c - sys->y[1]*s;
}

void dmSystemCoodTransBodyToWorld(dmSystem *sys, double u, double w, double *x, double *y)
{
  double s, c;

  zSinCos( sys->theta, &s, &c );
  *x = sys->x[0] - u*s - w*c;
  *y = sys->y[0] + u*c - w*s;
}

zVec dp(double t, zVec p, void *dummy, zVec v)
{
  double du, vu, dw, vw;
  dmSystem *sys;

  sys = (dmSystem *)dummy;
  dmSystemObserve( sys, &du, &vu, &dw, &vw );
  pdCtrlUpdate( &sys->c, du, vu, dw, vw );
  dmSystemCoodTransBodyToWorld( sys, pdCtrlZMPTan(&sys->c), pdCtrlZMPRad(&sys->c), &sys->xz, &sys->yz );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCtrlZeta(&sys->c)) * ( zVecElem(p,0) - sys->xz ) + sys->adx;
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCtrlZeta(&sys->c)) * ( zVecElem(p,2) - sys->yz ) + sys->ady;
  return v;
}

#define UPDATE_SKEW 1
void dmSystemUpdateCtrl(dmSystem *sys, dmODESolver *solver)
{
  solver->t += DT;
  sys->x[0] = zVecElem(solver->p,0);
  sys->x[1] = zVecElem(solver->p,1);
  sys->y[0] = zVecElem(solver->p,2);
  sys->y[1] = zVecElem(solver->p,3);
  zODEUpdate( &solver->ode, solver->t, solver->p, DT, sys );
  sys->nx[0] = zVecElem(solver->p,0);
  sys->nx[1] = zVecElem(solver->p,1);
  sys->ny[0] = zVecElem(solver->p,2);
  sys->ny[1] = zVecElem(solver->p,3);
}

void dmSystemUpdateFoot(dmSystem *sys)
{
  double d;
  double s, c;

  d = 0.5 * pdCtrlPrmRad(&sys->c)->dist;
  zSinCos( sys->theta, &s, &c );
  dmRobotFootPos( &sys->lf.p, &sys->rf.p );
  dmRobotFootAtt( &sys->lf.a, &sys->rf.a );

  /* update IK constraints for feet */
  /* zVec3DCopy( &sys->lf.ps, &dm_d_lf ); */
  /* zVec3DCopy( &sys->rf.ps, &dm_d_rf ); */
  /* zVec3DCopy( &sys->lf.as, &dm_d_att_lf ); */
  /* zVec3DCopy( &sys->rf.as, &dm_d_att_rf ); */
  zVec3DCreate( &dm_d_lf, sys->xd-d*c, sys->yd-d*s, 0);
  zVec3DCreate( &dm_d_rf, sys->xd+d*c, sys->yd+d*s, 0);
  zVec3DCreate( &dm_d_att_lf, sys->theta+zPI_2, 0, 0 );
  zVec3DCreate( &dm_d_att_rf, sys->theta+zPI_2, 0, 0 );
}

void _dmSystemUpdateRefPosTheta(dmSystem *sys)
{
  double kappa;
  double dw, ndw;
  double s, c;
  double cos_d;
  double kx, ky, kw;

  kappa = pdCtrlKappa(&sys->c);
  zSinCos( sys->theta, &s, &c );
  dw = -( sys->xd - sys->x[0] )*c - ( sys->yd - sys->y[0] )*s;

  kx = kappa * ( sys->nx[0] - sys->x[0] );
  ky = kappa * ( sys->ny[0] - sys->y[0] );
  cos_d = ( kx*c + ky*s + 1 ) / sqrt( zSqr(kx+c) + zSqr(ky+s) );
  ndw = ( dw + (sys->nx[0]-sys->x[0])*c + (sys->ny[0]-sys->y[0])*s ) / cos_d;
  if( !zIsTiny(kappa) )
    ndw += ( 1 - cos_d ) / ( kappa * cos_d );
  kw = 1.0 + kappa * ndw;
  sys->xd = sys->xd + (sys->nx[0]-sys->xd)/kw - ndw*c/kw;
  sys->yd = sys->yd + (sys->ny[0]-sys->yd)/kw - ndw*s/kw;
  sys->theta = atan2( (kappa*(sys->ny[0]-sys->yd)+s)/kw, (kappa*(sys->nx[0]-sys->xd)+c)/kw );
}

void dmSystemUpdateRef(dmSystem *sys)
{
  /* automatic activation when walking */
  if( !zIsTiny( sys->com.vud ) ){
    pdCtrlPrmRad(&sys->c)->rho = 1.0;
  } else {
    pdCtrlPrmRad(&sys->c)->rho = sys->com.rho;
  }

  /* automatic update of referential position and orientation */
  _dmSystemUpdateRefPosTheta( sys );

  /* update IK constraints for COM */
  zVec3DCreate( &dm_d_com, sys->nx[0], sys->ny[0], sys->c.vrt.zd );
  zVec3DCreate( &dm_d_att_body, sys->theta+zPI_2, 0, 0 );
  printf( "theta: %g\n", sys->theta );
}

void dmSystemInitFoot(dmSystem *sys)
{
  dmRobotFootPos( &sys->lf.p, &sys->rf.p );
  dmRobotFootPos( &sys->lf.pd, &sys->rf.pd );
  dmRobotFootPos( &sys->lf.ps, &sys->rf.ps );
  dmRobotFootAtt( &sys->lf.a, &sys->rf.a );
  dmRobotFootAtt( &sys->lf.as, &sys->rf.as );

  /* spring-damper tracking */
  sys->lf.stride_x = 1;
  sys->lf.track_kx = 3000;
  sys->lf.track_cx = 50;
  sys->lf.track_xold = zVec3DElem(&sys->lf.p,zX);
  sys->lf.stride_y = 1;
  sys->lf.track_ky = 3000;
  sys->lf.track_cy = 50;
  sys->lf.track_yold = zVec3DElem(&sys->lf.p,zY);
  sys->lf.track_kz = 3000;
  sys->lf.track_cz = 50;
  sys->lf.track_zold = zVec3DElem(&sys->lf.p,zZ);
  sys->rf.stride_x = 1;
  sys->rf.track_kx = 3000;
  sys->rf.track_cx = 50;
  sys->rf.track_xold = zVec3DElem(&sys->rf.p,zX);
  sys->rf.stride_y = 1;
  sys->rf.track_ky = 3000;
  sys->rf.track_cy = 50;
  sys->rf.track_yold = zVec3DElem(&sys->rf.p,zY);
  sys->rf.track_kz = 3000;
  sys->rf.track_cz = 50;
  sys->rf.track_zold = zVec3DElem(&sys->rf.p,zZ);
  /* stepping range */
  sys->lf.xsfor = 0.2;
  sys->lf.xsaft =-0.2;
  sys->lf.ysin  = zVec3DElem(&sys->lf.pd,zY) + 0.042;
  sys->lf.ysout = 0.2;
  sys->lf.xfc   = 0.0;
  sys->lf.yfc   = 0.042;
  sys->rf.xsfor = 0.2;
  sys->rf.xsaft =-0.2;
  sys->rf.ysin  = zVec3DElem(&sys->rf.pd,zY) - 0.042;
  sys->rf.ysout =-0.2;
  sys->rf.xfc   = 0.0;
  sys->rf.yfc   = 0.042;
  /* initial position of swing foot */
  sys->lf.xk0 = zVec3DElem(&sys->lf.p,zX);
  sys->lf.yk0 = zVec3DElem(&sys->lf.p,zY);
  sys->rf.xk0 = zVec3DElem(&sys->rf.p,zX);
  sys->rf.yk0 = zVec3DElem(&sys->rf.p,zY);
  sys->lf.is_bf = false;
  sys->lf.is_ff = false;
  sys->rf.is_bf = false;
  sys->rf.is_ff = false;
}

void dmSystemInitConsole(dmSystem *sys, dmConsole *con)
{
  dmConsoleInit( con );
  dmConsoleAddEval( con, "COM height", 0.24, 0.28, 0.26, 0, &sys->com.zd );
  dmConsoleAddEval( con, "VU-ref", -0.3, 0.3, 0, 20, &sys->com.vud );
  dmConsoleAddEval( con, "U-pole 1", 0.0, 2.0, 1.0, 0, &sys->com.qu1 );
  dmConsoleAddEval( con, "U-pole 2", 0.0, 2.0, 0.0, 0, &sys->com.qu2 );
  dmConsoleAddEval( con, "VW-ref", -0.15, 0.15, 0, 20, &sys->com.vwd );
  dmConsoleAddEval( con, "W-pole 1", 0.0, 2.0, 1.0, 0, &sys->com.qw1 );
  dmConsoleAddEval( con, "W-pole 2", 0.0, 2.0, 1.5, 0, &sys->com.qw2 );
  dmConsoleAddEval( con, "Foot dist", 0.02, 0.16, 0.1, 0, &sys->com.dist );
  dmConsoleAddEval( con, "Kappa", -3.0, 3.0, 0.0, 20, &sys->com.kappa );
  dmConsoleAddEval( con, "W-activation", 0, 1, 0, 0, &sys->com.rho );
  dmConsoleAddEval( con, "W-initiation", 0.5, 2, 1, 0, &sys->com.kr );
  dmConsoleAddEval( con, "L lift height", 0, 0.02, 0.02, 0, &sys->com.lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.02, 0.02, 0, &sys->com.rfh );
}

void dmSystemUpdateCommand(dmSystem *sys)
{
  /* pdCtrlSetPrm( &sys->c, sys->com.qu1, sys->com.qu2, sys->com.qw1, sys->com.qw2, sys->com.kappa, sys->com.rho, sys->com.kr ); */
  pdCtrlPrmTan(&sys->c)->q1 = sys->com.qu1;
  pdCtrlPrmTan(&sys->c)->q2 = sys->com.qu2;
  pdCtrlPrmTan(&sys->c)->kappa = sys->com.kappa;
  pdCtrlPrmRad(&sys->c)->q1 = sys->com.qw1;
  pdCtrlPrmRad(&sys->c)->q2 = sys->com.qw2;
  pdCtrlPrmRad(&sys->c)->kappa = sys->com.kappa;
  /* pdCtrlPrmRad(&sys->c)->rho = sys->com.rho; */
  pdCtrlPrmRad(&sys->c)->kr = sys->com.kr;
  pdCtrlSetRefVrt( &sys->c, sys->com.zd );
  pdCtrlSetRefHrz( &sys->c, sys->com.vud, sys->com.vwd, sys->com.dist );
  sys->lf.h   = sys->com.lfh;
  sys->rf.h   = sys->com.rfh;
}

void dmSystemInitState(dmSystem *sys)
{
  sys->x[0] = 0.5 * ( sys->xzmin + sys->xzmax );
  sys->y[0] = 0.5 * ( sys->yzmin + sys->yzmax );
  sys->x[1] = 0;
  sys->y[1] = 0;
  sys->xz = sys->x[0];
  sys->yz = sys->y[0];
  sys->xd = sys->x[0];
  sys->yd = sys->y[0];
  sys->theta = 0;
  sys->adx = sys->ady = 0;
}

void dmSystemInit(dmSystem *sys, dmConsole *con)
{
  pdCtrlInit( &sys->c );

  dmSystemUpdateRobot( sys );
  dmSystemInitConsole( sys, con );
  dmSystemUpdateCommand( sys );
  dmSystemInitState( sys );
  dmSystemInitFoot( sys );
}

void dmSystemExit(dmSystem *sys)
{
  pdCtrlDestroy( &sys->c );
}

void dmSystemODESolverInit(dmODESolver *solver, dmSystem *sys)
{
  solver->t = 0;
  solver->p = zVecCreateList( 4, sys->x[0], sys->x[1], sys->y[0], sys->y[1] );
  zODEAssign( &solver->ode, RKF45, NULL, NULL );
  zODEInit( &solver->ode, 4, 0, dp );
}

void dmSystemODESolverExit(dmODESolver *solver)
{
  zODEDestroy( &solver->ode );
}

#define DM_CONSOLE_WIDTH 240
void resize(zxWindow *win, dmConsole *con, dmScene *sx, dmScene *sy)
{
  short w, h;

  zxWindowClear( win );
  w = ( zxWindowWidth(win) - DM_CONSOLE_WIDTH ) / 2;
  h = zxWindowHeight(win) / 2;

  dmConsoleMove( con, w*2, 0 );
  dmConsoleDraw( con, win );

  zxwSepBoxLower( win, 6, 6, w-4, 2*h-12 );
  zxwSepBoxLower( win, w+6, 6, w-4, 2*h-12 );
  dmSceneResize( sx, w, 0, w, h );
  dmSceneResize( sy, 0, 0, w, h );

  zxDequeueEvent(); /* flush unprocessed events */
}

void capture(zxWindow *win)
{
  static int count = 0;
  static char str[BUFSIZ];
  zxImage img;

  sprintf( str, "dynmorph%05d.bmp", count++ );
  zxImageFromPixmap( &img, zxCanvas(win), zxWindowWidth(win), zxWindowHeight(win) );
  zxImageWriteBMPFile( &img, str );
  zxImageDestroy( &img );
}

typedef struct{
  bool pause;
  bool frame;
  bool perturb_x;
  bool perturb_y;
  bool rec;
  bool log;
} dmFlagset;

void dmFlagsetInit(dmFlagset *flag)
{
  flag->pause = false;
  flag->frame = false;
  flag->perturb_x = false;
  flag->perturb_y = false;
  flag->rec = false;
  flag->log = false;
}

void frame_one(zxWindow *win, dmConsole *con, dmSystem *sys, dmODESolver *solver, dmScene *sx, dmScene *sy, dmFlagset *flag)
{
  zVec3D force = { { 0, 0, 0 } };
  double s, c;

  zVec3DCreate( &force, sys->adx, sys->ady, 0 );
  if( !flag->frame || flag->frame ){
    if( flag->frame ) flag->frame = false;
    /* udpate state */
    dmSystemUpdateCommand( sys );
    dmSystemUpdateCtrl( sys, solver );
    dmSystemUpdateFoot( sys );
    dmSystemUpdateRobot( sys );
    dmSystemUpdateRef( sys );
    /* printf("kappa: %g, vu: %g, dist: %g, zd: %g             \r", pdCtrlKappa(&sys->c), pdCtrlPrmTan(&sys->c)->vd, pdCtrlPrmRad(&sys->c)->dist, sys->c.vrt.zd ); */
  }
  zSinCos( sys->theta, &s, &c );
  dmSceneLookAt( sx, sys->xd-4*c, sys->yd-4*s, 0.4, sys->xd, sys->yd, 0.3);
  dmSceneLookAt( sy, sys->xd+4*s, sys->yd-4*c, 0.4, sys->xd, sys->yd, 0.3 );
  dmSceneDraw( sx, &force );
  dmSceneDraw( sy, &force );
}

#define ANIM_SKIP 1000
void mainloop(zxWindow *win, dmConsole *con, dmSystem *sys, dmODESolver *solver, dmScene *sx, dmScene *sy)
{
  dmFlagset flag;
  int count = ANIM_SKIP;

  dmFlagsetInit( &flag );
  while( 1 ){
    switch( zxGetEvent() ){
    case Expose:
    case ConfigureNotify:
      zxWindowUpdateRegion( win );
      resize( win, con, sx, sy );
      break;
    case ClientMessage:
      if( zxDeleteWindowEvent() ) return;
      break;
    case ButtonPress:
      dmConsoleButtonPress( con );
      break;
    case ButtonRelease:
      dmConsoleButtonRelease( con );
      break;
    case MotionNotify:
      dmConsoleMouseMove( con, win );
      break;
    case KeyPress:
      switch( zxKeySymbol() ){
      case XK_p: flag.pause = 1 - flag.pause; break;
      case XK_f: flag.frame = flag.pause; break;
      case XK_r: flag.rec = 1 - flag.rec; break;
      case XK_Up: sys->ady = 0.1; break;
      case XK_Down: sys->ady = -0.1; break;
      case XK_Left: sys->adx = -0.1; break;
      case XK_Right: sys->adx = +0.1; break;
      case XK_q: return;
      }
      break;
    case KeyRelease:
      sys->adx = sys->ady = 0;
      break;
    default: ;
    }
    if( ++count > ANIM_SKIP ){
      frame_one( win, con, sys, solver, sx, sy, &flag );
      count = 0;
      if( flag.rec ) capture( win );
    }
  }
}

#define WIDTH 960
#define HEIGHT 640
int main(int argc, char *argv[])
{
  zxWindow mainwin;
  dmConsole con;
  dmSystem sys;
  dmScene sx, sy;
  dmODESolver solver;

  glrkInitGLX();
  zxWindowCreate( &mainwin, 0, 0, WIDTH, HEIGHT );
  zxWindowSetBG( &mainwin, (char *)"lightgray" );
  zxWindowClear( &mainwin );
  zxKeyEnable( &mainwin );
  zxMouseEnable( &mainwin );
  zxWindowSetTitle( &mainwin, (char *)"dynamics morphing" );
  zxWindowOpen( &mainwin );
  zxWidgetInit( &mainwin );

  dmSceneInit( &sx, &mainwin );
  dmSceneInit( &sy, &mainwin );

  dmRobotInit();

  dmSystemInit( &sys, &con );
  dmSystemODESolverInit( &solver, &sys );
  mainloop( &mainwin, &con, &sys, &solver, &sx, &sy );
  dmSystemODESolverExit( &solver );
  dmSystemExit( &sys );

  dmConsoleExit( &con );
  dmRobotExit();

  dmSceneExit( &sx );
  dmSceneExit( &sy );

  glrkCloseGLX();
  return 0;
}
