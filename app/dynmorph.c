#include <zm/zm_ode.h>
#include <zx11/zxwidget.h>
#include <zx11/zximage_dib.h>
#include <pedi2/pd_cz.h>

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
  pdCZ c;
  dmFoot lf, rf;
  double xz, yz;
  double xzmin, xzmax;
  double yzmin, yzmax;
  double xd, yd;
  double theta;
  double nxd, nyd;
  double ntheta;
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
  *dw = ( sys->xd - sys->x[0] )*c + ( sys->yd - sys->y[0] )*s;
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
  pdCZUpdate( &sys->c, du, vu, dw, vw );
  dmSystemCoodTransBodyToWorld( sys, pdCZZMPTan(&sys->c), pdCZZMPRad(&sys->c), &sys->xz, &sys->yz );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCZZeta(&sys->c)) * ( zVecElem(p,0) - sys->xz ) + sys->adx;
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCZZeta(&sys->c)) * ( zVecElem(p,2) - sys->yz ) + sys->ady;
  return v;
}

#define DT 0.01
void dmSystemUpdateCtrl(dmSystem *sys, dmODESolver *solver)
{
  solver->t += DT;
  sys->x[0] = sys->nx[0];
  sys->x[1] = sys->nx[1];
  sys->y[0] = sys->ny[0];
  sys->y[1] = sys->ny[1];
  sys->xd = sys->nxd;
  sys->yd = sys->nyd;
  sys->theta = sys->ntheta;
  zODEUpdate( &solver->ode, solver->t, solver->p, DT, sys );
  sys->nx[0] = zVecElem(solver->p,0);
  sys->nx[1] = zVecElem(solver->p,1);
  sys->ny[0] = zVecElem(solver->p,2);
  sys->ny[1] = zVecElem(solver->p,3);
}

void dmSystemUpdateFoot(dmSystem *sys)
{
  double vw;
  double s, c;
  zComplex pz;

  zSinCos( sys->theta, &s, &c );
  dmRobotFootPos( &sys->lf.p, &sys->rf.p );
  dmRobotFootAtt( &sys->lf.a, &sys->rf.a );
  vw = -sys->x[1]*c - sys->y[1]*s;
  dmCtrlZMPPhase( &sys->c, vw, &pz );
  dmRobotFootRegion( &sys->lf.yout, &sys->lf.yin, &sys->lf.dy, &sys->rf.yout, &sys->rf.yin, &sys->rf.dy );
  dmFootLift( &sys->c, &sys->lf, &sys->rf, sys->xd, sys->yd, sys->theta, &pz );
  dmFootMove( &sys->c, &sys->lf, &sys->rf, sys->xd, sys->yd, sys->theta );
  dmFootUpdatePos( &sys->lf, &sys->rf );

  /* update IK constraints for feet */
  zVec3DCopy( &sys->lf.ps, &dm_d_lf );
  zVec3DCopy( &sys->rf.ps, &dm_d_rf );
  zVec3DCopy( &sys->lf.as, &dm_d_att_lf );
  zVec3DCopy( &sys->rf.as, &dm_d_att_rf );
}

void _dmSystemUpdateRefPosTheta(dmSystem *sys)
{
  double kappa;
  double dw, ndw;
  double s, c;
  double cos_d;
  double kx, ky, kw;

  kappa = pdCZKappa(&sys->c);
  zSinCos( sys->theta, &s, &c );
  dw = -( sys->xd - sys->x[0] )*c - ( sys->yd - sys->y[0] )*s;

  kx = kappa * ( sys->nx[0] - sys->x[0] );
  ky = kappa * ( sys->ny[0] - sys->y[0] );
  cos_d = ( kx*c + ky*s + 1 ) / sqrt( zSqr(kx+c) + zSqr(ky+s) );
  ndw = ( dw + (sys->nx[0]-sys->x[0])*c + (sys->ny[0]-sys->y[0])*s ) / cos_d;
  if( !zIsTiny(kappa) )
    ndw += ( 1 - cos_d ) / ( kappa * cos_d );
  kw = 1.0 + kappa * ndw;
  sys->nxd = ( kappa*ndw*sys->xd + sys->nx[0] - ndw*c )/kw;
  sys->nyd = ( kappa*ndw*sys->yd + sys->ny[0] - ndw*s )/kw;
  sys->ntheta = atan2( ( kappa*(sys->ny[0]-sys->yd) + s )/kw,
                      ( kappa*(sys->nx[0]-sys->xd) + c )/kw );
}

void dmSystemUpdateRef(dmSystem *sys)
{
  /* automatic activation when walking */
  if( !zIsTiny( sys->com.vud ) ){
    pdCZPrmRad(&sys->c)->rho = 1.0;
  } else {
    pdCZPrmRad(&sys->c)->rho = sys->com.rho;
  }

  /* automatic update of referential position and orientation */
  _dmSystemUpdateRefPosTheta( sys );

  /* update IK constraints for COM */
  zVec3DCreate( &dm_d_com, sys->nx[0], sys->ny[0], sys->c.vrt.zd );
  zVec3DCreate( &dm_d_att_body, sys->theta+zPI_2, 0, 0 );
}

void dmSystemInitFoot(dmSystem *sys)
{
  double d;
  double s, c;

  dmRobotFootPos( &sys->lf.p, &sys->rf.p );
  dmRobotFootPos( &sys->lf.pd, &sys->rf.pd );
  dmRobotFootPos( &sys->lf.ps, &sys->rf.ps );
  dmRobotFootAtt( &sys->lf.a, &sys->rf.a );
  dmRobotFootAtt( &sys->lf.as, &sys->rf.as );

  d = 0.5 * pdCZPrmRad(&sys->c)->dist;
  zSinCos( sys->theta, &s, &c );
  zVec3DCreate( &dm_d_lf, sys->xd-d*c, sys->yd-d*s, 0 );
  zVec3DCreate( &dm_d_rf, sys->xd+d*c, sys->yd+d*s, 0 );
  zVec3DCreate( &dm_d_att_lf, sys->theta+zPI_2, 0, 0 );
  zVec3DCreate( &dm_d_att_rf, sys->theta+zPI_2, 0, 0 );

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
  dmConsoleAddEval( con, "L lift height", 0, 0.04, 0.04, 0, &sys->com.lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.04, 0.04, 0, &sys->com.rfh );
}

void dmSystemUpdateCommand(dmSystem *sys)
{
  /* pdCZSetPrm( &sys->c, sys->com.qu1, sys->com.qu2, sys->com.qw1, sys->com.qw2, sys->com.kappa, sys->com.rho, sys->com.kr ); */
  pdCZPrmTan(&sys->c)->q1 = sys->com.qu1;
  pdCZPrmTan(&sys->c)->q2 = sys->com.qu2;
  pdCZPrmTan(&sys->c)->kappa = sys->com.kappa;
  pdCZPrmRad(&sys->c)->q1 = sys->com.qw1;
  pdCZPrmRad(&sys->c)->q2 = sys->com.qw2;
  pdCZPrmRad(&sys->c)->kappa = sys->com.kappa;
  /* pdCZPrmRad(&sys->c)->rho = sys->com.rho; */
  pdCZPrmRad(&sys->c)->kr = sys->com.kr;
  pdCZSetRefVrt( &sys->c, sys->com.zd );
  pdCZSetRefHrz( &sys->c, sys->com.vud, sys->com.vwd, sys->com.dist );
  sys->lf.h   = sys->com.lfh;
  sys->rf.h   = sys->com.rfh;
}

void dmSystemInitState(dmSystem *sys)
{
  sys->x[0] = 0.5 * ( sys->xzmin + sys->xzmax );
  sys->y[0] = 0.5 * ( sys->yzmin + sys->yzmax );
  sys->x[1] = 0.001;
  sys->y[1] = 0;
  sys->xz = sys->x[0];
  sys->yz = sys->y[0];
  sys->xd = sys->nxd = sys->x[0];
  sys->yd = sys->nyd = sys->y[0];
  sys->nx[0] = sys->x[0];
  sys->nx[1] = sys->x[1];
  sys->ny[0] = sys->y[0];
  sys->ny[1] = sys->y[1];
  sys->theta = sys->ntheta = 0;
  sys->adx = sys->ady = 0;
}

void dmSystemInit(dmSystem *sys, dmConsole *con)
{
  pdCZInit( &sys->c );

  dmSystemInitConsole( sys, con );
  dmSystemUpdateCommand( sys );
  dmSystemInitState( sys );
  dmSystemInitFoot( sys );
  dmSystemUpdateRobot( sys );
}

void dmSystemExit(dmSystem *sys)
{
  pdCZDestroy( &sys->c );
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

void dmSystemLog(FILE *fp, dmSystem *sys)
{
  fprintf( fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n",
           sys->x[0], sys->x[1], sys->nx[0], sys->nx[1],
           sys->y[0], sys->y[1], sys->ny[0], sys->ny[1],
           sys->xz, sys->xd, sys->yz, sys->yd,
           sys->theta );
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

void frame_one(zxWindow *win, dmConsole *con, dmSystem *sys, dmODESolver *solver, dmScene *sx, dmScene *sy, dmFlagset *flag, FILE *fp)
{
  zVec3D force = { { 0, 0, 0 } };
  double s, c;

  zVec3DCreate( &force, sys->adx, sys->ady, 0 );
  if( !flag->pause || flag->frame ){
    if( flag->frame ) flag->frame = false;
    /* udpate state */
    dmSystemUpdateCommand( sys );
    dmSystemUpdateCtrl( sys, solver );
    dmSystemUpdateFoot( sys );
    dmSystemUpdateRobot( sys );
    dmSystemUpdateRef( sys );
    if( fp )
      dmSystemLog( fp, sys );
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
  FILE *fp = NULL;
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
      case XK_l:
        if( ( flag.log = 1 - flag.log ) ){
          fp = fopen( "dynmorph.log", "w" );
        } else{
          fclose( fp );
        }
        break;
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
      frame_one( win, con, sys, solver, sx, sy, &flag, fp );
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
