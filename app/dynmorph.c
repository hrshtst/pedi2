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
} dmComVal;

typedef struct{
  double x[2], y[2];
  pdCtrl c;
  dmFoot lf, rf;
  double xz, yz;
  double xzmin, xzmax;
  double yzmin, yzmax;
  dmComVal com;
} dmSystem;

void dmSystemUpdateRobot(dmSystem *sys)
{
  dmRobotSolveIK();
  dmSupportRegion();
  dmSupportRegionBorder( &sys->xzmin, &sys->xzmax, &sys->yzmin, &sys->yzmax );
}

#define UPDATE_SKEW 1
void dmSystemUpdateCtrl(dmSystem *sys)
{
}

void dmSystemUpdateFoot(dmSystem *sys)
{
  dmRobotFootPos( &sys->lf.p, &sys->rf.p );

  /* update IK constraints for feet */
  zVec3DCopy( &sys->lf.pds, &dm_d_lf );
  zVec3DCopy( &sys->rf.pds, &dm_d_rf );
}

void dmSystemUpdateRef(dmSystem *sys)
{
  /* update IK constraints for COM */
  zVec3DCreate( &dm_d_com, sys->x[0], sys->y[0], sys->c.vrt.zd );
}

void dmSystemInitFoot(dmSystem *sys)
{
  dmRobotFootPos( &sys->lf.p, &sys->rf.p );
  dmRobotFootPos( &sys->lf.pd, &sys->rf.pd );
  dmRobotFootPos( &sys->lf.pds, &sys->rf.pds );
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
  dmConsoleAddEval( con, "U-pole 2", 0.0, 2.0, 0.5, 0, &sys->com.qu2 );
  dmConsoleAddEval( con, "VW-ref", -0.15, 0.15, 0, 20, &sys->com.vwd );
  dmConsoleAddEval( con, "W-pole 1", 0.0, 2.0, 1.0, 0, &sys->com.qw1 );
  dmConsoleAddEval( con, "W-pole 2", 0.0, 2.0, 0.5, 0, &sys->com.qw2 );
  dmConsoleAddEval( con, "Foot dist", 0.02, 0.16, 0.1, 0, &sys->com.dist );
  dmConsoleAddEval( con, "Kappa", -2.0, 2.0, 0.0, 20, &sys->com.kappa );
  dmConsoleAddEval( con, "W-activation", 0, 1, 0, 0, &sys->com.rho );
  dmConsoleAddEval( con, "W-initiation", 0.5, 2, 1, 0, &sys->com.kr );
  dmConsoleAddEval( con, "L lift height", 0, 0.02, 0.02, 0, &sys->com.lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.02, 0.02, 0, &sys->com.rfh );
}

void dmSystemUpdateComVal(dmSystem *sys)
{
  pdCtrlSetPrm( &sys->c, sys->com.qu1, sys->com.qu2, sys->com.qw1, sys->com.qw2, sys->com.kappa, sys->com.rho, sys->com.kr );
  pdCtrlSetRefVrt( &sys->c, sys->com.zd );
  pdCtrlSetRefHrz( &sys->c, sys->com.vud, sys->com.vwd, sys->com.dist );
  /* sys->lf.h   = sys->cv.lfh; */
  /* sys->rf.h   = sys->cv.rfh; */
}

void dmSystemInitState(dmSystem *sys)
{
  sys->x[0] = 0.5 * ( sys->xzmin + sys->xzmax );
  sys->y[0] = 0.5 * ( sys->yzmin + sys->yzmax );
  sys->x[1] = 0;
  sys->y[1] = 0;
  sys->xz = sys->x[0];
  sys->yz = sys->y[0];
}

void dmSystemInit(dmSystem *sys, dmConsole *con)
{
  pdCtrlInit( &sys->c );

  dmSystemUpdateRobot( sys );
  dmSystemInitConsole( sys, con );
  dmSystemUpdateComVal( sys );
  dmSystemInitState( sys );
  dmSystemInitFoot( sys );
}

void dmSystemExit(dmSystem *sys)
{
  pdCtrlDestroy( &sys->c );
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

void frame_one(zxWindow *win, dmConsole *con, dmSystem *sys, dmScene *sx, dmScene *sy, dmFlagset *flag)
{
  zVec3D force = { { 0, 0, 0 } };

  if( !flag->frame || flag->frame ){
    if( flag->frame ) flag->frame = false;
    /* udpate state */
    dmSystemUpdateComVal( sys );
    dmSystemUpdateCtrl( sys );
    dmSystemUpdateFoot( sys );
    dmSystemUpdateRobot( sys );
    dmSystemUpdateRef( sys );
    /* printf("kappa: %g, vu: %g, dist: %g, zd: %g             \r", pdCtrlKappa(&sys->c), pdCtrlPrmTan(&sys->c)->vd, pdCtrlPrmRad(&sys->c)->dist, sys->c.vrt.zd ); */
  }
  dmSceneLookAt( sx, sys->x[0], -4, 0.4, sys->x[0], 0, 0.3);
  dmSceneLookAt( sy, sys->x[0]+4, 0, 0.4, sys->x[0], 0, 0.3 );
  dmSceneDraw( sx, &force );
  dmSceneDraw( sy, &force );
}

#define ANIM_SKIP 1000
void mainloop(zxWindow *win, dmConsole *con, dmSystem *sys, dmScene *sx, dmScene *sy)
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
      case XK_q: return;
      }
      break;
    default: ;
    }
    if( ++count > ANIM_SKIP ){
      frame_one( win, con, sys, sx, sy, &flag );
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
  mainloop( &mainwin, &con, &sys, &sx, &sy );
  dmSystemExit( &sys );

  dmConsoleExit( &con );
  dmRobotExit();

  dmSceneExit( &sx );
  dmSceneExit( &sy );

  glrkCloseGLX();
  return 0;
}
