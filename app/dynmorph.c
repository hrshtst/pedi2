#include <zx11/zxwidget.h>
#include <zx11/zximage_dib.h>
#include <pedi2/pd_ctrl.h>

#include "util/dm_scene.h"
#include "util/dm_console.h"

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
  double xz, yz;
  dmComVal com;
} dmSystem;

void dmSystemUpdateRobot(dmSystem *sys)
{
  dmRobotSolveIK();
  dmSupportRegion();
  /* dmSupportRegionBorder( &sys->cx.xzmin, &sys->cx.xzmax, &sys->cy.xzmin, &sys->cy.xzmax ); */
}

#define UPDATE_SKEW 1
void dmSystemUpdateCtrl(dmSystem *sys)
{
}

void dmSystemUpdateFoot(dmSystem *sys)
{
}

void dmSystemUpdateRef(dmSystem *sys)
{
}

void dmSystemInitFoot(dmSystem *sys)
{
}

void dmSystemInitConsole(dmSystem *sys, dmConsole *con)
{
  /* sys->cv.xd = 0.5 * ( sys->cx.xzmin + sys->cx.xzmax ); */
  /* sys->cv.yd = 0.5 * ( sys->cy.xzmin + sys->cy.xzmax ); */
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
  sys->x[0] = sys->x[1] = 0;
  sys->y[0] = sys->y[1] = 0;
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
    dmSystemUpdateRef( sys );
    printf("kappa: %g, vu: %g, dist: %g            \r", pdCtrlKappa(&sys->c), pdCtrlPrmTan(&sys->c)->vd, pdCtrlPrmRad(&sys->c)->dist );
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
