#include <zx11/zxwidget.h>
#include <zx11/zximage_dib.h>
#include <pedi2/pd_core.h>

#include "util/dm_scene.h"
#include "util/dm_console.h"

typedef enum{
  DM_MODEL_INVALID=-1,
  DM_MODEL_MIGHTY=0,
  DM_MODEL_HYDRA,
} dmModel;

void init_console_mighty(dmConsole *con, pdCommand *com)
{
  dmConsoleAddEval( con, "COM height", 0.24, 0.28, 0.26, 0, &com->zd );
  dmConsoleAddEval( con, "VU-ref", -0.3, 0.3, 0, 20, &com->vud );
  dmConsoleAddEval( con, "U-pole 1", 0.0, 2.0, 1.0, 0, &com->qu1 );
  dmConsoleAddEval( con, "U-pole 2", 0.0, 2.0, 0.0, 0, &com->qu2 );
  dmConsoleAddEval( con, "VW-ref", -0.15, 0.15, 0, 20, &com->vwd );
  dmConsoleAddEval( con, "W-pole 1", 0.0, 2.0, 1.0, 0, &com->qw1 );
  dmConsoleAddEval( con, "W-pole 2", 0.0, 2.0, 1.5, 0, &com->qw2 );
  dmConsoleAddEval( con, "Foot dist", 0.02, 0.16, 0.1, 0, &com->dist );
  dmConsoleAddEval( con, "Kappa", -3.0, 3.0, 0.0, 20, &com->kappa );
  dmConsoleAddEval( con, "W-activation", 0, 1, 0, 0, &com->rho );
  dmConsoleAddEval( con, "W-initiation", 0.5, 2, 1, 0, &com->kr );
  dmConsoleAddEval( con, "L lift height", 0, 0.04, 0.04, 0, &com->lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.04, 0.04, 0, &com->rfh );
}

void init_console_hydra(dmConsole *con, pdCommand *com)
{
  dmConsoleAddEval( con, "COM height", 0.7, 0.95, 0.85, 0, &com->zd );
  dmConsoleAddEval( con, "VU-ref", -0.3, 0.3, 0, 20, &com->vud );
  dmConsoleAddEval( con, "U-pole 1", 0.0, 2.0, 1.0, 0, &com->qu1 );
  dmConsoleAddEval( con, "U-pole 2", 0.0, 2.0, 0.0, 0, &com->qu2 );
  dmConsoleAddEval( con, "VW-ref", -0.15, 0.15, 0, 20, &com->vwd );
  dmConsoleAddEval( con, "W-pole 1", 0.0, 2.0, 1.0, 0, &com->qw1 );
  dmConsoleAddEval( con, "W-pole 2", 0.0, 2.0, 1.5, 0, &com->qw2 );
  dmConsoleAddEval( con, "Foot dist", 0.05, 0.5, 0.35, 0, &com->dist );
  dmConsoleAddEval( con, "Kappa", -3.0, 3.0, 0.0, 20, &com->kappa );
  dmConsoleAddEval( con, "W-activation", 0, 1, 0, 0, &com->rho );
  dmConsoleAddEval( con, "W-initiation", 0.5, 2, 1, 0, &com->kr );
  dmConsoleAddEval( con, "L lift height", 0, 0.1, 0.1, 0, &com->lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.1, 0.1, 0, &com->rfh );
}

void init_console(dmConsole *con, pdCommand *com, dmModel model)
{
  dmConsoleInit( con );
  if( model == DM_MODEL_MIGHTY )
    init_console_mighty( con, com );
  else if( model == DM_MODEL_HYDRA )
    init_console_hydra( con, com );
  else {
    ZRUNERROR( "invalid model" );
    exit( EXIT_FAILURE );
  }
}


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

#define DT 0.01
void frame_one(zxWindow *win, pdCore *core, dmConsole *con, dmScene *sx, dmScene *sy, dmFlagset *flag, dmModel model, FILE *fp)
{
  zVec3D force = { { 0, 0, 0 } };
  double s, c;
  zVec dis;

  dis = zVecAlloc( pdCoreGetJointSize(core) );
  pdCoreGetJointDis( core, dis );
  zVec3DCreate( &force, core->adx, core->ady, 0 );
  if( !flag->pause || flag->frame ){
    if( flag->frame ) flag->frame = false;
    /* udpate state */
    pdCoreUpdate( core, DT );
    if( fp )
      pdCoreFWrite( core, fp );
  }
  zSinCos( core->theta, &s, &c );
  if( model == DM_MODEL_MIGHTY ){
    dmSceneLookAt( sx, core->xd-4*c, core->yd-4*s, 0.4, core->xd, core->yd, 0.3);
    dmSceneLookAt( sy, core->xd+4*s, core->yd-4*c, 0.4, core->xd, core->yd, 0.3 );
  } else if ( model == DM_MODEL_HYDRA ) {
    dmSceneLookAt( sx, core->xd-10*c, core->yd-10*s, 1, core->xd, core->yd, 0.8 );
    dmSceneLookAt( sy, core->xd+10*s, core->yd-10*c, 1, core->xd, core->yd, 0.8 );
  }
  dmSceneDraw( sx, dis, &force );
  dmSceneDraw( sy, dis, &force );
  zVecFree( dis );
}

#define ANIM_SKIP 1000
void mainloop(zxWindow *win, pdCore *core, dmConsole *con, dmScene *sx, dmScene *sy, dmModel model)
{
  FILE *fp = NULL;
  dmFlagset flag;
  int count = ANIM_SKIP;

  dmFlagsetInit( &flag );
  zxGetEvent();
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
      case XK_Up: core->ady = 0.1; break;
      case XK_Down: core->ady = -0.1; break;
      case XK_Left: core->adx = -0.1; break;
      case XK_Right: core->adx = +0.1; break;
      case XK_q: return;
      }
      break;
    case KeyRelease:
      core->adx = core->ady = 0;
      break;
    default: ;
    }
    if( ++count > ANIM_SKIP ){
      frame_one( win, core, con, sx, sy, &flag, model, fp );
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
  pdCommand com;
  pdCore core;
  dmConsole con;
  dmScene sx, sy;
  dmModel model;

  model = DM_MODEL_INVALID;
  if( argc > 1 ){
    if( !strcmp( argv[1], "mighty" ) )
      model = DM_MODEL_MIGHTY;
    else if( !strcmp( argv[1], "hydra" ) )
      model = DM_MODEL_HYDRA;
    else {
      ZRUNERROR( "invalid argument: %s ", argv[1] );
      exit( EXIT_FAILURE );
    }
  } else
    model = DM_MODEL_MIGHTY;

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
  init_console( &con, &com, model );

  pdCoreInit( &core, &com );
  if( model == DM_MODEL_MIGHTY ){
    pdCoreLoad( &core, "model/mighty.zkc", "model/mighty_ik.conf" );
    dmGLInit( "model/mighty.zkc" );
  } else if ( model == DM_MODEL_HYDRA ) {
    pdCoreLoad( &core, "model/hydra.zkc", "model/hydra_ik.conf" );
    dmGLInit( "model/hydra.zkc" );
  } else {
    ZRUNERROR( "invalid model" );
    exit( EXIT_FAILURE );
  }
  mainloop( &mainwin, &core, &con, &sx, &sy, model );
  dmGLExit();
  pdCoreExit( &core );

  dmConsoleExit( &con );
  dmSceneExit( &sx );
  dmSceneExit( &sy );

  glrkCloseGLX();
  return 0;
}
