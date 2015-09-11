#include <zx11/zxwidget.h>
#include <zx11/zximage_dib.h>
#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_robot.h>
#include "util/dm_scene.h"
#include "util/dm_console.h"

typedef enum{
  DM_MODEL_INVALID=-1,
  DM_MODEL_MIGHTY=0,
  DM_MODEL_HYDRA,
} dmModel;

void init_console_mighty(dmConsole *con, pdCmd *cmd)
{
  dmConsoleAddEval( con, "COM height", 0.24, 0.28, 0.26, 0, &cmd->zd );
  dmConsoleAddEval( con, "VU-ref", -0.3, 0.3, 0, 20, &cmd->vud );
  dmConsoleAddEval( con, "U-pole 1", 0.0, 2.0, 1.0, 0, &cmd->qu1 );
  dmConsoleAddEval( con, "U-pole 2", 0.0, 2.0, 0.0, 0, &cmd->qu2 );
  dmConsoleAddEval( con, "VW-ref", -0.15, 0.15, 0, 20, &cmd->vwd );
  dmConsoleAddEval( con, "W-pole 1", 0.0, 2.0, 1.0, 0, &cmd->qw1 );
  dmConsoleAddEval( con, "W-pole 2", 0.0, 2.0, 1.5, 0, &cmd->qw2 );
  dmConsoleAddEval( con, "Foot dist", 0.084, 0.16, 0.1, 0, &cmd->dist );
  dmConsoleAddEval( con, "Kappa", -3.0, 3.0, 0.0, 20, &cmd->kappa );
  dmConsoleAddEval( con, "W-activation", 0, 1, 0, 0, &cmd->rho );
  dmConsoleAddEval( con, "W-initiation", 0.5, 2, 1, 0, &cmd->kr );
  dmConsoleAddEval( con, "L lift height", 0, 0.04, 0.04, 0, &cmd->lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.04, 0.04, 0, &cmd->rfh );
}

void init_console_hydra(dmConsole *con, pdCmd *cmd)
{
  dmConsoleAddEval( con, "COM height", 0.8, 1.0, 0.95, 0, &cmd->zd );
  dmConsoleAddEval( con, "VU-ref", -0.3, 0.3, 0, 20, &cmd->vud );
  dmConsoleAddEval( con, "U-pole 1", 0.0, 2.0, 1.0, 0, &cmd->qu1 );
  dmConsoleAddEval( con, "U-pole 2", 0.0, 2.0, 0.0, 0, &cmd->qu2 );
  dmConsoleAddEval( con, "VW-ref", -0.15, 0.15, 0, 20, &cmd->vwd );
  dmConsoleAddEval( con, "W-pole 1", 0.0, 2.0, 1.0, 0, &cmd->qw1 );
  dmConsoleAddEval( con, "W-pole 2", 0.0, 2.0, 0.3, 0, &cmd->qw2 );
  dmConsoleAddEval( con, "Foot dist", 0.05, 0.5, 0.3, 0, &cmd->dist );
  dmConsoleAddEval( con, "Kappa", -3.0, 3.0, 0.0, 20, &cmd->kappa );
  dmConsoleAddEval( con, "W-activation", 0, 1, 0, 0, &cmd->rho );
  dmConsoleAddEval( con, "W-initiation", 0.5, 2, 1, 0, &cmd->kr );
  dmConsoleAddEval( con, "L lift height", 0, 0.2, 0.1, 0, &cmd->lfh );
  dmConsoleAddEval( con, "R lift height", 0, 0.2, 0.1, 0, &cmd->rfh );
}

void init_console(dmConsole *con, pdCmd *cmd, dmModel model)
{
  pdCmdDefaultInit( cmd );
  dmConsoleInit( con );
  if( model == DM_MODEL_MIGHTY )
    init_console_mighty( con, cmd );
  else if( model == DM_MODEL_HYDRA )
    init_console_hydra( con, cmd );
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
  dmSceneResize( sx, 0, 0, w, h );
  dmSceneResize( sy, w, 0, w, h );

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

#define ANIM_SKEW 2
void frame_one(zxWindow *win, pdRobot *robot, pdBiped *biped, pdState *state, dmConsole *con, dmScene *sx, dmScene *sy, dmFlagset *flag, dmModel model, FILE *fp)
{
  zVec3D force = { { 0, 0, 0 } };
  double theta;
  double s, c;
  double xd, yd;
  zVec dis;
  static int anim_skew_cnt;

  dis = zVecAlloc( pdRobotJointSize(robot) );
  zVecCopy( pdRobotJointDis( robot ), dis );
  /* zVec3DCreate( &force, core->adx, core->ady, 0 ); */
  if( !flag->pause || flag->frame ){
    if( flag->frame ) flag->frame = false;
    /* udpate controller */
    pdBipedUpdate( biped );
    pdRobotSetBipedRefVec( robot, biped );
    pdRobotSolveIK( robot );
    /* update state */
    pdBipedUpdateState( biped );
    pdRobotUpdateState( robot, state );
    if( fp )
      pdBipedDataFWrite( fp, biped );
  }
  theta = zVec3DElem( &state->base_att, 0 );
  zSinCos( theta, &s, &c );
  xd = biped->cmd->xd;
  yd = biped->cmd->yd;
  if( model == DM_MODEL_MIGHTY ){
    dmSceneLookAt( sx, xd-4*c, yd-4*s, 0.4, xd, yd, 0.3 );
    dmSceneLookAt( sy, xd+4*s, yd-4*c, 0.4, xd, yd, 0.3 );
  } else if ( model == DM_MODEL_HYDRA ) {
    dmSceneLookAt( sx, xd-10*c, yd-10*s, 1, xd, yd, 0.8 );
    dmSceneLookAt( sy, xd+10*s, yd-10*c, 1, xd, yd, 0.8 );
  }
  if( ++anim_skew_cnt > ANIM_SKEW ){
    dmSceneDraw( sx, dis, &force );
    dmSceneDraw( sy, dis, &force );
    anim_skew_cnt = 0;
  }
  zVecFree( dis );
}

#define ANIM_SKIP 1000
void mainloop(zxWindow *win, pdRobot *robot, pdBiped *biped, pdState *state, dmConsole *con, dmScene *sx, dmScene *sy, dmModel model)
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
      /* case XK_Up: core->ady = 0.1; break; */
      /* case XK_Down: core->ady = -0.1; break; */
      /* case XK_Left: core->adx = -0.1; break; */
      /* case XK_Right: core->adx = +0.1; break; */
      case XK_q: return;
      }
      break;
    case KeyRelease:
      /* core->adx = core->ady = 0; */
      break;
    default: ;
    }
    if( ++count > ANIM_SKIP ){
      frame_one( win, robot, biped, state, con, sx, sy, &flag, model, fp );
      count = 0;
      if( flag.rec ) capture( win );
    }
  }
}

#define WIDTH 960
#define HEIGHT 640
#define DT 0.01
int main(int argc, char *argv[])
{
  zxWindow mainwin;
  pdCmd cmd;
  pdState state;
  pdBiped biped;
  pdRobot robot;
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

  rkglInitGLX();
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
  init_console( &con, &cmd, model );

  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, &state, DT );
  pdRobotInit( &robot );
  if( model == DM_MODEL_MIGHTY ){
    if( !pdRobotLoad( &robot, "../model/mighty.zkc" ) )
      exit( EXIT_FAILURE );
    dmGLInit( "../model/mighty.zkc" );
  } else if ( model == DM_MODEL_HYDRA ) {
    if( !pdRobotLoad( &robot, "../model/hydra.zkc" ) )
      exit( EXIT_FAILURE );
    dmGLInit( "../model/hydra.zkc" );
  } else {
    ZRUNERROR( "invalid model" );
    exit( EXIT_FAILURE );
  }
  pdRobotUpdateState( &robot, &state );
  pdBipedDefaultPoseInit( &biped );
  pdRobotSetBipedRefVec( &robot, &biped );
  pdRobotSolveIK( &robot );
  pdRobotUpdateState( &robot, &state );

  mainloop( &mainwin, &robot, &biped, &state, &con, &sx, &sy, model );

  dmGLExit();
  pdRobotDestroy( &robot );
  pdBipedDestroy( &biped );
  pdStateDestroy( &state );
  pdCmdDestroy( &cmd );

  dmConsoleExit( &con );
  dmSceneExit( &sx );
  dmSceneExit( &sy );

  rkglCloseGLX();
  return 0;
}
