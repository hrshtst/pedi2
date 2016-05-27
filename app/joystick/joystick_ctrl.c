#include <roki/rokigl.h>
#include <roki/rkgl_glx.h>
#include <zx11/zximage_png.h>
#include <liw/liw_time.h>
#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_robot.h>
#include <aviator.h>
#include <pthread.h>

#define JOYSTICK_CTRL_TITLE "Joystick Operation"

enum{
  OPT_TITLE=0,
  OPT_ENVFILE,
  OPT_WIDTH, OPT_HEIGHT,
  OPT_DT,
  OPT_QU1, OPT_QU2, OPT_QW1, OPT_QW2,
  OPT_ZD, OPT_DIST, OPT_HMAX,
  OPT_REPLAY,
  OPT_HELP,
  OPT_INVALID
};
zOption opt[] = {
  { "title", NULL, "<title name>", "title of sequence", (char *)"anim", false },
  { "env", NULL, "<.z3d/.zkc file>", "environment model file", NULL, false },
  { "width", NULL, "<width>", "set window width", (char *)"500", false },
  { "height", NULL, "<height>", "set window height", (char *)"500", false },
  { "dt", NULL, "<dt value>", "time step", (char *)"0.01", false },
  { "qu1", NULL, "<qu1 value>", "control parameter qu1", (char *)"0", false },
  { "qu2", NULL, "<qu2 value>", "control parameter qu2", (char *)"0", false },
  { "qw1", NULL, "<qw1 value>", "control parameter qw1", (char *)"0", false },
  { "qw2", NULL, "<qw2 value>", "control parameter qw2", (char *)"0", false },
  { "zd",   NULL, "<value>", "referential position of COM height", (char *)"0", false },
  { "dist", NULL, "<value>", "distance of the feet", (char *)"0", false },
  { "hmax", NULL, "<value>", "maximal height of the foot", (char *)"0", false },
  { "R", NULL, "<cmd.log>", "replay with command log and recording", (char *)"cmd.log", false },
  { "help", NULL, NULL, "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

#define JOYSTICK_CTRL_BUFSIZ 512
static rkChain chain;
static rkglChain gl_chain;

static rkChain chain_env;
static rkglChain ge;
static int env = 0;
static rkglCamera cam;
static rkglLight light;
static int path = 0;

static zxWindow win;
static Window glwin;

static pdCmd cmd;
static pdState state;
static pdBiped biped;
static pdRobot robot;
static zVec dis;

static pthread_t thread;

#define DATALOGFILE "data.log"
#define SRLOGFILE   "sr.log"
#define CMDLOGFILE  "cmd.log"
static FILE *data_fp = NULL;
static FILE *sr_fp = NULL;
static FILE *cmd_fp = NULL;
static bool is_logging = false;
static bool is_recording = false;
static bool is_cmdlogging = false;

#define DEVFILE "/dev/input/js0"
static aviator_t av;

void roll_move(aviator_t *av, int value, void *arg){ av->roll = value; }
void pitch_move(aviator_t *av, int value, void *arg){ av->pitch = value; }
void yaw_move(aviator_t *av, int value, void *arg){ av->yaw = value; }

void joystickCtrlAviatorInit(void)
{
  aviator_open( &av, DEVFILE );
  aviator_lever_entry( &av, AV_EV_ROLL,  roll_move );
  aviator_lever_entry( &av, AV_EV_PITCH, pitch_move );
  aviator_lever_entry( &av, AV_EV_YAW,   yaw_move );
}

static void* joystickCtrlCommand(void *args);
void* joystickCtrlCommand(void *args)
{
#define JOYSTICK_CTRL_VAL_LIM 32767
  int last_state, last_type, last_tmp;

  pthread_setcanceltype( PTHREAD_CANCEL_ASYNCHRONOUS, &last_type );
  pthread_setcancelstate( PTHREAD_CANCEL_ENABLE, &last_state );
  while( 1 ){
    aviator_action( &av, NULL );
    pthread_setcanceltype( PTHREAD_CANCEL_DEFERRED, &last_tmp );
    cmd.vud   = -0.3 * av.pitch / JOYSTICK_CTRL_VAL_LIM;
    cmd.vwd   = -0.1 * av.roll  / JOYSTICK_CTRL_VAL_LIM;
    cmd.kappa = -3.0 * av.yaw   / JOYSTICK_CTRL_VAL_LIM;
    pthread_setcanceltype( PTHREAD_CANCEL_ASYNCHRONOUS, &last_tmp );
  }
  return NULL;
}

void joystickCtrlUsage(void)
{
  eprintf( "Usage: joystick_ctrl <.zkc file> [options]\n" );
  eprintf( "<.zkc file>\tkinematic chain model file\n" );
  eprintf( "[options]\n" );
  zOptionHelp( opt );
  exit( 0 );
}

void joystickCtrlLoad(char modelfile[])
{
  pdCmdDefaultInit( &cmd );
  if( opt[OPT_QU1].flag ) cmd.qu1 = atof(opt[OPT_QU1].arg);
  if( opt[OPT_QU2].flag ) cmd.qu2 = atof(opt[OPT_QU2].arg);
  if( opt[OPT_QW1].flag ) cmd.qw1 = atof(opt[OPT_QW1].arg);
  if( opt[OPT_QW2].flag ) cmd.qw2 = atof(opt[OPT_QW2].arg);
  if( opt[OPT_ZD].flag ) cmd.zd = atof(opt[OPT_ZD].arg);
  if( opt[OPT_DIST].flag ) cmd.dist = atof(opt[OPT_DIST].arg);
  if( opt[OPT_HMAX].flag ) {
    cmd.lfh = atof(opt[OPT_HMAX].arg);
    cmd.rfh = atof(opt[OPT_HMAX].arg);
  }
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, atof( opt[OPT_DT].arg ) );
  pdRobotInit( &robot );
  if( !pdRobotLoad( &robot, modelfile ) )
    exit( 1 );

  if( !rkChainReadFile( &chain, modelfile ) ||
      !rkglChainLoad( &gl_chain, &chain, NULL ) ){
    ZOPENERROR( modelfile );
    exit( 1 );
  }

  dis = zVecAlloc( pdRobotJointSize( &robot ) );

  pdRobotUpdateState( &robot, &state );
  pdBipedDefaultPoseInit( &biped, &state );
  pdRobotSetBipedRefVec( &robot, &biped );
  pdRobotSolveIK( &robot, 0 );
  pdRobotUpdateState( &robot, &state );

  if( !opt[OPT_HMAX].flag ) {
    cmd.lfh = cmd.rfh = 0.1 * cmd.zd;
  }

  pdRobotGetJointDisAll( &robot, dis );
  zVecSetElem( dis, 0, 2.5 );
  zVecSetElem( dis, 1, 3.5 );
  zVecSetElem( dis, 5, -zPI_2 );
  pdRobotResetPose( &robot, &biped, &state, dis );
}

int joystickCtrlLoadEnv(void)
{
  char *sfx;
  int entry;

  sfx = zGetSuffix( opt[OPT_ENVFILE].arg );
  if( !( strcmp( sfx, RK_CHAIN_SUFFIX ) == 0 ?
         rkChainReadFile( &chain_env, opt[OPT_ENVFILE].arg ) :
         rkChainMShape3DReadFile( &chain_env, opt[OPT_ENVFILE].arg  ) ) ){
    ZOPENERROR( opt[OPT_ENVFILE].arg );
    joystickCtrlUsage();
    exit( 1 );
  }
  if( !rkglChainLoad( &ge, &chain_env, NULL ) ) exit( 1 );

  entry = rkglBeginList();
  rkglChainDraw( &ge );
  glEndList();
  return entry;
}

void joystickCtrlInit(void)
{
  short width, height;
  GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };

  width = atoi( opt[OPT_WIDTH].arg );
  height = atoi( opt[OPT_HEIGHT].arg );
  zxWindowCreate( &win, 0, 0, width+8, height+32 );
  zxWindowSetTitle( &win, JOYSTICK_CTRL_TITLE );
  zxWindowOpen( &win );
  zxWindowSetBG( &win, (char *)"lightgray" );
  zxWindowSetFG( &win, (char *)"black" );
  zxWindowClear( &win );
  zxSetFont( &win, "-misc-fixed-medium-r-normal-*-24-*-*-*-*-*-*-*" );

  glwin = rkglWindowCreateGLX( &win, 4, 4, width, height, NULL );
  rkglKeyEnableGLX( glwin );
  rkglMouseEnableGLX( glwin );
  rkglWindowOpenGLX( glwin );

  rkglBGSet( &cam, 0.6, 0.6, 0.6 );
  rkglCALookAt( &cam, 8, 0, 0.3, 0, 0, 0.3, 0, 0, 1 );

  glEnable( GL_LIGHTING );
  rkglLightCreate( &light, 0, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0, 0, 0, 0 );
  rkglLightSetPos( &light, 10, 0, 4 );

  if( opt[OPT_ENVFILE].flag )
    env = joystickCtrlLoadEnv();
  else
    env = rkglGauge( zX, 6.0, zY, 6.0, 1.0, 0.2, white );

  joystickCtrlAviatorInit();
  pthread_create( &thread, NULL, joystickCtrlCommand, (void *)NULL );
}

bool joystickCtrlCommandArgs(int argc, char *argv[])
{
  zStrList arglist;

  if( argc <= 1 ) joystickCtrlUsage();
  zOptionRead( opt, argv, &arglist );
  if( opt[OPT_HELP].flag ) joystickCtrlUsage();
  if( zListIsEmpty( &arglist ) ){
    ZRUNERROR( "kinematic chain model not specified" );
    return false;
  }
  joystickCtrlInit();
  joystickCtrlLoad( zListHead( &arglist )->data );
  zStrListDestroy( &arglist, false );
  return true;
}

void joystickCtrlSetCamera(void)
{
  double theta;
  double s, c;
  double xd, yd, zd;

  theta = zVec3DElem( &state.base_att, 0 );
  zSinCos( theta, &s, &c );
  xd = cmd.xd;
  yd = cmd.yd;
  zd = cmd.zd;
  rkglCALookAt( &cam, xd-15*zd*c, yd-15*zd*s, 2.5*zd, xd, yd, zd, 0, 0, 1 );
}

void joystickCtrlReshape(void)
{
  zxRegion reg;
  double wx, wy;

  zxGetGeometry( glwin, &reg );
  rkglVPCreate( &cam, 0, 0, reg.width, reg.height );
  wx = 0.1;
  wy = wx / rkglVPAspect( &cam );
  rkglFrustum( &cam, -wx, wx, -wy, wy, 1, 20 );
}

void joystickCtrlDraw(void)
{
  if( opt[OPT_ENVFILE].flag ){
    glCallList( env );
  } else {
    glDisable( GL_LIGHTING );
    glCallList( env );
    glEnable( GL_LIGHTING );
  }
  if( path > 0 )
    glCallList( path );
  rkglChainDraw( &gl_chain );
}

void joystickCtrlDisplay(void)
{
  rkglActivateGLX( glwin );
  rkglClear();
  rkglCALoad( &cam );
  rkglLightPut( &light );
  joystickCtrlDraw();
  rkglSwapBuffersGLX( glwin );
  rkglFlushGLX();
}

void joystickCtrlRedisplay(void)
{
#define JOYSTICK_CTRL_DISPLAY_SKIP 5
  static int count = 0;

  if( count++ == JOYSTICK_CTRL_DISPLAY_SKIP ){
    count = 0;
    joystickCtrlSetCamera();
    joystickCtrlDisplay();
  }
}

void joystickCtrlDrawStatusbar(void)
{
  static char statusbar[JOYSTICK_CTRL_BUFSIZ];
  zxRegion reg;

  sprintf( statusbar,
           "vud:%0.3f vwd:%0.3f kappa:%0.3f",
           cmd.vud, cmd.vwd, cmd.kappa );
  zxTextArea( statusbar, 0, 0, &reg );
  zxWindowClear( &win );
  zxDrawString( &win, zxWindowWidth(&win)-reg.width-8, zxWindowHeight(&win)-8, statusbar );
  zxFlush();
}

void joystickCtrlCapture(void)
{
  zxImage img;
  static char imgfile[JOYSTICK_CTRL_BUFSIZ];
  static int cnt = 0;

  sprintf( imgfile, "capture%05d.png", cnt++ );
  zxImageAllocDefault( &img, zxWindowWidth(&win), zxWindowHeight(&win) );
  zxImageFromPixmap( &img, zxCanvas(&win), img.width, img.height );
  zxImageWritePNGFile( &img, imgfile );
  zxImageDestroy( &img );
}

void joystickCtrlLog(void)
{
  pdBipedDataFWrite( data_fp, &biped );
  pdStateSRDataFWrite( sr_fp, &state );
}

void joystickCtrlCommandLog(void)
{
  pdCmdDataFWrite( cmd_fp, &cmd );
}

bool joystickCtrlReplayIsTerminated(void)
{
  return !pdCmdDataFRead( cmd_fp, &cmd );
}

int joystickCtrlKeyPress(void)
{
  zxModkeyOn( zxKeySymbol() );
  switch( zxKeySymbol() ){
  case XK_c: case XK_C:
    eprintf( "capture\n" );
    joystickCtrlCapture();
    break;
  case XK_l: case XK_L:
    if( ( is_logging = 1 - is_logging ) ){
      data_fp = fopen( DATALOGFILE, "w" );
      sr_fp = fopen( SRLOGFILE, "w" );
      eprintf( "start logging.\n" );
    } else {
      fclose( data_fp );
      fclose( sr_fp );
      eprintf( "quit logging.\n" );
    }
    break;
  case XK_k: case XK_K:
    if( ( is_cmdlogging = 1 - is_cmdlogging ) ){
      cmd_fp = fopen( CMDLOGFILE, "w" );
      eprintf( "start command logging.\n" );
    } else {
      fclose( cmd_fp );
      eprintf( "quit command logging.\n" );
    }
    break;
  case XK_r: case XK_R:
    if( (is_recording = 1 - is_recording ) ){
      eprintf( "start recording.\n" );
    } else {
      eprintf( "quit recording.\n" );
    }
    break;
  case XK_q: case XK_Q:
    eprintf( "quit.\n" );
    return -1;
  }
  return 0;
}

int joystickCtrlEvent(void)
{
  switch( zxDequeueEvent() ){
  case Expose:
  case ConfigureNotify: joystickCtrlReshape();   break;
  case KeyPress:
    if( joystickCtrlKeyPress() < 0 ) return -1;  break;
  case KeyRelease: zxModkeyOff( zxKeySymbol() ); break;
  default: ;
  }
  return 0;
}

void joystickCtrlUpdate(void)
{
  pdBipedUpdate( &biped, &state );
  pdRobotSetBipedRefVec( &robot, &biped );
  pdRobotSolveIK( &robot, 0 );
  zVecCopy( pdRobotJointDis( &robot ), dis );
  rkChainFK( &chain, dis );
  pdBipedUpdateState( &biped, &state );
  pdRobotUpdateState( &robot, &state );
  liwSleep( (long)atof( opt[OPT_DT].arg ), 0 );
}

int _joystickCtrlDrawArc(zVec3D *org, double r, double theta1, double theta2, double w, GLfloat color[])
{
#define JOYSTICK_CTRL_ARC_DIV 0.01
  int entry;
  double th_s, th_e;
  zEdge3D edge;
  zVec3D e1, e2;

  th_s = zMin( theta1, theta2 );
  th_e = zMax( theta1, theta2 );

  entry = rkglBeginList();
  glLineWidth( w );
  glColor3fv( color );
  for( ; th_s <= th_e; th_s += JOYSTICK_CTRL_ARC_DIV ){
    zVec3DCreatePolar( &e1, r, zPI_2, th_s );
    zVec3DCreatePolar( &e2, r, zPI_2, th_s+JOYSTICK_CTRL_ARC_DIV );
    zVec3DAddDRC( &e1, org );
    zVec3DAddDRC( &e2, org );
    zEdge3DCreate( &edge, &e1, &e2 );
    rkglEdge( &edge );
  }
  glEndList();
  return entry;
}

int joystickCtrlDrawArc(zVec3D *start, double kappa, double theta, double alpha, double w, GLfloat color[])
{
  zVec3D v, org;
  double s, c;
  double r, theta1, theta2;

  if( zIsTiny( kappa ) ){
    ZRUNERROR( "kappa == 0 cannot be approved" );
    return 0;
  }
  theta1 = theta;
  zSinCos( theta, &s, &c );
  zVec3DCreate( &v, c, s, 0 );
  r = 1 / kappa;
  zVec3DMulDRC( &v, -r );
  zVec3DAdd( start, &v, &org );
  theta2 = theta1 + alpha;
  return _joystickCtrlDrawArc( &org, r, theta1, theta2, w, color );
}

int _joystickCtrlDrawLine(zVec3D *s, zVec3D *e, double w, GLfloat color[])
{
  int entry;
  zEdge3D edge;

  entry = rkglBeginList();
  glLineWidth( w );
  glColor3fv( color );
  zEdge3DCreate( &edge, s, e );
  rkglEdge( &edge );
  glEndList();
  return entry;
}

int joystickCtrlDrawLine(zVec3D *start, double r, double theta, double alpha, double w, GLfloat color[])
{
  zVec3D v, end;
  double s, c;
  double d;

  zSinCos( theta, &s, &c );
  zVec3DCreate( &v, -s, c, 0 );
  d = r*sin( alpha );
  zVec3DMulDRC( &v, d );
  zVec3DAdd( start, &v, &end );
  return _joystickCtrlDrawLine( start, &end, w, color );
}

void joystickCtrlUpdatePath(void)
{
  zVec3D v;
  GLfloat color[4] = { 1.0, 0.0, 0.0, 1.0 }; /* red */
  double alpha;
  zOpticalInfo oi;

  alpha = zPI_2;
  zOpticalInfoCreateSimple( &oi, color[0], color[1], color[2], NULL );
  rkglMaterial( &oi );
  zVec3DCreate( &v, cmd.xd, cmd.yd, 0 );
  if( zIsTiny( cmd.kappa ) && cmd.vud > 0 )
    path = joystickCtrlDrawLine( &v, 2, cmd.thetad, alpha, 5.0, color );
  else if( zIsTiny( cmd.kappa ) && cmd.vud < 0 )
    path = joystickCtrlDrawLine( &v, -2, cmd.thetad, alpha, 5.0, color );
  else if( cmd.kappa > 0 && cmd.vud > 0 )
    path = joystickCtrlDrawArc( &v, cmd.kappa, cmd.thetad, alpha, 5.0, color );
  else if( cmd.kappa < 0 && cmd.vud > 0 )
    path = joystickCtrlDrawArc( &v, cmd.kappa, cmd.thetad, -alpha, 5.0, color );
  else if( cmd.kappa > 0 && cmd.vud < 0 )
    path = joystickCtrlDrawArc( &v, cmd.kappa, cmd.thetad, -alpha, 5.0, color );
  else if( cmd.kappa < 0 && cmd.vud < 0 )
    path = joystickCtrlDrawArc( &v, cmd.kappa, cmd.thetad, alpha, 5.0, color );
  else
    path = 0;
}

void joystickCtrlPlay(void)
{
  joystickCtrlUpdate();
  joystickCtrlUpdatePath();
  joystickCtrlReshape();
  joystickCtrlDisplay();
  if( opt[OPT_REPLAY].flag ){
    pthread_cancel( thread );
    pthread_join( thread, NULL );
    cmd_fp = fopen( opt[OPT_REPLAY].arg, "r" );
    while( !joystickCtrlReplayIsTerminated() ){
      joystickCtrlUpdate();
      joystickCtrlUpdatePath();
      joystickCtrlDrawStatusbar();
      joystickCtrlRedisplay();
      joystickCtrlCapture();
    }
    fclose( cmd_fp );
  } else {
    while( 1 ){
      if( joystickCtrlEvent() < 0 ) return;
      joystickCtrlUpdate();
      joystickCtrlUpdatePath();
      joystickCtrlDrawStatusbar();
      joystickCtrlRedisplay();
      if( is_logging )
        joystickCtrlLog();
      if( is_recording )
        joystickCtrlCapture();
      if( is_cmdlogging )
        joystickCtrlCommandLog();
    }
  }
}

void joystickCtrlExit(void)
{
  aviator_close( &av );
  zVecFree( dis );
  rkglChainUnload( &gl_chain );
  rkChainDestroy( &chain );
  pdRobotDestroy( &robot );
  pdBipedDestroy( &biped );
  pdStateDestroy( &state );
  pdCmdDestroy( &cmd );
  glDeleteLists( env, 1 );
  rkglWindowDestroyGLX( glwin );
  rkglCloseGLX();
  zxWindowDestroy( &win );
  pthread_cancel( thread );
  pthread_join( thread, NULL );
}

int main(int argc, char *argv[])
{
  rkglInitGLX();
  if( !joystickCtrlCommandArgs( argc, argv+1 ) ) return 1;
  joystickCtrlPlay();
  joystickCtrlExit();
  return 0;
}
