#include <roki/rokigl.h>
#include <roki/rkgl_glx.h>
#include <zx11/zximage_png.h>
#include <liw/liw_time.h>
#include <pedi2/pd_cmd.h>
#include <pedi2/pd_core.h>
#include <aviator.h>
#include <pthread.h>

#define JOYSTICK_CTRL_TITLE "Joystick Control"

#define JOYSTICK_CTRL_MODEL "../model/mighty.zkc"

#define JOYSTICK_CTRL_BUFSIZ 512
static rkChain robot;
static rkglChain gl_robot;

static rkglCamera cam;
static rkglLight light;
static int gl_gauge;

static zxWindow win;
static Window glwin;

static pdCmd cmd;
static pdCore core;
static zVec dis;

static pthread_t thread;

#define DATALOGFILE "data.log"
#define SRLOGFILE   "sr.log"
static FILE *data_fp = NULL;
static FILE *sr_fp = NULL;
static bool is_logging = false;

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

#define WIDTH 500
#define HEIGHT 500
void joystickCtrlInit(void)
{
  short width, height;
  GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };

  width = WIDTH;
  height = HEIGHT;
  zxWindowCreate( &win, 0, 0, width+8, height+32 );
  zxWindowSetTitle( &win, JOYSTICK_CTRL_TITLE );
  zxWindowOpen( &win );
  zxWindowSetBG( &win, (char *)"lightgray" );
  zxWindowSetFG( &win, (char *)"black" );
  zxDoubleBufferEnable( &win );
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

  gl_gauge = rkglGauge( zX, 6.0, zY, 6.0, 1.0, 0.2, white );

  joystickCtrlAviatorInit();
  pthread_create( &thread, NULL, joystickCtrlCommand, (void *)NULL );
}

#define DT 0.005
void joystickCtrlLoad(void)
{
  pdCmdDefaultInit( &cmd );
  cmd.zd = 0.26;
  cmd.dist = 0.1;
  cmd.lfh = 0.02;
  cmd.rfh = 0.02;
  pdCoreInit( &core, &cmd, DT );
  if( !pdCoreLoad( &core, JOYSTICK_CTRL_MODEL ) )
    exit( EXIT_FAILURE );

  if( !rkChainReadFile( &robot, (char *)JOYSTICK_CTRL_MODEL ) ||
      !rkglChainLoad( &gl_robot, &robot, NULL ) ){
    ZOPENERROR( JOYSTICK_CTRL_MODEL );
    exit( EXIT_FAILURE );
  }

  dis = zVecAlloc( pdCoreJointSize( &core ) );
}

void joystickCtrlSetCamera(void)
{
  double theta;
  double s, c;
  double xd, yd;

  theta = zVec3DElem( &core.state.base_att, 0 );
  zSinCos( theta, &s, &c );
  xd = core.cmd->xd;
  yd = core.cmd->yd;
  rkglCALookAt( &cam, xd-5*c, yd-5*s, 0.6, xd, yd, 0.3, 0, 0, 1 );
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

void joystickCtrlGauge(void)
{
  glDisable( GL_LIGHTING );
  glCallList( gl_gauge );
  glEnable( GL_LIGHTING );
}

void joystickCtrlDraw(void)
{
  rkglChainDraw( &gl_robot );
}

void joystickCtrlDisplay(void)
{
  rkglActivateGLX( glwin );
  rkglClear();
  rkglCALoad( &cam );
  rkglLightPut( &light );
  joystickCtrlGauge();
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

void joystickCtrlCapture(void)
{
  zxImage img;
  static char imgfile[JOYSTICK_CTRL_BUFSIZ];
  static int cnt = 0;

  sprintf( imgfile, "capture%05d.png", cnt++ );
  zxImageAllocDefault( &img, zxWindowWidth(&win), zxWindowHeight(&win) );
  printf("width:%d, height:%d\n", img.width, img.height );
  zxImageFromPixmap( &img, zxCanvas(&win), img.width, img.height );
  zxImageWritePNGFile( &img, imgfile );
  zxImageDestroy( &img );
}

void joystickCtrlLog(void)
{
  pdCoreDataFWrite( data_fp, &core );
  pdStateSRDataFWrite( sr_fp, &core.state );
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

void joystickCtrlDrawStatusbar(void)
{
  static char statusbar[JOYSTICK_CTRL_BUFSIZ];
  zxRegion reg;

  sprintf( statusbar,
           "vud:%0.3f vwd:%0.3f kappa:%0.3f",
           cmd.vud, cmd.vwd, cmd.kappa );
  zxTextArea( statusbar, 0, 0, &reg );
  zxWindowClear( &win );
  zxClear( &win );
  zxDrawString( &win, zxWindowWidth(&win)-reg.width-8, zxWindowHeight(&win)-8, statusbar );
  zxDoubleBufferPartAppear( &win, 4, zxWindowHeight(&win)-28, zxWindowWidth(&win)-8, 24 );
  zxFlush();
}

void joystickCtrlUpdate(void)
{
  pdCoreUpdate( &core );
  zVecCopy( pdCoreJointDis( &core ), dis );
  rkChainFK( &robot, dis );
  liwSleep( (long)DT, 0 );
}

void joystickCtrlPlay(void)
{
  joystickCtrlUpdate();
  joystickCtrlReshape();
  joystickCtrlDisplay();
  while( 1 ){
    if( joystickCtrlEvent() < 0 ) return;
    joystickCtrlUpdate();
    joystickCtrlDrawStatusbar();
    joystickCtrlRedisplay();
    if( is_logging )
      joystickCtrlLog();
  }
}

void joystickCtrlExit(void)
{
  aviator_close( &av );
  zVecFree( dis );
  rkglChainUnload( &gl_robot );
  rkChainDestroy( &robot );
  pdCoreDestroy( &core );
  glDeleteLists( gl_gauge, 1 );
  rkglWindowDestroyGLX( glwin );
  rkglCloseGLX();
  zxDoubleBufferDisable( &win );
  zxWindowDestroy( &win );
  pthread_cancel( thread );
}

int main(int argc, char *argv[])
{
  rkglInitGLX();
  joystickCtrlInit();
  joystickCtrlLoad();
  joystickCtrlPlay();
  joystickCtrlExit();
  return 0;
}
