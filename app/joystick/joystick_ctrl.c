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
static bool is_running;

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
  while( is_running ){
    aviator_action( &av, NULL );
    cmd.vud   = -0.3  * av.pitch / JOYSTICK_CTRL_VAL_LIM;
    cmd.vwd   = -0.15 * av.roll  / JOYSTICK_CTRL_VAL_LIM;
    cmd.kappa = -3.0  * av.yaw   / JOYSTICK_CTRL_VAL_LIM;
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
  zxWindowCreate( &win, 0, 0, width+8, height+8 );
  zxWindowSetTitle( &win, JOYSTICK_CTRL_TITLE );
  zxWindowOpen( &win );
  zxWindowSetBG( &win, (char *)"lightgray" );
  zxWindowClear( &win );

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
  is_running = true;
  pthread_create( &thread, NULL, joystickCtrlCommand, (void *)NULL );
}

#define DT 0.01
void joystickCtrlLoad(void)
{
  pdCmdDefaultInit( &cmd );
  cmd.zd = 0.26;
  cmd.dist = 0.084;
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

int joystickCtrlKeyPress(void)
{
  zxModkeyOn( zxKeySymbol() );
  switch( zxKeySymbol() ){
  case XK_q: case XK_Q:
    is_running = false;
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
  pdCoreUpdate( &core );
  zVecCopy( pdCoreJointDis( &core ), dis );
  rkChainFK( &robot, dis );
  liwSleep( DT, 0 );
}

void joystickCtrlPlay(void)
{
  joystickCtrlUpdate();
  joystickCtrlReshape();
  joystickCtrlDisplay();
  while( 1 ){
    if( joystickCtrlEvent() < 0 ) return;
    joystickCtrlUpdate();
    joystickCtrlRedisplay();
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
  zxWindowDestroy( &win );
  pthread_join( thread, NULL );
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
