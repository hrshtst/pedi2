#include "dm_scene.h"

void dmSceneInit(dmScene *scene, zxWindow *parent)
{
  scene->canvas = glrkWindowCreateGLX( parent, 0, 0, 1, 1, NULL );
  glrkWindowOpenGLX( scene->canvas );

  glrkBGSet( &scene->cam, 0.6, 0.6, 0.6 );
  glrkCALookAt( &scene->cam, 8, 0, 0.3, 0, 0, 0.3, 0, 0, 1 );

  glEnable( GL_LIGHTING );
  glrkLightCreate( &scene->light, 0, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0, 0, 0, 0 );
  glrkLightSetPos( &scene->light, 10, 0, 4 );
}

void dmSceneExit(dmScene *scene)
{
  glrkWindowDestroyGLX( scene->canvas );
}

void dmSceneLookAt(dmScene *scene, double cx, double cy, double cz, double fx, double fy, double fz)
{
  glrkCALookAt( &scene->cam, cx, cy, cz, fx, fy, fz, 0, 0, 1 );
}

void dmSceneResize(dmScene *scene, int x, int y, int w, int h)
{
  double wx, wy;

  zxRegionSet( &scene->reg, x+8, y+8, w-8, h-8 );
  wx = 0.1;
  XMoveWindow( zxdisplay, scene->canvas, scene->reg.x, scene->reg.y );
  XResizeWindow( zxdisplay, scene->canvas, scene->reg.width, scene->reg.height );
  glrkVPCreate( &scene->cam, 0, 0, scene->reg.width, scene->reg.height );
  wy = wx / glrkVPAspect(&scene->cam);
  glrkFrustum( &scene->cam, -wx, wx, -wy, wy, 1, 20 );
}

void dmSceneDraw(dmScene *scene, pdRobot *robot, zVec3D *force)
{
  glrkActivateGLX( scene->canvas );
  glrkClear();
  glrkCALoad( &scene->cam );
  dmGLGauge();
  glrkLightPut( &scene->light );
  dmGLRobot();
  dmGLSupportRegion( robot );
  if( !zVec3DIsTiny( force ) ){
    GLfloat color[4] = { 0.1, 0.3, 0.8, 0.5 };
    glrkArrow( rkChainWldCOM(&robot->chain), force, 0.3, color );
  }
  glrkSwapBuffersGLX( scene->canvas );
  glrkFlushGLX();
}

/* scene stuff */
static glrkChain dm_gl_robot;
static int dm_gl_gauge;
static zOpticalInfo dm_gl_sr_oi;

void dmGLInit(pdRobot *robot)
{
  GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };

  /* scene stuff */
  if( !glrkChainLoad( &dm_gl_robot, &robot->chain, NULL ) ){
    ZRUNERROR( "unable to load robot model" );
    exit( EXIT_FAILURE );
  }
  dm_gl_gauge = glrkGauge( zX, 6.0, zY, 6.0, 1.0, 0.2, white );
  zOpticalInfoCreateSimple( &dm_gl_sr_oi, 1.0, 0.4, 0.8, NULL );
}

void dmGLExit(void)
{
  glrkChainUnload( &dm_gl_robot );
  glDeleteLists( dm_gl_gauge, 1 );
}

void dmGLGauge(void)
{
  glDisable( GL_LIGHTING );
  glCallList( dm_gl_gauge );
  glEnable( GL_LIGHTING );
}

void dmGLRobot(void)
{
  glrkChainDraw( &dm_gl_robot );
}

void dmGLSupportRegion(pdRobot *robot)
{
  zVec3DListCell *vc;

  glBegin( GL_POLYGON );
    glShadeModel( GL_FLAT );
    glrkMaterial( &dm_gl_sr_oi );
    glrkNormal( Z_UNITZVEC3D );
    zListForEachRew( &robot->sr, vc )
      glrkVertex( vc->data );
  glEnd();
}
