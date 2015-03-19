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

void dmSceneDraw(dmScene *scene, zVec3D *force)
{
  glrkActivateGLX( scene->canvas );
  glrkClear();
  glrkCALoad( &scene->cam );
  dmGLGauge();
  glrkLightPut( &scene->light );
  dmGLRobot();
  dmGLSupportRegion();
  if( !zVec3DIsTiny( force ) ){
    GLfloat color[4] = { 0.1, 0.3, 0.8, 0.5 };
    glrkArrow( rkChainWldCOM(&dm_robot), force, 0.3, color );
  }
  glrkSwapBuffersGLX( scene->canvas );
  glrkFlushGLX();
}
