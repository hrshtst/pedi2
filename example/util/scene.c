#include "scene.h"

void initScene(Scene *scene, zxWindow *parent)
{
  GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };

  scene->canvas = glrkWindowCreateGLX( parent, 0, 0, 1, 1, NULL );
  glrkWindowOpenGLX( scene->canvas );

  glrkBGSet( &scene->cam, 0.6, 0.6, 0.6 );
  glrkCALookAt( &scene->cam, 8, 0, 0.3, 0, 0, 0.3, 0, 0, 1 );

  glEnable( GL_LIGHTING );
  glrkLightCreate( &scene->light, 0, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0, 0, 0, 0 );
  glrkLightSetPos( &scene->light, 10, 0, 4 );

  scene->gauge = glrkGauge( zX, 6.0, zY, 6.0, 1.0, 0.2, white );
}

void exitScene(Scene *scene)
{
  glDeleteLists( scene->gauge, 1 );
  glrkWindowDestroyGLX( scene->canvas );
}

void lookAt(Scene *scene, double cx, double cy, double cz, double fx, double fy, double fz)
{
  glrkCALookAt( &scene->cam, cx, cy, cz, fx, fy, fz, 0, 0, 1 );
}

void resizeScene(Scene *scene, int x, int y, int w, int h)
{
  double wx, wy;

  zxRegionSet( &scene->reg, x, y, w, h );
  wx = 0.1;
  XMoveWindow( zxdisplay, scene->canvas, scene->reg.x, scene->reg.y );
  XResizeWindow( zxdisplay, scene->canvas, scene->reg.width, scene->reg.height );
  glrkVPCreate( &scene->cam, 0, 0, scene->reg.width, scene->reg.height );
  wy = wx / glrkVPAspect(&scene->cam);
  glrkFrustum( &scene->cam, -wx, wx, -wy, wy, 1, 20 );
}

void _create_sphere(zVec3D *cen, double r, double g, double b)
{
  zOpticalInfo oi;
  zSphere3D sphere;

  zOpticalInfoCreateSimple( &oi, r, g, b, NULL );
  zSphere3DCreate( &sphere, cen, 0.02, 0 );
  glrkMaterial( &oi );
  glrkSphere( &sphere );
}

void _create_cylinder(zVec3D *c1, zVec3D *c2)
{
  zOpticalInfo oi;
  zCyl3D cyl;

  zOpticalInfoCreateSimple( &oi, 0.5, 0.5, 0.5, NULL );
  zCyl3DCreate( &cyl, c1, c2, 0.005, 0 );
  glrkMaterial( &oi );
  glrkCyl( &cyl );
}

void drawScene(Scene *scene, zVec3D *com, zVec3D *zmp)
{
  glrkActivateGLX( scene->canvas );
  glrkClear();
  /* camera */
  glrkCALoad( &scene->cam );
  /* gauge */
  glDisable( GL_LIGHTING );
  glCallList( scene->gauge );
  glEnable( GL_LIGHTING );
  /* light */
  glrkLightPut( &scene->light );
  /* stuff */
  _create_sphere( com, 0.8, 0, 0 );
  _create_sphere( zmp, 0, 0.8, 0 );
  _create_cylinder( com, zmp );
  /* buffering */
  glrkSwapBuffersGLX( scene->canvas );
  glrkFlushGLX();
}
