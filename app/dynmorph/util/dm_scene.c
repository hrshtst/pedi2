#include "dm_scene.h"

/* scene stuff */
static rkChain dm_robot;
static rkglChain dm_gl_robot;
static int dm_gl_gauge;
static zOpticalInfo dm_gl_sr_oi;
static int dm_lf_id, dm_rf_id;
static zLoop3D dm_sr_lf, dm_sr_rf, dm_sr;

void dmSceneInit(dmScene *scene, zxWindow *parent)
{
  scene->canvas = rkglWindowCreateGLX( parent, 0, 0, 1, 1, NULL );
  rkglWindowKeyEnableGLX( scene->canvas );
  rkglWindowMouseEnableGLX( scene->canvas );
  rkglWindowOpenGLX( scene->canvas );

  rkglCameraInit( &scene->cam );
  rkglCameraSetBackground( &scene->cam, 0.6, 0.6, 0.6 );
  rkglCameraLookAt( &scene->cam, 8, 0, 0.3, 0, 0, 0.3, 0, 0, 1 );
  rkglSetDefaultCamera( &scene->cam );

  glEnable( GL_LIGHTING );
  rkglLightCreate( &scene->light, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0, 0, 0 );
  rkglLightMove( &scene->light, 10, 0, 4 );
}

void dmSceneExit(dmScene *scene)
{
  rkglWindowCloseGLX( scene->canvas );
  rkglWindowDestroyGLX( scene->canvas );
}

void dmSceneLookAt(dmScene *scene, double cx, double cy, double cz, double fx, double fy, double fz)
{
  rkglCameraLookAt( &scene->cam, cx, cy, cz, fx, fy, fz, 0, 0, 1 );
}

void dmSceneResize(dmScene *scene, int x, int y, int w, int h)
{
  double wx, wy;

  zxRegionSet( &scene->reg, x+8, y+8, w-8, h-8 );
  wx = 0.1;
  XMoveWindow( zxdisplay, scene->canvas, scene->reg.x, scene->reg.y );
  XResizeWindow( zxdisplay, scene->canvas, scene->reg.width, scene->reg.height );
  rkglCameraSetViewport( &scene->cam, 0, 0, scene->reg.width, scene->reg.height );
  wy = wx / rkglCameraViewportAspectRatio(&scene->cam);
  rkglCameraSetFrustum( &scene->cam, -wx, wx, -wy, wy, 1, 20 );
}

void dmSceneDraw(dmScene *scene, zVec dis, zVec3D *force)
{
  rkChainFK( &dm_robot, dis );
  rkglWindowActivateGLX( scene->canvas );
  rkglClear();
  rkglLightPut( &scene->light );
  rkglCameraPut( &scene->cam );

  dmGLGauge();
  dmGLRobot();
  dmGLSupportRegion();
  if( !zVec3DIsTiny( force ) ){
    GLfloat color[4] = { 0.1, 0.3, 0.8, 0.5 };
    glColor4fv( color );
    rkglArrow( rkChainWldCOM(&dm_robot), force, 0.3 );
  }

  rkglWindowSwapBuffersGLX( scene->canvas );
  rkglFlushGLX();
}

void dmGLInit(const char* model_file, rkglLight *light)
{
  register int i;

  /* load robot model file */
  if( !rkChainReadFile( &dm_robot, (char *)model_file ) )
    exit( EXIT_FAILURE );

  for( i=0; i<rkChainLinkNum(&dm_robot); i++ ){
    if( !strcmp( "left_foot", rkChainLinkName(&dm_robot,i) ) )
      dm_lf_id = i;
    if( !strcmp( "right_foot", rkChainLinkName(&dm_robot,i) ) )
      dm_rf_id = i;
  }
  zListInit( &dm_sr_lf );
  zListInit( &dm_sr_rf );
  zListInit( &dm_sr );

  /* scene stuff */
  if( !rkglChainLoad( &dm_gl_robot, &dm_robot, NULL, light ) ){
    ZRUNERROR( "unable to load robot model" );
    exit( EXIT_FAILURE );
  }
  dm_gl_gauge = rkglBeginList();
  rkglRGBByStr( "white" );
  glLineWidth( 0.5 );
  rkglGauge( zX, 6.0, zY, 6.0, 0.2 );
  glEndList();
  zOpticalInfoCreateSimple( &dm_gl_sr_oi, 1.0, 0.4, 0.8, NULL );
}

void dmGLExit(void)
{
  rkglChainUnload( &dm_gl_robot );
  glDeleteLists( dm_gl_gauge, 1 );
}

void dmGLGauge(void)
{
  glCallList( dm_gl_gauge );
}

void dmGLRobot(void)
{
  rkglChainDraw( &dm_gl_robot );
}

#define DM_TOL (1.0e-3)
static void _dmSupportRegion(void)
{
  int i;
  rkLink *foot;
  zShape3D *sole;
  zVec3D v;
  zVec3DData sr_lf_vert, sr_rf_vert, sr_vert;

  zVec3DDataInitArray( &sr_lf_vert, 4 );
  zVec3DDataInitArray( &sr_rf_vert, 4 );
  zVec3DDataInitArray( &sr_vert, 8 );
  /* left foot */
  foot = rkChainLink( &dm_robot, dm_lf_id );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXform3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( v.c.z < DM_TOL ){
      zVec3DDataAdd( &sr_lf_vert, &v );
      zVec3DDataAdd( &sr_vert, &v );
    }
  }
  /* right foot */
  foot = rkChainLink( &dm_robot, dm_rf_id );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXform3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( v.c.z < DM_TOL ){
      zVec3DDataAdd( &sr_rf_vert, &v );
      zVec3DDataAdd( &sr_vert, &v );
    }
  }
  /* supporting region */
  zLoop3DDestroy( &dm_sr_lf );
  zLoop3DDestroy( &dm_sr_rf );
  zLoop3DDestroy( &dm_sr );
  if( zVec3DDataSize( &sr_lf_vert ) > 0 ) zVec3DDataConvexHull2D( &sr_lf_vert, &dm_sr_lf );
  if( zVec3DDataSize( &sr_rf_vert ) > 0 ) zVec3DDataConvexHull2D( &sr_rf_vert, &dm_sr_rf );
  if( zVec3DDataSize( &sr_vert ) > 0 ) zVec3DDataConvexHull2D( &sr_vert, &dm_sr );
  zVec3DDataDestroy( &sr_lf_vert );
  zVec3DDataDestroy( &sr_rf_vert );
  zVec3DDataDestroy( &sr_vert );
}

void dmGLSupportRegion(void)
{
  zLoop3DCell *cp;

  _dmSupportRegion();
  glBegin( GL_POLYGON );
    glShadeModel( GL_FLAT );
    rkglMaterial( &dm_gl_sr_oi );
    rkglNormal( ZVEC3DZ );
    zListForEachRew( &dm_sr, cp )
      rkglVertex( cp->data );
  glEnd();
}
