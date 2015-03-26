#include "dm_scene.h"

/* scene stuff */
static rkChain dm_robot;
static glrkChain dm_gl_robot;
static int dm_gl_gauge;
static zOpticalInfo dm_gl_sr_oi;
static int dm_lf_id, dm_rf_id;
static zVec3DList dm_sr_lf, dm_sr_rf, dm_sr;

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

void dmSceneDraw(dmScene *scene, zVec dis, zVec3D *force)
{
  rkChainFK( &dm_robot, dis );
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

void dmGLInit(const char* model_file)
{
  GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
  register int i;

  /* load robot model file */
  if( !rkChainReadFile( &dm_robot, (char *)model_file ) )
    exit( EXIT_FAILURE );

  for( i=0; i<rkChainNum(&dm_robot); i++ ){
    if( !strcmp( "left_foot", rkChainLinkName(&dm_robot,i) ) )
      dm_lf_id = i;
    if( !strcmp( "right_foot", rkChainLinkName(&dm_robot,i) ) )
      dm_rf_id = i;
  }
  zListInit( &dm_sr_lf );
  zListInit( &dm_sr_rf );
  zListInit( &dm_sr );

  /* scene stuff */
  if( !glrkChainLoad( &dm_gl_robot, &dm_robot, NULL ) ){
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

#define DM_TOL (1.0e-3)
static void _dmSupportRegion(void)
{
  int i, nl, nr, n;
  rkLink *foot;
  zShape3D *sole;
  zVec3D v;
  zVec3D sr_lf_vert[4], sr_rf_vert[4], sr_vert[8];

  nl = nr = n = 0;
  /* left foot */
  foot = rkChainLink( &dm_robot, dm_lf_id );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < DM_TOL ){
      zVec3DCopy( &v, &sr_lf_vert[nl++] );
      zVec3DCopy( &v, &sr_vert[n++] );
    }
  }
  /* right foot */
  foot = rkChainLink( &dm_robot, dm_rf_id );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < DM_TOL ){
      zVec3DCopy( &v, &sr_rf_vert[nr++] );
      zVec3DCopy( &v, &sr_vert[n++] );
    }
  }
  /* supporting region */
  zVec3DListDestroy( &dm_sr_lf, false );
  zVec3DListDestroy( &dm_sr_rf, false );
  zVec3DListDestroy( &dm_sr, false );
  if( nl > 0 ) zCH2D( &dm_sr_lf, sr_lf_vert, nl );
  if( nr > 0 ) zCH2D( &dm_sr_rf, sr_rf_vert, nr );
  if( n  > 0 ) zCH2D( &dm_sr, sr_vert, n );
}

void dmGLSupportRegion(void)
{
  zVec3DListCell *vc;

  _dmSupportRegion();
  glBegin( GL_POLYGON );
    glShadeModel( GL_FLAT );
    glrkMaterial( &dm_gl_sr_oi );
    glrkNormal( Z_UNITZVEC3D );
    zListForEachRew( &dm_sr, vc )
      glrkVertex( vc->data );
  glEnd();
}
