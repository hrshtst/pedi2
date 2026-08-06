#include <unistd.h>
#include <zm/zm_ode.h>
#include <zeo/zeo_vec3d.h>
#include <zx11/zxutil.h>
#include <roki_gl/roki_glx.h>
#include <roki_gl/rkgl_camera.h>
#include <roki_gl/rkgl_optic.h>
#include <roki_gl/rkgl_shape.h>
#include <pedi2/pd_cz.h>
#include <liw/liw_time.h>

zVec3D com;
zVec3D vel;
zVec3D acc;
zVec3D zmp;
zVec3D comd;
double theta, thetad;

typedef struct{
  rkglCamera cam;
  rkglLight light;
  Window canvas;
  zxRegion reg;
  int gauge;
} Scene;

void initScene(Scene *scene, zxWindow *parent)
{
  scene->canvas = rkglWindowCreateGLX( parent, 0, 0, 1, 1, NULL );
  rkglWindowOpenGLX( scene->canvas );

  rkglCameraInit( &scene->cam );
  rkglCameraSetBackground( &scene->cam, 0.6, 0.6, 0.6 );
  rkglCameraLookAt( &scene->cam, 8, 0, 0.3, 0, 0, 0.3, 0, 0, 1 );
  rkglSetDefaultCamera( &scene->cam );

  glEnable( GL_LIGHTING );
  rkglLightCreate( &scene->light, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0, 0, 0 );
  rkglLightMove( &scene->light, 10, 0, 4 );

  scene->gauge = rkglBeginList();
  rkglRGBByStr( "white" );
  glLineWidth( 0.5 );
  rkglGauge( zX, 6.0, zY, 6.0, 0.2 );
  glEndList();
}

void exitScene(Scene *scene)
{
  glDeleteLists( scene->gauge, 1 );
  rkglWindowCloseGLX( scene->canvas );
  rkglWindowDestroyGLX( scene->canvas );
}

void lookAt(Scene *scene, double cx, double cy, double cz, double fx, double fy, double fz)
{
  rkglCameraLookAt( &scene->cam, cx, cy, cz, fx, fy, fz, 0, 0, 1 );
}

void resizeScene(Scene *scene, int x, int y, int w, int h)
{
  double wx, wy;

  zxRegionSet( &scene->reg, x, y, w, h );
  wx = 0.1;
  XMoveWindow( zxdisplay, scene->canvas, scene->reg.x, scene->reg.y );
  XResizeWindow( zxdisplay, scene->canvas, scene->reg.width, scene->reg.height );
  rkglCameraSetViewport( &scene->cam, 0, 0, scene->reg.width, scene->reg.height );
  wy = wx / rkglCameraViewportAspectRatio( &scene->cam );
  rkglCameraSetViewvolumeXY( &scene->cam, -wx, wx, -wy, wy );
  rkglCameraSetViewvolumeZ( &scene->cam, 1, 20 );
  rkglCameraSetFrustum( &scene->cam );
  rkglCameraPutViewvolume( &scene->cam );
}

void _create_sphere(zVec3D *cen, double r, double g, double b)
{
  zOpticalInfo oi;
  zSphere3D sphere;

  zOpticalInfoCreateSimple( &oi, r, g, b, NULL );
  zSphere3DCreate( &sphere, cen, 0.02, 0 );
  rkglMaterial( &oi );
  rkglSphere( &sphere, RKGL_FACE );
}

void _create_cylinder(zVec3D *c1, zVec3D *c2)
{
  zOpticalInfo oi;
  zCyl3D cyl;

  zOpticalInfoCreateSimple( &oi, 0.5, 0.5, 0.5, NULL );
  zCyl3DCreate( &cyl, c1, c2, 0.005, 0 );
  rkglMaterial( &oi );
  rkglCyl( &cyl, RKGL_FACE );
}

void drawScene(Scene *scene, zVec3D *com, zVec3D *zmp)
{
  rkglWindowActivateGLX( scene->canvas );
  rkglClear();
  /* light */
  rkglLightPut( &scene->light );
  /* camera */
  rkglCameraPut( &scene->cam );
  /* gauge */
  glCallList( scene->gauge );
  /* stuff */
  _create_sphere( com, 0.8, 0, 0 );
  _create_sphere( zmp, 0, 0.8, 0 );
  _create_cylinder( com, zmp );
  /* buffering */
  rkglWindowSwapBuffersGLX( scene->canvas );
  rkglFlushGLX();
}

void polar(double *r, double *theta)
{
  *r = sqrt( zSqr(com.c.x) + zSqr(com.c.y) );
  *theta = atan2( com.c.y, com.c.x );
}

void measure(pdCZ *ctrl, double vx, double vy, double *du, double *vu, double *dw, double *vw)
{
  double r, rd;
  double theta, thetad;
  double s, c;

  polar( &r, &theta );
  rd = 1.0 / pdCZKappa( ctrl );
  thetad = theta;
  zSinCos( theta, &s, &c );
  *du = rd * ( thetad - theta );
  *vu = -s*vx + c*vy;
  *dw = rd - r;
  *vw = -c*vx - s*vy;
}

void world_zmp(double uz, double wz)
{
  double r, theta;
  double s, c;

  polar( &r, &theta );
  zSinCos( theta, &s, &c );
  zmp.c.x = com.c.x - uz*s - wz*c;
  zmp.c.y = com.c.y + uz*c - wz*s;
}

zVec dp(double t, zVec p, void *dummy, zVec v)
{
  double du, vu, dw, vw;
  pdCZ *ctrl;

  ctrl = (pdCZ *)dummy;
  com.c.x = zVecElem(p,0);
  com.c.y = zVecElem(p,2);
  measure( ctrl, zVecElem(p,1), zVecElem(p,3), &du, &vu, &dw, &vw );
  /* pdCZUpdate( ctrl, du, vu, dw, vw ); */
  /* world_zmp( pdCZZMPTan(ctrl), pdCZZMPRad(ctrl) ); */
  zVecElemNC(v,0) = zVecElem(p,1);
  zVecElemNC(v,1) = zSqr(pdCZZeta(ctrl)) * ( zVecElem(p,0) - zmp.c.x );
  zVecElemNC(v,2) = zVecElem(p,3);
  zVecElemNC(v,3) = zSqr(pdCZZeta(ctrl)) * ( zVecElem(p,2) - zmp.c.y );
  return v;
}

#define DT     0.01
void init_ctrl(pdCZ *ctrl)
{
  pdCZInit( ctrl, DT );
  /*                qu1, qu2, qw1, qw2, kappa, rho, kr */
  pdCZSetQ1U( ctrl, 1.0 );
  pdCZSetQ2U( ctrl, 1.0 );
  pdCZSetQ1W( ctrl, 1.0 );
  pdCZSetQ2W( ctrl, 1.5 );
  pdCZSetKappa( ctrl, 2.0 );
  pdCZSetRho( ctrl, 1.0 );
  pdCZSetKr( ctrl, 1.0 );
  /*                     zd */
  pdCZSetQ1Z( ctrl, 1.0 );
  pdCZSetQ2Z( ctrl, 0.5 );
  /*                      vud, vwd, dist */
  pdCZSetRefVelU( ctrl, 0.25 );
  pdCZSetRefVelW( ctrl, 0.0 );
  pdCZSetDist( ctrl, 0.1 );

  zVec3DCreate( &com, 0.6, 0, 0.26 );
  zVec3DZero( &vel );
  zVec3DZero( &acc );
  zVec3DCreate( &zmp, 0.6, 0, 0 );
  zVec3DCopy( &com, &comd );
  theta = thetad = -zPI_2;
  pdCZSetCmdCOMVec( ctrl, &comd );
  pdCZSetCmdTheta( ctrl, thetad );
}

void resize(zxWindow *win, Scene *sc)
{
  short w,h;

  zxWindowClear( win );
  w = zxWindowWidth( win );
  h = zxWindowHeight( win );
  resizeScene( sc, 0, 0, w, h );
}

void mainloop(zxWindow *win, Scene *sc, zODE *ode, pdCZ *ctrl)
{
  int count = 0;
  int t;
  zVec p;

  p = zVecCreateList( 4, com.c.x, 0, com.c.y, 0 );
  while( 1 ){
    switch( zxGetEvent() ){
    case KeyPress:
      switch( zxKeySymbol() ){
      case XK_q: return;
      }
      break;
    case ConfigureNotify:
      zxWindowUpdateRegion( win );
      resize( win, sc );
      break;
    default: ;
    }
    t = count*DT;
    resize( win, sc );
    /* zODEUpdate( ode, t, p, DT, ctrl ); */
    pdCZUpdate( ctrl, &com, &vel, &acc, &zmp, 0, ZVEC3DZERO, theta, NULL );
    lookAt( sc, 0.0, -5.5, 3, 0.0, 0.0, 0.0 );
    drawScene( sc, &com, &zmp );
    usleep( sec2usec(DT) );
    if( !zIsTiny( pdCZRefVelU( ctrl ) ) )
      pdCZAutoUpdateRef_old( ctrl, &comd, &thetad );
    zVec3DCopy( pdCZRefCOM(ctrl), &com );
    zVec3DCopy( pdCZRefVel(ctrl), &vel );
    zVec3DCopy( pdCZRefAcc(ctrl), &acc );
    zVec3DCopy( pdCZRefZMP(ctrl), &zmp );
    theta = thetad;
    count++;
  }
}



#define WIDTH  480
#define HEIGHT 480
int main(int argc, char *argv[])
{
  zxWindow win;
  zODE ode;
  Scene sc;
  pdCZ ctrl;

  rkglInitGLX();
  zxWindowCreate( &win, 0, 0, WIDTH, HEIGHT );
  zxWindowSetBGColorByName( &win, (char *)"lightgray" );
  zxWindowClear( &win );
  zxWindowKeyEnable( &win );
  zxWindowSetTitle( &win, (char *)"COM-ZMP control test" );
  zxWindowOpen( &win );

  /* zODEAssign( &ode, RKF45, NULL, NULL ); */
  /* zODEInit( &ode, 4, 0, dp ); */
  init_ctrl( &ctrl );
  initScene( &sc, &win );

  mainloop( &win, &sc, &ode, &ctrl );

  exitScene( &sc );
  pdCZDestroy( &ctrl );
  return 0;
}
