#include <unistd.h>
#include <zm/zm_ode.h>
#include <zeo/zeo_vec3d.h>
#include <zx11/zxutil.h>
#include <roki/glrk_glx.h>
#include <roki/glrk_camera.h>
#include <roki/glrk_optic.h>
#include <roki/glrk_shape.h>
#include <pedi2/pd_cz.h>
#include <liw/liw_time.h>

zVec3D com;
zVec3D zmp;

typedef struct{
  glrkCamera cam;
  glrkLight light;
  Window canvas;
  zxRegion reg;
  int gauge;
} Scene;

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

void polar(double *r, double *theta)
{
  *r = sqrt( zSqr(zVec3DElem(&com,zX)) + zSqr(zVec3DElem(&com,zY)) );
  *theta = atan2( zVec3DElem(&com,zY), zVec3DElem(&com,zX) );
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
  zVec3DElem(&zmp,zX) = zVec3DElem(&com,zX) - uz*s - wz*c;
  zVec3DElem(&zmp,zY) = zVec3DElem(&com,zY) + uz*c - wz*s;
}

zVec dp(double t, zVec p, void *dummy, zVec v)
{
  double du, vu, dw, vw;
  pdCZ *ctrl;

  ctrl = (pdCZ *)dummy;
  zVec3DElem(&com,zX) = zVecElem(p,0);
  zVec3DElem(&com,zY) = zVecElem(p,2);
  measure( ctrl, zVecElem(p,1), zVecElem(p,3), &du, &vu, &dw, &vw );
  pdCZUpdate( ctrl, du, vu, dw, vw );
  world_zmp( pdCZZMPTan(ctrl), pdCZZMPRad(ctrl) );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCZZeta(ctrl)) * ( zVecElem(p,0) - zVec3DElem(&zmp,zX) );
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCZZeta(ctrl)) * ( zVecElem(p,2) - zVec3DElem(&zmp,zY) );
  return v;
}

void init_ctrl(pdCZ *ctrl)
{
  pdCZInit( ctrl );
  /*                qu1, qu2, qw1, qw2, kappa, rho, kr */
  pdCZSetPrm( ctrl, 1,   0,   1, 1.5,     2,   1,  1 );
  /*                     zd */
  pdCZSetRefVrt( ctrl, 0.26 );
  /*                      vud, vwd, dist */
  pdCZSetRefHrz( ctrl, 0.25, 0.0, 0.1 );
  zVec3DCreate( &com, 0.6, 0, 0.26 );
  zVec3DCreate( &zmp, 0.6, 0, 0 );
}

void resize(zxWindow *win, Scene *sc)
{
  short w,h;

  zxWindowClear( win );
  w = zxWindowWidth( win );
  h = zxWindowHeight( win );
  resizeScene( sc, 0, 0, w, h );
}

#define DT     0.01
void mainloop(zxWindow *win, Scene *sc, zODE *ode, pdCZ *ctrl)
{
  int count = 0;
  int t;
  zVec p;

  p = zVecCreateList( 4, zVec3DElem(&com,zX), 0, zVec3DElem(&com,zY), 0 );
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
    zODEUpdate( ode, t, p, DT, ctrl );
    lookAt( sc, 0.0, -5.5, 3, 0.0, 0.0, 0.0 );
    drawScene( sc, &com, &zmp );
    usleep( sec2usec(DT) );
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

  glrkInitGLX();
  zxWindowCreate( &win, 0, 0, WIDTH, HEIGHT );
  zxWindowSetBG( &win, (char *)"lightgray" );
  zxWindowClear( &win );
  zxKeyEnable( &win );
  zxWindowSetTitle( &win, (char *)"ctrl view test" );
  zxWindowOpen( &win );

  zODEAssign( &ode, RKF45, NULL, NULL );
  zODEInit( &ode, 4, 0, dp );
  init_ctrl( &ctrl );
  initScene( &sc, &win );

  mainloop( &win, &sc, &ode, &ctrl );

  exitScene( &sc );
  pdCZDestroy( &ctrl );
  return 0;
}
