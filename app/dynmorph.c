#include <zx11/zxwidget.h>
#include <zx11/zximage_dib.h>
#include <roki/glrk_glx.h>
#include <roki/glrk_camera.h>
#include <roki/glrk_optic.h>
#include <roki/glrk_shape.h>
#include <pedi2/pd_ctrl.h>

typedef struct{
  double x[2], y[2];
  pdCtrl c;
  double xz, yz;
} dmSystem;

void dmSystemInit(dmSystem *sys)
{
  pdCtrlInit( &sys->c );
  sys->x[0] = sys->x[1] = 0;
  sys->y[0] = sys->y[1] = 0;
  sys->xz = sys->x[0];
  sys->yz = sys->y[0];
}

void dmSystemExit(dmSystem *sys)
{
  pdCtrlDestroy( &sys->c );
}

#define DM_CONSOLE_WIDTH 240
void resize(zxWindow *win)
{
  short w, h;

  zxWindowClear( win );
  w = ( zxWindowWidth(win) - DM_CONSOLE_WIDTH ) / 2;
  h = zxWindowHeight(win) / 2;

  zxwSepBoxLower( win, 6, 6, w-4, 2*h-12 );
  zxwSepBoxLower( win, w+6, 6, w-4, 2*h-12 );

  zxDequeueEvent(); /* flush unprocessed events */
}

void capture(zxWindow *win)
{
  static int count = 0;
  static char str[BUFSIZ];
  zxImage img;

  sprintf( str, "dynmorph%05d.bmp", count++ );
  zxImageFromPixmap( &img, zxCanvas(win), zxWindowWidth(win), zxWindowHeight(win) );
  zxImageWriteBMPFile( &img, str );
  zxImageDestroy( &img );
}

typedef struct{
  bool pause;
  bool frame;
  bool perturb_x;
  bool perturb_y;
  bool rec;
  bool log;
} dmFlagset;

void dmFlagsetInit(dmFlagset *flag)
{
  flag->pause = false;
  flag->frame = false;
  flag->perturb_x = false;
  flag->perturb_y = false;
  flag->rec = false;
  flag->log = false;
}

void frame_one(zxWindow *win, dmSystem *sys, dmFlagset *flag)
{
}

#define ANIM_SKIP 1000
void mainloop(zxWindow *win, dmSystem *sys)
{
  dmFlagset flag;
  int count = ANIM_SKIP;

  dmFlagsetInit( &flag );
  while( 1 ){
    switch( zxGetEvent() ){
    case Expose:
    case ConfigureNotify:
      zxWindowUpdateRegion( win );
      resize( win );
      break;
    case ClientMessage:
      if( zxDeleteWindowEvent() ) return;
      break;
    case KeyPress:
      switch( zxKeySymbol() ){
      case XK_p: flag.pause = 1 - flag.pause; break;
      case XK_f: flag.frame = flag.pause; break;
      case XK_r: flag.rec = 1 - flag.rec; break;
      case XK_q: return;
      }
      break;
    default: ;
    }
    if( ++count > ANIM_SKIP ){
      frame_one( win, sys, &flag );
      count = 0;
      if( flag.rec ) capture( win );
    }
  }
}

#define WIDTH 960
#define HEIGHT 640
int main(int argc, char *argv[])
{
  zxWindow mainwin;
  dmSystem sys;

  glrkInitGLX();
  zxWindowCreate( &mainwin, 0, 0, WIDTH, HEIGHT );
  zxWindowSetBG( &mainwin, (char *)"lightgray" );
  zxWindowClear( &mainwin );
  zxKeyEnable( &mainwin );
  zxMouseEnable( &mainwin );
  zxWindowSetTitle( &mainwin, (char *)"dynamics morphing" );
  zxWindowOpen( &mainwin );
  zxWidgetInit( &mainwin );

  dmSystemInit( &sys );
  mainloop( &mainwin, &sys );
  dmSystemExit( &sys );

  glrkCloseGLX();
  return 0;
}
