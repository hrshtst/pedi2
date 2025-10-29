#include "dm_console.h"

/* dmConsoleEval */

void dmConsoleEvalInit(dmConsoleEval *ce, const char str[], double min, double max, double val, short ndiv, double *valp)
{
  zxwLabelInit( &ce->label );
  zxwLabelSet( &ce->label, str );
  zxwBoxSetRegion( &ce->label, 0, 0, DM_CONSOLE_LABELLEN, zxwLabelHeight(NULL,NULL) );
  zxwAlignLeft( &ce->label );
  zxwAlignMiddle( &ce->label );
  zxwEditBoxCreate( &ce->eb, BUFSIZ, 0, 0, DM_CONSOLE_EDITLEN, 0 );
  zxwAlignRight( &ce->eb );
  zxwAlignFlowRight( &ce->eb );
  zxwAlignMiddle( &ce->label );
  zxwGaugeCreate( &ce->gauge, 0, 0, DM_CONSOLE_GAUGELEN, ndiv, min, max );
  zxwGaugeSetValue( &ce->gauge, val );
  *( ce->valp = valp ) = val;
}

void dmConsoleEvalExit(dmConsoleEval *ce)
{
  zxwLabelDestroy( &ce->label );
  zxwEditBoxDestroy( &ce->eb );
}

void dmConsoleEvalMove(dmConsoleEval *ce, int x, int y)
{
  zxwBoxSetPos( &ce->label, x, y );
  zxwBoxSetPos( &ce->eb, x+ce->gauge.reg.width-ce->eb.reg.width, y );
  zxwGaugeMove( &ce->gauge, x, y+DM_CONSOLE_LABELHEIGHT );
  *ce->valp = zxwGaugeValue( &ce->gauge );
}

void dmConsoleEvalDraw(dmConsoleEval *ce, zxWindow *win)
{
  static char buf[BUFSIZ];

  zxwLabelDraw( win, &ce->label );
  zxwGaugeDraw( win, &ce->gauge );
  sprintf( buf, "%f", *ce->valp );
  zxwEditBoxSetString( &ce->eb, buf );
  zxwEditBoxDraw( win, &ce->eb );
}

void dmConsoleEvalButtonPress(dmConsoleEval *ce)
{
  zxwKnobTryGrab( &ce->gauge, zxMouseX, zxMouseY );
}

void dmConsoleEvalButtonRelease(dmConsoleEval *ce)
{
  zxwUngrab( &ce->gauge );
}

bool dmConsoleEvalMouseMove(dmConsoleEval *ce, zxWindow *win)
{
  if( !zxwIsGrabbed( &ce->gauge ) ) return false;
  zxwGaugeSetX( &ce->gauge, zxMouseX );
  *ce->valp = zxwGaugeValue( &ce->gauge );
  dmConsoleEvalDraw( ce, win );
  return true;
}

void dmConsoleEvalForceMove(dmConsoleEval *ce, zxWindow *win, double val)
{
  zxwGaugeSetValue( &ce->gauge, val );
  *ce->valp = zxwGaugeValue( &ce->gauge );
  dmConsoleEvalDraw( ce, win );
}


/* dmConsole */

void dmConsoleInit(dmConsole *console)
{
  console->num = 0;
  console->eval = NULL;
  zxRegionSet( &console->reg, 0, 0, DM_CONSOLE_WIDTH, 16 );
}

bool dmConsoleAddEval(dmConsole *console, const char str[], double min, double max, double val, short ndiv, double *valp)
{
  dmConsoleEval *el;

  if( !( el = zRealloc( console->eval, dmConsoleEval, console->num+1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  console->eval = el;
  console->reg.height += DM_CONSOLE_HEIGHT;
  dmConsoleEvalInit( &console->eval[console->num], str, min, max, val, ndiv, valp );
  console->num++;
  return true;
}

void dmConsoleExit(dmConsole *console)
{
  int i;

  for( i=0; i<console->num; i++ )
    dmConsoleEvalExit( &console->eval[i] );
  console->num = 0;
  zFree( console->eval );
}

void dmConsoleMove(dmConsole *console, int x, int y)
{
  int i;

  console->reg.x = x;
  console->reg.y = y;
  for( i=0; i<console->num; i++ )
    dmConsoleEvalMove( &console->eval[i],
      console->reg.x+8, console->reg.y+8+i*DM_CONSOLE_HEIGHT );
}

void dmConsoleDraw(dmConsole *console, zxWindow *win)
{
  int i;

  for( i=0; i<console->num; i++ )
    dmConsoleEvalDraw( &console->eval[i], win );
}

void dmConsoleButtonPress(dmConsole *console)
{
  int i;

  for( i=0; i<console->num; i++ )
    dmConsoleEvalButtonPress( &console->eval[i] );
}

void dmConsoleButtonRelease(dmConsole *console)
{
  int i;

  for( i=0; i<console->num; i++ )
    dmConsoleEvalButtonRelease( &console->eval[i] );
}

bool dmConsoleMouseMove(dmConsole *console, zxWindow *win)
{
  int i;

  for( i=0; i<console->num; i++ )
    if( dmConsoleEvalMouseMove( &console->eval[i], win ) ) return true;
  return false;
}
