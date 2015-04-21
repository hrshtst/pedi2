#ifndef DM_CONSOLE_H
#define DM_CONSOLE_H

#include <zx11/zxwidget.h>

/* dmConsoleEval */

typedef struct{
  zxwLabel label;
  zxwEditBox eb;
  zxwGauge gauge;
  double *valp;
} dmConsoleEval;

void dmConsoleEvalInit(dmConsoleEval *ce, const char str[], double min, double max, double val, short ndiv, double *valp);
void dmConsoleEvalExit(dmConsoleEval *ce);
void dmConsoleEvalMove(dmConsoleEval *ce, int x, int y);
void dmConsoleEvalDraw(dmConsoleEval *ce, zxWindow *win);

void dmConsoleEvalButtonPress(dmConsoleEval *ce);
void dmConsoleEvalButtonRelease(dmConsoleEval *ce);
bool dmConsoleEvalMouseMove(dmConsoleEval *ce, zxWindow *win);
void dmConsoleEvalForceMove(dmConsoleEval *ce, zxWindow *win, double val);

/* dmConsole */

typedef struct{
  int num;
  dmConsoleEval *eval;
  zxRegion reg;
} dmConsole;

#define DM_CONSOLE_WIDTH      240
#define DM_CONSOLE_LABELLEN   136
#define DM_CONSOLE_EDITLEN     80
#define DM_CONSOLE_GAUGELEN   224
#define DM_CONSOLE_HEIGHT      40
#define DM_CONSOLE_LABELHEIGHT 24

void dmConsoleInit(dmConsole *console);
bool dmConsoleAddEval(dmConsole *console, const char str[], double min, double max, double val, short ndiv, double *valp);
void dmConsoleExit(dmConsole *console);

void dmConsoleMove(dmConsole *console, int x, int y);
void dmConsoleDraw(dmConsole *console, zxWindow *win);

void dmConsoleButtonPress(dmConsole *console);
void dmConsoleButtonRelease(dmConsole *console);
bool dmConsoleMouseMove(dmConsole *console, zxWindow *win);

#endif /* DM_CONSOLE_H */
