#include <pedi2/pd_core.h>

static void _pdCoreInitState(pdCore *core);
static void _pdCoreInitFoot(pdCore *core);
static void _pdCoreInitODESolver(pdCore *core);
static zVec _pd_dp(double t, zVec p, void *dummy, zVec v);

void _pdCoreInitState(pdCore *core)
{
  core->x[0] = 0;
  core->y[0] = 0;
  core->x[1] = 0.001;
  core->y[1] = 0;
  core->xz = core->x[0];
  core->yz = core->y[0];
  core->xd = core->nxd = core->x[0];
  core->yd = core->nyd = core->y[0];
  core->nx[0] = core->x[0];
  core->nx[1] = core->x[1];
  core->ny[0] = core->y[0];
  core->ny[1] = core->y[1];
  core->theta = core->ntheta = 0;
  core->adx = core->ady = 0;
}

void _pdCoreInitFoot(pdCore *core)
{
  /* spring-damper tracking */
  core->lf.track_k[0] = 3000;
  core->lf.track_c[0] = 50;
  core->lf.track_k[1] = 3000;
  core->lf.track_c[1] = 50;
  core->lf.track_k[2] = 3000;
  core->lf.track_c[2] = 50;
  core->rf.track_k[0] = 3000;
  core->rf.track_c[0] = 50;
  core->rf.track_k[1] = 3000;
  core->rf.track_c[1] = 50;
  core->rf.track_k[2] = 3000;
  core->rf.track_c[2] = 50;
  /* sole width */
  core->lf.sole_w = 0;
  core->rf.sole_w = 0;
  /* direction */
  core->lf.dy = 1;
  core->rf.dy = -1;
}

zVec _pd_dp(double t, zVec p, void *dummy, zVec v)
{
  return v;
}

void _pdCoreInitODESolver(pdCore *core)
{
  core->solver.t = 0;
  core->solver.p = zVecCreateList( 4, core->x[0], core->x[1], core->y[0], core->y[1] );
  zODEAssign( &core->solver.ode, RKF45, NULL, NULL );
  zODEInit( &core->solver.ode, 4, 0, _pd_dp );
}

void pdCoreInit(pdCore *core)
{
  pdRobotInit( &core->robot );
  pdCZInit( &core->cz );
  _pdCoreInitState( core );
  _pdCoreInitFoot( core );
  _pdCoreInitODESolver( core );
}

void pdCoreLoad(pdCore *core, char *model_file, char *conf_file)
{
}

void pdCoreExit(pdCore *core)
{
  zODEDestroy( &core->solver.ode );
  pdCZDestroy( &core->cz );
  pdRobotExit( &core->robot );
}


void pdCoreSetCommand(pdCore *core, pdCommand *com)
{}

void pdCoreSetState(pdCore *core, double *x, double *y)
{}

void pdCoreUpdate(pdCore *core)
{}


void pdCoreFWrite(pdCore *core, FILE *fp)
{}
