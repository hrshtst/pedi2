#include <pedi2/pd_core.h>

static void _pdCoreInitState(pdCore *core);
static void _pdCoreInitFoot(pdCore *core);
static void _pdCoreInitODESolver(pdCore *core);
static zVec _pd_dp(double t, zVec p, void *dummy, zVec v);
static void _pdCoreUpdateRobot(pdCore *core);
static double _pdCoreCalcSoleWidth(zVec3DList *sr);

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

void _pdCoreUpdateRobot(pdCore *core)
{
  pdRobotSolveIK( &core->robot );
  pdRobotSupportRegion( &core->robot );
  pdRobotFootPos( &core->robot, &core->lf.p, &core->rf.p );
  pdRobotFootAtt( &core->robot, &core->lf.a, &core->rf.a );
}

double _pdCoreCalcSoleWidth(zVec3DList *sr)
{
  zVec3DListCell *vc;
  double ymin, ymax, y;

  ymin = ymax = zVec3DInnerProd( Z_UNITYVEC3D, zListTail(sr)->data );
  zListForEach( sr, vc ){
    y = zVec3DInnerProd( Z_UNITYVEC3D, vc->data );
    if( y < ymin ) ymin = y;
    if( y > ymax ) ymax = y;
  }
  return ymax - ymin;
}

void pdCoreLoad(pdCore *core, char *model_file, char *conf_file)
{
  pdRobotLoad( &core->robot, model_file, conf_file );
  _pdCoreUpdateRobot( core );
  /* sole width */
  core->lf.sole_w = _pdCoreCalcSoleWidth( &core->robot.sr_lf );
  core->rf.sole_w = _pdCoreCalcSoleWidth( &core->robot.sr_rf );
  /* feet distance */
  pdCZPrmRad(&core->cz)->dist = zVec3DElem(&core->lf.p,zY) - zVec3DElem(&core->rf.p,zY);
  /* COM height */
  core->cz.vrt.zd = zVec3DElem(&core->robot.d_com_pos,zZ) - zVec3DElem(&core->lf.p,zZ);
  core->cz.vrt.zd = 0.9 * core->cz.vrt.zd;
}

void pdCoreExit(pdCore *core)
{
  zODEDestroy( &core->solver.ode );
  pdCZDestroy( &core->cz );
  pdRobotExit( &core->robot );
}


void pdCoreSetState(pdCore *core, double *x, double *y)
{}

void pdCoreUpdate(pdCore *core)
{}


void pdCoreFWrite(pdCore *core, FILE *fp)
{}
