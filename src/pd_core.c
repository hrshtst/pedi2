#include <pedi2/pd_core.h>

void pdCommandInit(pdCommand *com)
{
  com->qu1 = com->qu2 = 0;
  com->qw1 = com->qw2 = 0;
  com->kappa = 0;
  com->rho = 0;
  com->kr = 0;
  com->zd = 0;
  com->vud = com->vwd = 0;
  com->dist = 0;
  com->lfh = com->rfh = 0;
}

void pdCommandDefaultInit(pdCommand *com)
{
  pdCommandInit( com );
  com->qu1 = 1.0;
  com->qu2 = 0.0;
  com->qw1 = 1.0;
  com->qw2 = 1.5;
  com->rho = 0.0;
  com->kr = 1.0;
}

void pdCommandExit(pdCommand *com)
{
  pdCommandInit( com );
}

static void _pdCoreInitState(pdCore *core);
static void _pdCoreInitFoot(pdCore *core);
static void _pdCoreInitODESolver(pdCore *core);
static zVec _pd_dp(double t, zVec p, void *dummy, zVec v);
static double _pdCoreCalcSoleWidth(zVec3DList *sr);
static void _pdCoreCoodTransWtoM(pdCore *core, double *du, double *vu, double *dw, double *vw);
static void _pdCoreCoodTransMtoW(pdCore *core, double u, double w, double *x, double *y);
static void _pdCoreUpdateCommand(pdCore *core);
static void _pdCoreUpdateCZ(pdCore *core, double dt);
static void _pdCoreUpdateFoot(pdCore *core, double dt);
static void _pdCoreUpdateRobot(pdCore *core);
static void _pdCoreUpdateRefPosTheta(pdCore *core);
static void _pdCoreUpdateRef(pdCore *core);

void _pdCoreLoad(pdCore *core, const char* model_file, const char* conf_file);
void _pdCorePoseInit(pdCore *core);

void _pdCoreWritePosAtt(pdCore *core)
{
  /* for debug */
  zVec3D com_p, body_a;
  zVec3D lf_p, lf_a;
  zVec3D rf_p, rf_a;

  pdRobotCOMPos( &core->robot, &com_p );
  pdRobotBodyAtt( &core->robot, &body_a );
  pdRobotFootPos( &core->robot, &lf_p, &rf_p );
  pdRobotFootAtt( &core->robot, &lf_a, &rf_a );
  printf( "com_pos:" );zVec3DWrite( &com_p );
  printf( "lf_pos :");zVec3DWrite( &lf_p );
  printf( "rf_pos :");zVec3DWrite( &rf_p );
  printf( "bod_att:" );zVec3DWrite( &body_a );
  printf( "lf_att :");zVec3DWrite( &lf_a );
  printf( "rf_att :");zVec3DWrite( &rf_a );
}

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
  double du, vu, dw, vw;
  pdCore *core;

  core = (pdCore *)dummy;
  _pdCoreCoodTransWtoM( core, &du, &vu, &dw, &vw );
  pdCZUpdate( &core->cz, du, vu, dw, vw );
  _pdCoreCoodTransMtoW( core, pdCZZMPTan(&core->cz), pdCZZMPRad(&core->cz), &core->xz, &core->yz );
  zVecElem(v,0) = zVecElem(p,1);
  zVecElem(v,1) = zSqr(pdCZZeta(&core->cz)) * ( zVecElem(p,0) - core->xz ) + core->adx;
  zVecElem(v,2) = zVecElem(p,3);
  zVecElem(v,3) = zSqr(pdCZZeta(&core->cz)) * ( zVecElem(p,2) - core->yz ) + core->ady;
  return v;
}

void _pdCoreInitODESolver(pdCore *core)
{
  core->solver.t = 0;
  core->solver.p = zVecCreateList( 4, core->x[0], core->x[1], core->y[0], core->y[1] );
  zODEAssign( &core->solver.ode, RKF45, NULL, NULL );
  zODEInit( &core->solver.ode, 4, 0, _pd_dp );
}

void pdCoreInit(pdCore *core, pdCommand *com)
{
  pdRobotInit( &core->robot );
  pdCZInit( &core->cz );
  _pdCoreInitState( core );
  _pdCoreInitFoot( core );
  _pdCoreInitODESolver( core );
  core->com = com;
}

double _pdCoreCalcSoleWidth(zVec3DList *sr)
{
  zVec3DListCell *vc;
  double ymin, ymax, y;

  ymin = ymax = zVec3DInnerProd( ZVEC3DY, zListTail(sr)->data );
  zListForEach( sr, vc ){
    y = zVec3DInnerProd( ZVEC3DY, vc->data );
    if( y < ymin ) ymin = y;
    if( y > ymax ) ymax = y;
  }
  return ymax - ymin;
}

void _pdCoreLoad(pdCore *core, const char* model_file, const char* conf_file)
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

void _pdCorePoseInit(pdCore *core)
{
  double d;
  double s, c;

  d = 0.5 * pdCZPrmRad(&core->cz)->dist;
  zSinCos( core->theta, &s, &c );
  /* feet */
  zVec3DCreate( &core->robot.d_lf_pos, core->xd-d*c, core->yd-d*s, 0 );
  zVec3DCreate( &core->robot.d_rf_pos, core->xd+d*c, core->yd+d*s, 0 );
  zVec3DCreate( &core->robot.d_lf_att, core->theta+zPI_2, 0, 0 );
  zVec3DCreate( &core->robot.d_rf_att, core->theta+zPI_2, 0, 0 );
  zVec3DCopy( &core->robot.d_lf_pos, &core->lf.pd );
  zVec3DCopy( &core->robot.d_lf_pos, &core->lf.ps );
  zVec3DCopy( &core->robot.d_rf_pos, &core->rf.pd );
  zVec3DCopy( &core->robot.d_rf_pos, &core->rf.ps );
  zVec3DCopy( &core->robot.d_lf_att, &core->lf.as );
  zVec3DCopy( &core->robot.d_rf_att, &core->rf.as );
  /* body */
  zVec3DCreate( &core->robot.d_com_pos, core->xd, core->yd, core->cz.vrt.zd );
  zVec3DCreate( &core->robot.d_body_att, core->theta+zPI_2, 0, 0 );
  /* update */
  _pdCoreUpdateRobot( core );
  /* spring-damper tracking */
  core->lf.track_old[0] = zVec3DElem( &core->lf.p, zX );
  core->lf.track_old[1] = zVec3DElem( &core->lf.p, zY );
  core->lf.track_old[2] = zVec3DElem( &core->lf.p, zZ );
  core->rf.track_old[0] = zVec3DElem( &core->rf.p, zX );
  core->rf.track_old[1] = zVec3DElem( &core->rf.p, zY );
  core->rf.track_old[2] = zVec3DElem( &core->rf.p, zZ );
}

void pdCoreLoad(pdCore *core, char *model_file, char *conf_file)
{
  _pdCoreLoad( core, model_file, conf_file );
  _pdCorePoseInit( core );
}

void pdCoreExit(pdCore *core)
{
  zODEDestroy( &core->solver.ode );
  pdCZDestroy( &core->cz );
  pdRobotExit( &core->robot );
}

void pdCoreSetState(pdCore *core, double *x, double *y, double theta)
{
  core->x[0] = x[0];
  core->x[1] = x[1];
  core->y[0] = y[0];
  core->y[1] = y[1];
  core->theta = theta;
}

void _pdCoreCoodTransWtoM(pdCore *core, double *du, double *vu, double *dw, double *vw)
{
  double s, c;

  zSinCos( core->theta, &s, &c );
  *du = 0;
  *vu = -core->x[1]*s + core->y[1]*c;
  *dw = ( core->xd - core->x[0] )*c + ( core->yd - core->y[0] )*s;
  *vw = -core->x[1]*c - core->y[1]*s;
}

void _pdCoreCoodTransMtoW(pdCore *core, double u, double w, double *x, double *y)
{
  double s, c;

  zSinCos( core->theta, &s, &c );
  *x = core->x[0] - u*s - w*c;
  *y = core->y[0] + u*c - w*s;
}

void _pdCoreUpdateCommand(pdCore *core)
{
  /* pdCZSetPrm( &core->cz, core->com->qu1, core->com->qu2, core->com->qw1, core->com->qw2, core->com->kappa, core->com->rho, core->com->kr ); */
  pdCZPrmTan(&core->cz)->q1 = core->com->qu1;
  pdCZPrmTan(&core->cz)->q2 = core->com->qu2;
  pdCZPrmTan(&core->cz)->kappa = core->com->kappa;
  pdCZPrmRad(&core->cz)->q1 = core->com->qw1;
  pdCZPrmRad(&core->cz)->q2 = core->com->qw2;
  pdCZPrmRad(&core->cz)->kappa = core->com->kappa;
  /* pdCZPrmRad(&core->cz)->rho = core->com->rho; */
  pdCZPrmRad(&core->cz)->kr = core->com->kr;
  pdCZSetRefVrt( &core->cz, core->com->zd );
  pdCZSetRefHrz( &core->cz, core->com->vud, core->com->vwd, core->com->dist );
  core->lf.h   = core->com->lfh;
  core->rf.h   = core->com->rfh;
}

void _pdCoreUpdateCZ(pdCore *core, double dt)
{
  core->solver.t += dt;
  core->x[0] = core->nx[0];
  core->x[1] = core->nx[1];
  core->y[0] = core->ny[0];
  core->y[1] = core->ny[1];
  core->xd = core->nxd;
  core->yd = core->nyd;
  core->theta = core->ntheta;
  zODEUpdate( &core->solver.ode, core->solver.t, core->solver.p, dt, core );
  core->nx[0] = zVecElem(core->solver.p,0);
  core->nx[1] = zVecElem(core->solver.p,1);
  core->ny[0] = zVecElem(core->solver.p,2);
  core->ny[1] = zVecElem(core->solver.p,3);
}

void _pdCoreUpdateFoot(pdCore *core, double dt)
{
  double du, vu, dw, vw;
  double s, c;
  zComplex pz;

  zSinCos( core->theta, &s, &c );
  pdRobotFootPos( &core->robot, &core->lf.p, &core->rf.p );
  pdRobotFootAtt( &core->robot, &core->lf.a, &core->rf.a );
  _pdCoreCoodTransWtoM( core, &du, &vu, &dw, &vw );
  pdCZZMPPhase( &core->cz, dw, vw, &pz );
  pdFootLift( &core->cz, &core->lf, &core->rf, core->xd, core->yd, core->theta, &pz );
  pdFootMove( &core->cz, &core->lf, &core->rf, core->xd, core->yd, core->theta );
  pdFootUpdate( &core->lf, &core->rf, dt );

  /* update IK constraints for feet */
  zVec3DCopy( &core->lf.ps, &core->robot.d_lf_pos );
  zVec3DCopy( &core->rf.ps, &core->robot.d_rf_pos );
  zVec3DCopy( &core->lf.as, &core->robot.d_lf_att);
  zVec3DCopy( &core->rf.as, &core->robot.d_rf_att);
}

void _pdCoreUpdateRobot(pdCore *core)
{
  /* printf( "--\n" ); */
  /* _pdCoreWritePosAtt( core ); */
  pdRobotSolveIK( &core->robot );
  pdRobotSupportRegion( &core->robot );
  pdRobotFootPos( &core->robot, &core->lf.p, &core->rf.p );
  pdRobotFootAtt( &core->robot, &core->lf.a, &core->rf.a );
  /* _pdCoreWritePosAtt( core ); */
}

void _pdCoreUpdateRefPosTheta(pdCore *core)
{
  double kappa;
  double dw, ndw;
  double s, c;
  double cos_d;
  double kx, ky, kw;

  kappa = pdCZKappa(&core->cz);
  zSinCos( core->theta, &s, &c );
  dw = -( core->xd - core->x[0] )*c - ( core->yd - core->y[0] )*s;

  kx = kappa * ( core->nx[0] - core->x[0] );
  ky = kappa * ( core->ny[0] - core->y[0] );
  cos_d = ( kx*c + ky*s + 1 ) / sqrt( zSqr(kx+c) + zSqr(ky+s) );
  ndw = ( dw + (core->nx[0]-core->x[0])*c + (core->ny[0]-core->y[0])*s ) / cos_d;
  if( !zIsTiny(kappa) )
    ndw += ( 1 - cos_d ) / ( kappa * cos_d );
  kw = 1.0 + kappa * ndw;
  core->nxd = ( kappa*ndw*core->xd + core->nx[0] - ndw*c )/kw;
  core->nyd = ( kappa*ndw*core->yd + core->ny[0] - ndw*s )/kw;
  core->ntheta = atan2( ( kappa*(core->ny[0]-core->yd) + s )/kw,
                      ( kappa*(core->nx[0]-core->xd) + c )/kw );
}

void _pdCoreUpdateRef(pdCore *core)
{
  /* automatic activation when walking */
  if( !zIsTiny( core->com->vud ) ){
    pdCZPrmRad(&core->cz)->rho = 1.0;
  } else {
    pdCZPrmRad(&core->cz)->rho = core->com->rho;
  }

  /* automatic update of referential position and orientation */
  _pdCoreUpdateRefPosTheta( core );

  /* update IK constraints for COM */
  zVec3DCreate( &core->robot.d_com_pos, core->nx[0], core->ny[0], core->cz.vrt.zd );
  zVec3DCreate( &core->robot.d_body_att, core->theta+zPI_2, 0, 0 );
}

void pdCoreUpdate(pdCore *core, double dt)
{
  _pdCoreUpdateCommand( core );
  _pdCoreUpdateCZ( core, dt );
  _pdCoreUpdateFoot( core, dt );
  _pdCoreUpdateRobot( core );
  _pdCoreUpdateRef( core );
}


void pdCoreFWrite(pdCore *core, FILE *fp)
{
  fprintf( fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f\n",
           core->x[0], core->x[1], core->nx[0], core->nx[1],
           core->y[0], core->y[1], core->ny[0], core->ny[1],
           core->xz, core->xd, core->yz, core->yd,
           core->theta );
}
