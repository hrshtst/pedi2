#include <pedi2/pd_core.h>

static void _pdCoreUpdateCommand(pdCore *core);
static void _pdCoreUpdateCZ(pdCore *core);
static void _pdCoreUpdateFoot(pdCore *core);
static void _pdCoreUpdateHand(pdCore *core);
static void _pdCoreUpdateRobot(pdCore *core);
static void _pdCoreUpdateRef(pdCore *core);
static void _pdCoreUpdateState(pdCore *core);

void pdCoreInit(pdCore *core, pdCmd *cmd, double dt)
{
  pdCoreResetTime( core );
  pdCoreSetTimeStep( core, dt );
  pdCZInit( pdCoreCZPtr( core ), pdCoreTimeStep( core ) );
  pdFootInit( pdCoreLFPtr( core ), pdCZHrzPtr( pdCoreCZPtr( core ) ),
              PD_FOOT_LEFT,  pdCoreTimeStep( core ) );
  pdFootInit( pdCoreRFPtr( core ), pdCZHrzPtr( pdCoreCZPtr( core ) ),
              PD_FOOT_RIGHT, pdCoreTimeStep( core ) );
  pdRobotInit( pdCoreRobotPtr( core ) );
  pdStateInit( pdCoreStatePtr( core ) );
  core->cmd = cmd;
}

void _pdCorePoseInit(pdCore *core)
{
  zVec3D v;
  double foot_dist, com_height;
  double s, c, d, x, y, theta;

  foot_dist = zVec3DDist( &core->state.lf_pos, &core->state.rf_pos );
  com_height = zVec3DElem(&core->state.com_pos,zZ) - zVec3DElem(&core->state.lf_pos,zZ);
  pdCZSetDist( pdCoreCZPtr(core), foot_dist );
  theta = core->cmd->thetad;
  zSinCos( theta, &s, &c );
  d = 0.5 * foot_dist;
  x = zVec3DElem( &core->state.com_pos, zX );
  y = zVec3DElem( &core->state.com_pos, zY );

  zVec3DCopy( &core->state.com_pos, &v );
  zVec3DSetElem( &v, zZ, com_height*0.95 );
  pdRobotSetRefCOM( pdCoreRobotPtr(core), &v );

  zVec3DCopy( &core->state.base_att, &v );
  zVec3DSetElem( &v, zX, theta );
  zVec3DCreate( &v, theta, 0, 0 );
  pdRobotSetRefBaseAtt( pdCoreRobotPtr(core), &v );

  zVec3DCreate( &v, x-d*c, y-d*s, 0 );
  pdRobotSetRefLFPos( pdCoreRobotPtr(core), &v );

  zVec3DCopy( &core->state.lf_att, &v );
  zVec3DSetElem( &v, zX, theta );
  zVec3DCreate( &v, theta, 0, 0 );
  pdRobotSetRefLFAtt( pdCoreRobotPtr(core), &v );

  zVec3DCreate( &v, x+d*c, y+d*s, 0 );
  pdRobotSetRefRFPos( pdCoreRobotPtr(core), &v );

  zVec3DCopy( &core->state.rf_att, &v );
  zVec3DSetElem( &v, zX, theta );
  zVec3DCreate( &v, theta, 0, 0 );
  pdRobotSetRefRFAtt( pdCoreRobotPtr(core), &v );

  pdRobotSolveIK( pdCoreRobotPtr( core ) );
}

bool pdCoreLoad(pdCore *core, char *filename)
{
  if( !pdRobotLoad( pdCoreRobotPtr( core ), filename ) )
    return false;
  _pdCoreUpdateState( core );
  _pdCorePoseInit( core );
  _pdCoreUpdateState( core );
  return true;
}

void pdCoreDestroy(pdCore *core)
{
  core->cmd = NULL;
  pdStateDestroy( pdCoreStatePtr( core ) );
  pdRobotDestroy( pdCoreRobotPtr( core ) );
  pdFootDestroy( pdCoreLFPtr( core ) );
  pdFootDestroy( pdCoreRFPtr( core ) );
  pdCZDestroy( pdCoreCZPtr( core ) );
  pdCoreSetTimeStep( core, 0.0 );
  pdCoreResetTime( core );
}

void _pdCoreUpdateCommand(pdCore *core)
{
  pdCZSetQ1U( pdCoreCZPtr( core ), core->cmd->qu1 );
  pdCZSetQ2U( pdCoreCZPtr( core ), core->cmd->qu2 );
  pdCZSetQ1W( pdCoreCZPtr( core ), core->cmd->qw1 );
  pdCZSetQ2W( pdCoreCZPtr( core ), core->cmd->qw2 );
  pdCZSetQ1Z( pdCoreCZPtr( core ), core->cmd->qz1 );
  pdCZSetQ2Z( pdCoreCZPtr( core ), core->cmd->qz2 );
  pdCZSetKappa( pdCoreCZPtr( core ), core->cmd->kappa );
  /* pdCZSetRho( pdCoreCZPtr( core ), core->cmd->rho ); */
  pdCZSetKr( pdCoreCZPtr( core ), core->cmd->kr );
  pdCZSetKappa( pdCoreCZPtr( core ), core->cmd->kappa );
  pdCZSetCmdCOMX( pdCoreCZPtr( core ), core->cmd->xd );
  pdCZSetCmdCOMY( pdCoreCZPtr( core ), core->cmd->yd );
  pdCZSetCmdCOMZ( pdCoreCZPtr( core ), core->cmd->zd );
  pdCZSetCmdTheta( pdCoreCZPtr( core ), core->cmd->thetad );
  pdCZSetRefVelU( pdCoreCZPtr( core ), core->cmd->vud );
  pdCZSetRefVelW( pdCoreCZPtr( core ), core->cmd->vwd );
  pdCZSetDist( pdCoreCZPtr( core ), core->cmd->dist );
  pdFootSetMaxHeight( pdCoreLFPtr( core ), core->cmd->lfh );
  pdFootSetTrXK( pdCoreLFPtr( core ), core->cmd->lfkx );
  pdFootSetTrXC( pdCoreLFPtr( core ), core->cmd->lfcx );
  pdFootSetTrYK( pdCoreLFPtr( core ), core->cmd->lfky );
  pdFootSetTrYC( pdCoreLFPtr( core ), core->cmd->lfcy );
  pdFootSetTrZK( pdCoreLFPtr( core ), core->cmd->lfkz );
  pdFootSetTrZC( pdCoreLFPtr( core ), core->cmd->lfcz );
  pdFootSetMaxHeight( pdCoreRFPtr( core ), core->cmd->rfh );
  pdFootSetTrXK( pdCoreRFPtr( core ), core->cmd->rfkx );
  pdFootSetTrXC( pdCoreRFPtr( core ), core->cmd->rfcx );
  pdFootSetTrYK( pdCoreRFPtr( core ), core->cmd->rfky );
  pdFootSetTrYC( pdCoreRFPtr( core ), core->cmd->rfcy );
  pdFootSetTrZK( pdCoreRFPtr( core ), core->cmd->rfkz );
  pdFootSetTrZC( pdCoreRFPtr( core ), core->cmd->rfcz );
}

void _pdCoreUpdateCZ(pdCore *core)
{
  pdCZUpdate( pdCoreCZPtr(core),
              &core->state.com_pos,
              &core->state.com_vel,
              &core->state.com_acc,
              &core->state.zmp,
              core->state.base_att.e[0],
              &core->state.sr );
}

void _pdCoreUpdateFoot(pdCore *core)
{
  pdFootUpdate( pdCoreLFPtr(core), pdCoreRFPtr(core),
                pdCZDelta( pdCoreCZPtr(core) ),
                pdCZVelUW( pdCoreCZPtr(core) ),
                &core->state.zmp,
                &core->state.lf_pos,
                &core->state.rf_pos,
                &core->state.lf_att,
                &core->state.rf_att,
                &core->state.sr_lf,
                &core->state.sr_rf );
}

void _pdCoreUpdateHand(pdCore *core)
{
  /* dummy */
}

void _pdCoreUpdateRobot(pdCore *core)
{
  zVec3D v;

  pdRobotSetRefCOM( pdCoreRobotPtr(core), pdCZRefCOM( pdCoreCZPtr(core) ) );
  zVec3DCreate( &v, pdCZCmdTheta( pdCoreCZPtr(core) ), 0, 0 );
  pdRobotSetRefBaseAtt( pdCoreRobotPtr(core), &v );
  pdRobotSetRefLFPos( pdCoreRobotPtr(core), pdFootRefPos( pdCoreLFPtr(core) ) );
  pdRobotSetRefLFAtt( pdCoreRobotPtr(core), pdFootRefAtt( pdCoreLFPtr(core) ) );
  pdRobotSetRefRFPos( pdCoreRobotPtr(core), pdFootRefPos( pdCoreRFPtr(core) ) );
  pdRobotSetRefRFAtt( pdCoreRobotPtr(core), pdFootRefAtt( pdCoreRFPtr(core) ) );
  /* pdRobotSetRefLHPos( pdCoreRobotPtr(core), pdHandRefPos( pdCoreLFPtr(core) ) ); */
  /* pdRobotSetRefLHAtt( pdCoreRobotPtr(core), pdHandRefAtt( pdCoreLFPtr(core) ) ); */
  /* pdRobotSetRefRHPos( pdCoreRobotPtr(core), pdHandRefPos( pdCoreRFPtr(core) ) ); */
  /* pdRobotSetRefRHAtt( pdCoreRobotPtr(core), pdHandRefAtt( pdCoreRFPtr(core) ) ); */
  pdRobotSolveIK( pdCoreRobotPtr( core ) );
}

void _pdCoreUpdateRef(pdCore *core)
{
  zVec3D pd;

  if( !zIsTiny( core->cmd->vud ) )
    pdCZSetRho( pdCoreCZPtr(core), 1.0 );
  else
    pdCZSetRho( pdCoreCZPtr(core), core->cmd->rho );

  if( !zIsTiny( pdCZRefVelU( pdCoreCZPtr(core) ) ) )
    pdCZAutoUpdateRef( pdCoreCZPtr(core), &pd, &core->cmd->thetad );
  core->cmd->xd = pd.e[zX];
  core->cmd->yd = pd.e[zY];
  /* core->cmd->zd = pd.e[zZ]; */
}

void _pdCoreUpdateState(pdCore *core)
{
  pdRobotCOMPos( pdCoreRobotPtr(core), &core->state.com_pos );
  zVec3DCopy( pdCZRefVel( pdCoreCZPtr(core) ), &core->state.com_vel );
  zVec3DCopy( pdCZRefAcc( pdCoreCZPtr(core) ), &core->state.com_acc );
  pdRobotBaseAtt( pdCoreRobotPtr(core), &core->state.base_att );
  pdRobotFootPos( pdCoreRobotPtr(core), &core->state.lf_pos, &core->state.rf_pos );
  pdRobotFootAtt( pdCoreRobotPtr(core), &core->state.lf_att, &core->state.rf_att );
  pdRobotHandPos( pdCoreRobotPtr(core), &core->state.lh_pos, &core->state.rh_pos );
  pdRobotHandAtt( pdCoreRobotPtr(core), &core->state.lh_att, &core->state.rh_att );
  zVec3DCopy( pdCZRefZMP( pdCoreCZPtr(core) ), &core->state.zmp );
  core->state.fz = pdCZVrtRF( &core->cz._vrt );
  pdRobotSupportRegion( pdCoreRobotPtr(core), &core->state.sr_lf, &core->state.sr_rf, &core->state.sr );
}

void pdCoreUpdate(pdCore *core)
{
  _pdCoreUpdateCommand( core );
  _pdCoreUpdateCZ( core );
  _pdCoreUpdateFoot( core );
  _pdCoreUpdateHand( core );
  _pdCoreUpdateRobot( core );
  _pdCoreUpdateRef( core );
  _pdCoreUpdateState( core );
}
