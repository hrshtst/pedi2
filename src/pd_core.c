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
  pdCoreInitMode( core );
  core->cmd = cmd;
}

void _pdCorePoseInit(pdCore *core)
{
  zVec3D v;
  double foot_dist, com_height;
  double s, c, d, x, y, theta;
  double offset;

  offset = zPI_2;
  foot_dist = zVec3DDist( &core->state.lf_pos, &core->state.rf_pos );
  com_height = zVec3DElem(&core->state.com_pos,zZ) - zVec3DElem(&core->state.lf_pos,zZ);
  pdCZSetDist( pdCoreCZPtr(core), foot_dist );
  theta = core->cmd->thetad;
  zSinCos( theta, &s, &c );
  d = 0.5 * foot_dist;
  x = zVec3DElem( &core->state.com_pos, zX );
  y = zVec3DElem( &core->state.com_pos, zY );
  core->cmd->xd = x;
  core->cmd->yd = y;

  zVec3DCopy( &core->state.com_pos, &v );
  zVec3DSetElem( &v, zZ, com_height*0.95 );
  pdRobotSetRefCOM( pdCoreRobotPtr(core), &v );

  zVec3DCopy( &core->state.base_att, &v );
  zVec3DSetElem( &v, zX, theta + offset );
  pdRobotSetRefBaseAtt( pdCoreRobotPtr(core), &v );

  zVec3DCreate( &v, x-d*c, y-d*s, 0 );
  pdRobotSetRefLFPos( pdCoreRobotPtr(core), &v );

  zVec3DCopy( &core->state.lf_att, &v );
  zVec3DSetElem( &v, zX, theta + offset );
  pdRobotSetRefLFAtt( pdCoreRobotPtr(core), &v );

  zVec3DCreate( &v, x+d*c, y+d*s, 0 );
  pdRobotSetRefRFPos( pdCoreRobotPtr(core), &v );

  zVec3DCopy( &core->state.rf_att, &v );
  zVec3DSetElem( &v, zX, theta + offset );
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

bool pdCoreDoesIntendToStand(pdCore *core)
{
  return zIsTiny( core->cmd->rho ) &&
         zIsTiny( core->cmd->vud ) &&
         zIsTiny( core->cmd->vwd );
}

bool pdCoreDoesIntendToStep(pdCore *core)
{
  return !zIsTiny( core->cmd->rho );
}

bool pdCoreDoesIntendToWalk(pdCore *core)
{
  return !zIsTiny( core->cmd->vud );
}

void pdCoreInitMode(pdCore *core)
{
  core->mode.stand = true;
  core->mode.step = false;
  core->mode.walk = false;
  core->mode.sidewalk = false;
}

void pdCoreUpdateMode(pdCore *core)
{
  if( pdCoreIsBothFeetOn( core ) ){
    if( pdCoreDoesIntendToStand( core ) || !core->mode.step ){
      core->mode.stand = true;
      core->mode.step = false;
    }
  }
  if( pdCoreIsEitherFootOff( core ) ){
    core->mode.stand = false;
    core->mode.step = true;
  }
}

#define pdCoreBool2Str(b) ( b ? "TRUE" : "FALSE" )
void pdCoreWriteMode(pdCore *core)
{
  printf( "stand:%s, step:%s, walk:%s, sidewalk:%s\n",
          pdCoreBool2Str(core->mode.stand),
          pdCoreBool2Str(core->mode.step),
          pdCoreBool2Str(core->mode.walk),
          pdCoreBool2Str(core->mode.sidewalk) );
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
  /* pdCZSetRefVelU( pdCoreCZPtr( core ), core->cmd->vud ); */
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
  double offset;

  offset = zPI_2;
  pdCZUpdate( pdCoreCZPtr(core),
              &core->state.com_pos,
              &core->state.com_vel,
              &core->state.com_acc,
              &core->state.zmp,
              core->state.base_att.e[0] - offset,
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
  double offset;

  offset = zPI_2;

  pdRobotSetRefCOM( pdCoreRobotPtr(core), pdCZRefCOM( pdCoreCZPtr(core) ) );

  zVec3DCreate( &v, pdCZCmdTheta( pdCoreCZPtr(core) ), 0, 0 );
  zVec3DElem( &v, zX ) += offset;
  pdRobotSetRefBaseAtt( pdCoreRobotPtr(core), &v );

  pdRobotSetRefLFPos( pdCoreRobotPtr(core), pdFootRefPos( pdCoreLFPtr(core) ) );

  zVec3DCopy( pdFootRefAtt( pdCoreLFPtr(core) ), &v );
  pdRobotSetRefLFAtt( pdCoreRobotPtr(core), pdFootRefAtt( pdCoreLFPtr(core) ) );

  pdRobotSetRefRFPos( pdCoreRobotPtr(core), pdFootRefPos( pdCoreRFPtr(core) ) );

  zVec3DCopy( pdFootRefAtt( pdCoreLFPtr(core) ), &v );
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

  pdCZSetRho( pdCoreCZPtr(core), core->cmd->rho );
  pdCZSetRefVelU( pdCoreCZPtr( core ), core->cmd->vud );
  if( pdCoreDoesIntendToWalk( core ) ){
    pdCZSetRho( pdCoreCZPtr(core), 1.0 );
    if( !core->mode.step )
      pdCZSetRefVelU( pdCoreCZPtr( core ), 0.0 );
  }

  if( !zIsTiny( pdCZRefVelU( pdCoreCZPtr(core) ) ) ){
    pdCZAutoUpdateRef( pdCoreCZPtr(core), &pd, &core->cmd->thetad );
    core->cmd->xd = pd.e[zX];
    core->cmd->yd = pd.e[zY];
  }
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
  pdCoreUpdateMode( core );
}

void pdCoreDataFWrite(FILE *fp, pdCore *core)
{
  pdCZ *c;
  pdFoot *lf, *rf;

  c = pdCoreCZPtr( core );
  lf = pdCoreLFPtr( core );
  rf = pdCoreRFPtr( core );
  fprintf( fp, "%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n",
/* 0- 1*/  pdCoreTime(core), pdCoreTimeStep(core),
/* 2- 5*/  pdCZCmdCOMX(c), pdCZCmdCOMY(c), pdCZCmdCOMZ(c), pdCZCmdTheta(c),
/* 6- 9*/  pdCZCOMX(c), pdCZCOMY(c), pdCZCOMZ(c), pdCZTheta(c),
/*10-12*/  pdCZVelX(c), pdCZVelY(c), pdCZVelZ(c),
/*13-15*/  pdCZAccX(c), pdCZAccY(c), pdCZAccZ(c),
/*16-18*/  pdCZZMPX(c), pdCZZMPY(c), pdCZZMPZ(c),
/*19-21*/  pdCZRefPosU(c), pdCZDeltaU(c), pdCZVelU(c),
/*22-24*/  pdCZRefPosW(c), pdCZDeltaW(c), pdCZVelW(c),
/*25-27*/  pdCZRefCOMX(c), pdCZRefCOMY(c), pdCZRefCOMZ(c),
/*28-30*/  pdCZRefVelX(c), pdCZRefVelY(c), pdCZRefVelZ(c),
/*31-33*/  pdCZRefAccX(c), pdCZRefAccY(c), pdCZRefAccZ(c),
/*34-36*/  pdCZRefZMPX(c), pdCZRefZMPY(c), pdCZRefZMPZ(c),
/*37-39*/  zVec3DElem(&lf->_p,zX), zVec3DElem(&lf->_p,zY), zVec3DElem(&lf->_p,zZ),
/*40-42*/  zVec3DElem(&lf->_pd,zX), zVec3DElem(&lf->_pd,zY), zVec3DElem(&lf->_pd,zZ),
/*43-45*/  zVec3DElem(&lf->refp,zX), zVec3DElem(&lf->refp,zY), zVec3DElem(&lf->refp,zZ),
/*46-48*/  zVec3DElem(&lf->_a,zX), zVec3DElem(&lf->_a,zY), zVec3DElem(&lf->_a,zZ),
/*49-51*/  zVec3DElem(&lf->_ad,zX), zVec3DElem(&lf->_ad,zY), zVec3DElem(&lf->_ad,zZ),
/*52-54*/  zVec3DElem(&lf->refa,zX), zVec3DElem(&lf->refa,zY), zVec3DElem(&lf->refa,zZ),
/*55-56*/  pdFootUWPhi(pdFootUWPtr(lf)), pdFootZMaxHeight(pdFootZPtr(lf)),
/*57-58*/  pdFootZZMPPhase(pdFootZPtr(lf))->re, pdFootZZMPPhase(pdFootZPtr(lf))->im,
/*59-60*/  pdFootZFootPhase(pdFootZPtr(lf)), pdFootZRefZ(pdFootZPtr(lf)),
/*61-63*/  zVec3DElem(&rf->_p,zX), zVec3DElem(&rf->_p,zY), zVec3DElem(&rf->_p,zZ),
/*64-66*/  zVec3DElem(&rf->_pd,zX), zVec3DElem(&rf->_pd,zY), zVec3DElem(&rf->_pd,zZ),
/*67-69*/  zVec3DElem(&rf->refp,zX), zVec3DElem(&rf->refp,zY), zVec3DElem(&rf->refp,zZ),
/*70-72*/  zVec3DElem(&rf->_a,zX), zVec3DElem(&rf->_a,zY), zVec3DElem(&rf->_a,zZ),
/*73-75*/  zVec3DElem(&rf->_ad,zX), zVec3DElem(&rf->_ad,zY), zVec3DElem(&rf->_ad,zZ),
/*76-78*/  zVec3DElem(&rf->refa,zX), zVec3DElem(&rf->refa,zY), zVec3DElem(&rf->refa,zZ),
/*79-80*/  pdFootUWPhi(pdFootUWPtr(rf)), pdFootZMaxHeight(pdFootZPtr(rf)),
/*81-82*/  pdFootZZMPPhase(pdFootZPtr(rf))->re, pdFootZZMPPhase(pdFootZPtr(rf))->im,
/*83-84*/  pdFootZFootPhase(pdFootZPtr(rf)), pdFootZRefZ(pdFootZPtr(rf))
           );
}
