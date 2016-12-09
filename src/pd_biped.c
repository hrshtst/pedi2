#include <pedi2/pd_biped.h>

static bool _pdBipedIsStateValid(pdBiped *biped, pdState *state);
static void _pdBipedPoseInit(pdBiped *biped, pdState *state);
static void _pdBipedUpdateCommand(pdBiped *biped);
static void _pdBipedUpdateCZ(pdBiped *biped, pdState *state);
static void _pdBipedUpdateFoot(pdBiped *biped, pdState *state);
static void _pdBipedModifyCommand(pdBiped *biped, pdState *state);

void pdBipedInit(pdBiped *biped, pdCmd *cmd, double dt)
{
  pdBipedResetTime( biped );
  pdBipedSetTimeStep( biped, dt );
  pdCZInit( pdBipedCZPtr( biped ), pdBipedTimeStep( biped ) );
  pdFootInit( pdBipedLFPtr( biped ), pdCZHrzPtr( pdBipedCZPtr( biped ) ),
              PD_FOOT_LEFT,  pdBipedTimeStep( biped ) );
  pdFootInit( pdBipedRFPtr( biped ), pdCZHrzPtr( pdBipedCZPtr( biped ) ),
              PD_FOOT_RIGHT, pdBipedTimeStep( biped ) );
  biped->cmd = cmd;
  pdModeInit( &biped->mode );
  zVec3DClear( pdBipedRefCOMPos( biped ) );
  zVec3DClear( pdBipedRefBaseAtt( biped ) );
  zVec3DClear( pdBipedRefLFPos( biped ) );
  zVec3DClear( pdBipedRefLFAtt( biped ) );
  zVec3DClear( pdBipedRefRFPos( biped ) );
  zVec3DClear( pdBipedRefRFAtt( biped ) );
}

bool _pdBipedIsStateValid(pdBiped *biped, pdState *state)
{
  /* NOTE: This validation checking is insufficient */
  if( zIsTiny( zVec3DElem( &state->lf_pos, zZ ) -
               zVec3DElem( &state->com_pos, zZ ) ) ||
      zIsTiny( zVec3DElem( &state->rf_pos, zZ ) -
               zVec3DElem( &state->com_pos, zZ ) ) ){
    ZRUNERROR( "could not determine the height of COM" );
    return false;
  }
  if( zVec3DEqual( &state->lf_pos, ZVEC3DZERO ) &&
      zVec3DEqual( &state->rf_pos, ZVEC3DZERO ) ){
    ZRUNERROR( "could not determine the distance of feet" );
    return false;
  }
  return true;
}

void _pdBipedPoseInit(pdBiped *biped, pdState *state)
{
  zVec3D v;
  double foot_dist, com_height;
  double s, c, d, x, y, theta;
  double offset;

  offset = zPI_2;
  if( biped->cmd->dist > 0 )
    foot_dist = biped->cmd->dist;
  else {
    foot_dist = zVec3DDist( &state->lf_pos, &state->rf_pos );
    biped->cmd->dist = foot_dist;
  }
  if( biped->cmd->zd > 0 )
    com_height = biped->cmd->zd;
  else {
    com_height = zVec3DElem(&state->com_pos,zZ) - zVec3DElem(&state->lf_pos,zZ);
    com_height = 0.95 * com_height;
    biped->cmd->zd = com_height;
  }
  pdCZSetDist( pdBipedCZPtr(biped), foot_dist );
  theta = biped->cmd->thetad;
  zSinCos( theta, &s, &c );
  d = 0.5 * foot_dist;
  x = zVec3DElem( &state->com_pos, zX );
  y = zVec3DElem( &state->com_pos, zY );
  biped->cmd->xd = x;
  biped->cmd->yd = y;
  biped->cmd->xdd = biped->cmd->xd;
  biped->cmd->ydd = biped->cmd->yd;
  biped->cmd->zdd = biped->cmd->zd;

  zVec3DCopy( &state->com_pos, &v );
  zVec3DSetElem( &v, zZ, com_height );
  zVec3DCopy( &v, pdBipedRefCOMPos( biped ) );
  zVec3DCopy( &v, pdCZRefCOM( pdBipedCZPtr( biped ) ) );

  zVec3DCopy( &state->base_att, &v );
  zVec3DSetElem( &v, zX, theta + offset );
  zVec3DCopy( &v, pdBipedRefBaseAtt( biped ) );

  zVec3DCreate( &v, x-d*c, y-d*s, 0 );
  zVec3DCopy( &v, pdBipedRefLFPos( biped ) );

  zVec3DCopy( &state->lf_att, &v );
  zVec3DSetElem( &v, zX, theta + offset );
  zVec3DCopy( &v, pdBipedRefLFAtt( biped ) );

  zVec3DCreate( &v, x+d*c, y+d*s, 0 );
  zVec3DCopy( &v, pdBipedRefRFPos( biped ) );

  zVec3DCopy( &state->rf_att, &v );
  zVec3DSetElem( &v, zX, theta + offset );
  zVec3DCopy( &v, pdBipedRefRFAtt( biped ) );
}

bool pdBipedDefaultPoseInit(pdBiped *biped, pdState *state)
{
  if( !_pdBipedIsStateValid( biped, state ) )
    return false;
  _pdBipedPoseInit( biped, state );
  pdFootSetTrOldVec( pdBipedLFPtr(biped), pdBipedRefLFPos( biped ) );
  pdFootSetTrOldVec( pdBipedRFPtr(biped), pdBipedRefRFPos( biped ) );
  return true;
}

void pdBipedDestroy(pdBiped *biped)
{
  biped->cmd = NULL;
  pdFootDestroy( pdBipedLFPtr( biped ) );
  pdFootDestroy( pdBipedRFPtr( biped ) );
  pdCZDestroy( pdBipedCZPtr( biped ) );
  pdBipedSetTimeStep( biped, 0.0 );
  pdBipedResetTime( biped );
}

bool pdBipedDoesIntendToStand(pdBiped *biped)
{
  return zIsTiny( biped->cmd->rho ) &&
         zIsTiny( biped->cmd->vud ) &&
         zIsTiny( biped->cmd->vwd );
}

bool pdBipedDoesIntendToStep(pdBiped *biped)
{
  return !zIsTiny( biped->cmd->rho );
}

bool pdBipedDoesIntendToWalk(pdBiped *biped)
{
  return !zIsTiny( biped->cmd->vud );
}

bool pdBipedDoesIntendToSidewalk(pdBiped *biped)
{
  return !zIsTiny( biped->cmd->vwd );
}

void _pdBipedUpdateCommand(pdBiped *biped)
{
  pdCZSetQ1U( pdBipedCZPtr( biped ), biped->cmd->qu1 );
  /* pdCZSetQ2U( pdBipedCZPtr( biped ), biped->cmd->qu2 ); */
  pdCZSetQ1W( pdBipedCZPtr( biped ), biped->cmd->qw1 );
  pdCZSetQ2W( pdBipedCZPtr( biped ), biped->cmd->qw2 );
  pdCZSetQ1Z( pdBipedCZPtr( biped ), biped->cmd->qz1 );
  pdCZSetQ2Z( pdBipedCZPtr( biped ), biped->cmd->qz2 );
  pdCZSetKappa( pdBipedCZPtr( biped ), biped->cmd->kappa );
  /* pdCZSetRho( pdBipedCZPtr( biped ), biped->cmd->rho ); */
  pdCZSetKr( pdBipedCZPtr( biped ), biped->cmd->kr );
  pdCZSetKappa( pdBipedCZPtr( biped ), biped->cmd->kappa );
  pdCZSetCmdCOMX( pdBipedCZPtr( biped ), biped->cmd->xd );
  pdCZSetCmdCOMY( pdBipedCZPtr( biped ), biped->cmd->yd );
  pdCZSetCmdCOMZ( pdBipedCZPtr( biped ), biped->cmd->zd );
  pdCZSetCmdTheta( pdBipedCZPtr( biped ), biped->cmd->thetad );
  /* pdCZSetRefVelU( pdBipedCZPtr( biped ), biped->cmd->vud ); */
  /* pdCZSetRefVelW( pdBipedCZPtr( biped ), biped->cmd->vwd ); */
  /* pdCZSetDist( pdBipedCZPtr( biped ), biped->cmd->dist ); */
  pdCZSetErrCompKX( pdBipedCZPtr( biped ), biped->cmd->kx );
  pdCZSetErrCompKY( pdBipedCZPtr( biped ), biped->cmd->ky );
  pdCZSetErrCompKZ( pdBipedCZPtr( biped ), biped->cmd->kz );
  pdCZSetErrCompBX( pdBipedCZPtr( biped ), biped->cmd->bx );
  pdCZSetErrCompBY( pdBipedCZPtr( biped ), biped->cmd->by );
  pdCZSetErrCompBZ( pdBipedCZPtr( biped ), biped->cmd->bz );
  pdFootSetMaxHeight( pdBipedLFPtr( biped ), biped->cmd->lfh );
  pdFootSetTrXK( pdBipedLFPtr( biped ), biped->cmd->lfkx );
  pdFootSetTrXC( pdBipedLFPtr( biped ), biped->cmd->lfcx );
  pdFootSetTrYK( pdBipedLFPtr( biped ), biped->cmd->lfky );
  pdFootSetTrYC( pdBipedLFPtr( biped ), biped->cmd->lfcy );
  pdFootSetTrZK( pdBipedLFPtr( biped ), biped->cmd->lfkz );
  pdFootSetTrZC( pdBipedLFPtr( biped ), biped->cmd->lfcz );
  pdFootSetMaxHeight( pdBipedRFPtr( biped ), biped->cmd->rfh );
  pdFootSetTrXK( pdBipedRFPtr( biped ), biped->cmd->rfkx );
  pdFootSetTrXC( pdBipedRFPtr( biped ), biped->cmd->rfcx );
  pdFootSetTrYK( pdBipedRFPtr( biped ), biped->cmd->rfky );
  pdFootSetTrYC( pdBipedRFPtr( biped ), biped->cmd->rfcy );
  pdFootSetTrZK( pdBipedRFPtr( biped ), biped->cmd->rfkz );
  pdFootSetTrZC( pdBipedRFPtr( biped ), biped->cmd->rfcz );
}

void _pdBipedUpdateCZ(pdBiped *biped, pdState *state)
{
  double offset;

  offset = zPI_2;
  pdCZUpdate( pdBipedCZPtr(biped),
              &state->com_pos,
              &state->com_vel,
              &state->com_acc,
              &state->zmp,
              state->fz,
              &state->ef,
              state->base_att.e[0] - offset,
              &state->sr );
}

void _pdBipedUpdateFoot(pdBiped *biped, pdState *state)
{
  pdFootUpdate( pdBipedLFPtr(biped), pdBipedRFPtr(biped),
                pdCZDelta( pdBipedCZPtr(biped) ),
                pdCZVelUW( pdBipedCZPtr(biped) ),
                &state->deszmp,
                &state->lf_pos,
                &state->rf_pos,
                &state->lf_att,
                &state->rf_att,
                &state->sr_lf,
                &state->sr_rf );
}

void _pdBipedUpdateRef(pdBiped *biped, pdState *state)
{
  zVec3D v;
  double offset;

  offset = zPI_2;

  zVec3DCopy( pdCZRefCOM( pdBipedCZPtr(biped) ), pdBipedRefCOMPos(biped) );

  zVec3DCreate( &v, pdCZCmdTheta( pdBipedCZPtr(biped) ), 0, 0 );
  zVec3DElem( &v, zX ) += offset;
  zVec3DCopy( &v, pdBipedRefBaseAtt(biped) );

  zVec3DCopy( pdFootRefPos( pdBipedLFPtr(biped) ), pdBipedRefLFPos(biped) );
  zVec3DCopy( pdFootRefAtt( pdBipedLFPtr(biped) ), pdBipedRefLFAtt(biped) );
  zVec3DCopy( pdFootRefPos( pdBipedRFPtr(biped) ), pdBipedRefRFPos(biped) );
  zVec3DCopy( pdFootRefAtt( pdBipedRFPtr(biped) ), pdBipedRefRFAtt(biped) );
}

static double _pdBipedCalcDesFootDistFollow(pdBiped *biped, pdState *state);
static double _pdBipedCalcDesFootDistFollowToBrake(pdBiped *biped, pdState *state);
static double _pdBipedCalcDesFootDistBrake(pdBiped *biped, pdState *state);
static double _pdBipedCalcDesFootDistBrakeToFollow(pdBiped *biped, pdState *state);
double _pdBipedCalcDesFootDistFollow(pdBiped *biped, pdState *state)
{
  double q1, q2, zeta, phase;

  q1 = pdCZQ1W( pdBipedCZPtr(biped) );
  q2 = pdCZQ2W( pdBipedCZPtr(biped) );
  zeta = pdCZZeta( pdBipedCZPtr(biped) );
  if( pdBipedIsEitherFootOff(biped) )
    phase = pdFootPhase( pdBipedKFPtr( biped ) );
  else
    phase = 0;
  return biped->cmd->dist + zPIx2 * phase * fabs(pdCZRefVelW(pdBipedCZPtr(biped))) / ( zeta * sqrt( q1 * q2 ) );
}

double _pdBipedCalcDesFootDistFollowToBrake(pdBiped *biped, pdState *state)
{
  return pdStateFootDist( state );
}

double _pdBipedCalcDesFootDistBrake(pdBiped *biped, pdState *state)
{
  double phase, foot_dist;

  if( pdBipedIsEitherFootOff(biped) )
    phase = pdFootPhase( pdBipedKFPtr( biped ) );
  else
    phase = 0;
  foot_dist = pdStateFootDist( state );
  return foot_dist + phase * ( biped->cmd->dist - foot_dist );
}

double _pdBipedCalcDesFootDistBrakeToFollow(pdBiped *biped, pdState *state)
{
  return pdStateFootDist( state );
}

void _pdBipedModifyCommand(pdBiped *biped, pdState *state)
{
  zVec3D pd;
  double ref_dist;
  double s, c, dx, dy;

  pdCZSetRho( pdBipedCZPtr(biped), biped->cmd->rho );
  pdCZSetQ2U( pdBipedCZPtr( biped ), biped->cmd->qu2 );
  pdCZSetRefVelU( pdBipedCZPtr( biped ), biped->cmd->vud );
  pdCZSetRefVelW( pdBipedCZPtr( biped ), biped->cmd->vwd );
  pdCZSetDist( pdBipedCZPtr( biped ), biped->cmd->dist );
  if( biped->mode.trymove ){
    pdCZSetRho( pdBipedCZPtr(biped), 1.0 );
    if( !biped->mode.stepping ) {
      pdCZSetRefVelU( pdBipedCZPtr( biped ), 0.0 );
      pdCZSetRefVelW( pdBipedCZPtr( biped ), 0.0 );
    }
    if( biped->mode.walking )
      pdCZSetQ2U( pdBipedCZPtr( biped ), 0.0 );
  } else if( biped->mode.stepping ){
    pdCZSetRho( pdBipedCZPtr(biped), 1.0 );
  }

  if( biped->mode.sideways ) {
    if( biped->mode.following )
      ref_dist = _pdBipedCalcDesFootDistFollow( biped, state );
    else if( biped->mode.braking )
      ref_dist = _pdBipedCalcDesFootDistBrake( biped, state );
    else if( pdCZVelW( pdBipedCZPtr(biped) ) * biped->cmd->vwd > 0 )
      ref_dist = _pdBipedCalcDesFootDistFollowToBrake( biped, state );
    else
      ref_dist = _pdBipedCalcDesFootDistBrakeToFollow( biped, state );
    pdCZSetDist( pdBipedCZPtr( biped ), ref_dist );
  }
  pdCZAutoUpdateRef( pdBipedCZPtr(biped), &state->lf_pos, &state->rf_pos, &pd, &biped->cmd->thetad );
  biped->cmd->xd = pd.e[zX];
  biped->cmd->yd = pd.e[zY];

  if( biped->mode.warping ){
    dx = 0.1 * ( biped->cmd->xdd - biped->cmd->xd );
    dy = 0.1 * ( biped->cmd->ydd - biped->cmd->yd );
    if( dx > 0.01 )
      dx = zSgn( dx ) * zMax( fabs(dx), 0.05 );
    if( dy > 0.01 )
      dy = zSgn( dy ) * zMax( fabs(dy), 0.01 );
    zSinCos( pdCZTheta( pdBipedCZPtr( biped ) ), &s, &c );
    pdCZSetRho( pdBipedCZPtr(biped), 1.0 );
    if( biped->mode.stepping ){
      pdCZSetRefVelU( pdBipedCZPtr( biped ), -s*dx+c*dy );
      pdCZSetRefVelW( pdBipedCZPtr( biped ), -c*dx-s*dy );
      if( !zIsTiny( pdCZRefVelW(pdBipedCZPtr(biped)) ) ){
        if( pdStateBFOff( state, pdCZRefVelW(pdBipedCZPtr(biped)) ) )
          ref_dist = _pdBipedCalcDesFootDistFollow( biped, state );
        else if( pdStateFFOff( state, pdCZRefVelW(pdBipedCZPtr(biped)) ) )
          ref_dist = _pdBipedCalcDesFootDistBrake( biped, state );
        else if( pdCZVelW( pdBipedCZPtr(biped) ) * pdCZRefVelW(pdBipedCZPtr(biped)) > 0 )
          ref_dist = _pdBipedCalcDesFootDistFollowToBrake( biped, state );
        else
          ref_dist = _pdBipedCalcDesFootDistBrakeToFollow( biped, state );
        pdCZSetDist( pdBipedCZPtr( biped ), ref_dist );
      }
    }
  } else {
    biped->cmd->xdd = biped->cmd->xd;
    biped->cmd->ydd = biped->cmd->yd;
    biped->cmd->zdd = biped->cmd->zd;
  }
}

void pdBipedUpdate(pdBiped *biped, pdState *state)
{
  _pdBipedUpdateCommand( biped );
  _pdBipedUpdateCZ( biped, state );
  _pdBipedUpdateFoot( biped, state );
  _pdBipedUpdateRef( biped, state );
  pdModeUpdate( &biped->mode, biped->cmd, state );
  _pdBipedModifyCommand( biped, state );
  pdBipedIncrTime( biped );
}

void pdBipedUpdateState(pdBiped *biped, pdState *state)
{
  zVec3DCopy( pdBipedRefCOMPos(biped), &state->com_pos );
  zVec3DCopy( pdCZRefVel( pdBipedCZPtr(biped) ), &state->com_vel );
  zVec3DCopy( pdCZRefAcc( pdBipedCZPtr(biped) ), &state->com_acc );
  zVec3DCopy( pdBipedRefBaseAtt(biped), &state->base_att );
  zVec3DCopy( pdBipedRefLFPos(biped), &state->lf_pos );
  zVec3DCopy( pdBipedRefLFAtt(biped), &state->lf_att );
  zVec3DCopy( pdBipedRefRFPos(biped), &state->rf_pos );
  zVec3DCopy( pdBipedRefRFAtt(biped), &state->rf_att );
  zVec3DCopy( pdCZRefZMP( pdBipedCZPtr(biped) ), &state->zmp );
  zVec3DCopy( pdCZRefZMP( pdBipedCZPtr(biped) ), &state->deszmp );
  state->fz = pdCZVrtRF( &biped->cz._vrt );
}

void pdBipedFWrite(FILE *fp, pdBiped *biped)
{
  pdCZ *c;
  pdFoot *lf, *rf;

  c = pdBipedCZPtr( biped );
  lf = pdBipedLFPtr( biped );
  rf = pdBipedRFPtr( biped );
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "t:%g, dt:%g\n", pdCZTime(c), pdCZTimeStep(c) );
  fprintf( fp, "xd:%g, yd:%g, zd:%g, thetad:%g\n",
           pdCZCmdCOMX(c), pdCZCmdCOMY(c), pdCZCmdCOMZ(c), pdCZCmdTheta(c) );
  fprintf( fp, "x :%g, y: %g, z: %g, theta :%g\n",
           pdCZCOMX(c), pdCZCOMY(c), pdCZCOMZ(c), pdCZTheta(c) );
  fprintf( fp, "vx:%g, vy:%g, vz:%g\n", pdCZVelX(c), pdCZVelY(c), pdCZVelZ(c) );
  fprintf( fp, "ax:%g, ay:%g, az:%g\n", pdCZAccX(c), pdCZAccY(c), pdCZAccZ(c) );
  fprintf( fp, "zx:%g, zy:%g, zz:%g\n", pdCZZMPX(c), pdCZZMPY(c), pdCZZMPZ(c) );
  fprintf( fp, "ud:%g, deltau:%g, vu:%g\n",
           pdCZRefPosU(c), pdCZDeltaU(c), pdCZVelU(c) );
  fprintf( fp, "wd:%g, deltaw:%g, vw:%g\n",
           pdCZRefPosW(c), pdCZDeltaW(c), pdCZVelW(c) );
  fprintf( fp, "refx: %g, refy: %g, refz: %g\n",
           pdCZRefCOMX(c), pdCZRefCOMY(c), pdCZRefCOMZ(c) );
  fprintf( fp, "refvx:%g, refvy:%g, refvz:%g\n",
           pdCZRefVelX(c), pdCZRefVelY(c), pdCZRefVelZ(c) );
  fprintf( fp, "refax:%g, refay:%g, refaz:%g\n",
           pdCZRefAccX(c), pdCZRefAccY(c), pdCZRefAccZ(c) );
  fprintf( fp, "refzx:%g, refzy:%g, refzz:%g\n",
           pdCZRefZMPX(c), pdCZRefZMPY(c), pdCZRefZMPZ(c) );

  fprintf( fp, "--\n" );
  fprintf( fp, "lf    x:%g,    y:%g,    z:%g\n",
           zVec3DElem(&lf->_p,zX), zVec3DElem(&lf->_p,zY),
           zVec3DElem(&lf->_p,zZ) );
  fprintf( fp, "lf   xd:%g,   yd:%g,   zd:%g\n",
           zVec3DElem(&lf->_pd,zX), zVec3DElem(&lf->_pd,zY),
           zVec3DElem(&lf->_pd,zZ) );
  fprintf( fp, "lf refx:%g, refy:%g, refz:%g\n",
           zVec3DElem(&lf->refp,zX), zVec3DElem(&lf->refp,zY),
           zVec3DElem(&lf->refp,zZ) );
  fprintf( fp, "lf azim:%g, elev:%g, tilt:%g\n",
           zVec3DElem(&lf->_a,zX), zVec3DElem(&lf->_a,zY),
           zVec3DElem(&lf->_a,zZ) );
  fprintf( fp, "lf desa:%g, dese:%g, dest:%g\n",
           zVec3DElem(&lf->_ad,zX), zVec3DElem(&lf->_ad,zY),
           zVec3DElem(&lf->_ad,zZ) );
  fprintf( fp, "lf refa:%g, refe:%g, reft:%g\n",
           zVec3DElem(&lf->refa,zX), zVec3DElem(&lf->refa,zY),
           zVec3DElem(&lf->refa,zZ) );
  fprintf( fp, "lf sign:%g, kappa:%g, dist:%g, phi:%g\n",
           pdFootUWSign(pdFootUWPtr(lf)), pdFootUWKappa(pdFootUWPtr(lf)),
           pdFootUWDist(pdFootUWPtr(lf)), pdFootUWPhi(pdFootUWPtr(lf)) );
  fprintf( fp, "lf reguz:%g, regwz:%g\n",
           pdFootUWRegZMPU(pdFootUWPtr(lf)), pdFootUWRegZMPW(pdFootUWPtr(lf)) );
  fprintf( fp, "lf refud:%g, refwd:%g\n",
           pdFootUWRefPosU(pdFootUWPtr(lf)), pdFootUWRefPosW(pdFootUWPtr(lf)) );
  fprintf( fp, "lf sign:%g, h:%g, rho:%g, dist:%g\n",
           pdFootZSign(pdFootZPtr(lf)), pdFootZMaxHeight(pdFootZPtr(lf)),
           pdFootZRho(pdFootZPtr(lf)), pdFootZDist(pdFootZPtr(lf)) );
  fprintf( fp, "lf pz:" );zComplexFWrite( fp, pdFootZZMPPhase(pdFootZPtr(lf)) );
  fprintf( fp, ", phase:%g, zd:%g\n",
           pdFootZFootPhase(pdFootZPtr(lf)), pdFootZRefZ(pdFootZPtr(lf)) );

  fprintf( fp, "--\n" );
  fprintf( fp, "rf    x:%g,    y:%g,    z:%g\n",
           zVec3DElem(&rf->_p,zX), zVec3DElem(&rf->_p,zY),
           zVec3DElem(&rf->_p,zZ) );
  fprintf( fp, "rf   xd:%g,   yd:%g,   zd:%g\n",
           zVec3DElem(&rf->_pd,zX), zVec3DElem(&rf->_pd,zY),
           zVec3DElem(&rf->_pd,zZ) );
  fprintf( fp, "rf refx:%g, refy:%g, refz:%g\n",
           zVec3DElem(&rf->refp,zX), zVec3DElem(&rf->refp,zY),
           zVec3DElem(&rf->refp,zZ) );
  fprintf( fp, "rf azim:%g, elev:%g, tilt:%g\n",
           zVec3DElem(&rf->_a,zX), zVec3DElem(&rf->_a,zY),
           zVec3DElem(&rf->_a,zZ) );
  fprintf( fp, "rf desa:%g, dese:%g, dest:%g\n",
           zVec3DElem(&rf->_ad,zX), zVec3DElem(&rf->_ad,zY),
           zVec3DElem(&rf->_ad,zZ) );
  fprintf( fp, "rf refa:%g, refe:%g, reft:%g\n",
           zVec3DElem(&rf->refa,zX), zVec3DElem(&rf->refa,zY),
           zVec3DElem(&rf->refa,zZ) );
  fprintf( fp, "rf sign:%g, kappa:%g, dist:%g, phi:%g\n",
           pdFootUWSign(pdFootUWPtr(rf)), pdFootUWKappa(pdFootUWPtr(rf)),
           pdFootUWDist(pdFootUWPtr(rf)), pdFootUWPhi(pdFootUWPtr(rf)) );
  fprintf( fp, "rf reguz:%g, regwz:%g\n", pdFootUWRegZMPU(pdFootUWPtr(rf)),
           pdFootUWRegZMPW(pdFootUWPtr(rf)) );
  fprintf( fp, "rf refud:%g, refwd:%g\n",
           pdFootUWRefPosU(pdFootUWPtr(rf)), pdFootUWRefPosW(pdFootUWPtr(rf)) );
  fprintf( fp, "rf sign:%g, h:%g, rho:%g, dist:%g\n",
           pdFootZSign(pdFootZPtr(rf)), pdFootZMaxHeight(pdFootZPtr(rf)),
           pdFootZRho(pdFootZPtr(rf)), pdFootZDist(pdFootZPtr(rf)) );
  fprintf( fp, "rf pz:" );zComplexFWrite( fp, pdFootZZMPPhase(pdFootZPtr(rf)) );
  fprintf( fp, ", phase:%g, zd:%g\n",
           pdFootZFootPhase(pdFootZPtr(rf)), pdFootZRefZ(pdFootZPtr(rf)) );
}

void pdBipedDataFWrite(FILE *fp, pdBiped *biped)
{
  pdCZ *c;
  pdFoot *lf, *rf;

  c = pdBipedCZPtr( biped );
  lf = pdBipedLFPtr( biped );
  rf = pdBipedRFPtr( biped );
  fprintf( fp, "%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n",
/* 0- 1*/  pdBipedTime(biped), pdBipedTimeStep(biped),
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
