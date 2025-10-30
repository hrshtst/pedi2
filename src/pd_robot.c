#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( pdRobotChain( robot ) );
  pdRobotJointDis( robot ) = NULL;
  robot->disold = NULL;
  zVec3DDataInitList( pdRobotSRLFVert( robot ) );
  zVec3DDataInitList( pdRobotSRRFVert( robot ) );
  zVec3DDataInitList( pdRobotSRVert( robot ) );
  robot->_torso_id = -1;
  robot->_lf_id = -1;
  robot->_rf_id = -1;
  robot->_lh_id = -1;
  robot->_rh_id = -1;
}

static void _pdRobotSetTorsoLinkID(pdRobot *robot)
{
  const char *torso_cell_names[] = {
    PD_ROBOT_IKCELL_NAME_TORSO_ATT,
    NULL,
  };
  pdRobotTorsoID( robot ) = pdRobotFindLinkIDByIKCellName( robot, torso_cell_names );
}
static void _pdRobotSetLeftFootLinkID(pdRobot *robot)
{
  const char *left_foot_cell_names[] = {
    PD_ROBOT_IKCELL_NAME_LF_POS,
    PD_ROBOT_IKCELL_NAME_LF_ATT,
    NULL,
  };
  pdRobotLFID( robot ) = pdRobotFindLinkIDByIKCellName( robot, left_foot_cell_names );
}
static void _pdRobotSetRightFootLinkID(pdRobot *robot)
{
  const char *right_foot_cell_names[] = {
    PD_ROBOT_IKCELL_NAME_RF_POS,
    PD_ROBOT_IKCELL_NAME_RF_ATT,
    NULL,
  };
  pdRobotRFID( robot ) = pdRobotFindLinkIDByIKCellName( robot, right_foot_cell_names );
}
static void _pdRobotSetLeftHandLinkID(pdRobot *robot)
{
  const char *left_hand_cell_names[] = {
    PD_ROBOT_IKCELL_NAME_LH_POS,
    PD_ROBOT_IKCELL_NAME_LH_ATT,
    NULL,
  };
  pdRobotLHID( robot ) = pdRobotFindLinkIDByIKCellName( robot, left_hand_cell_names );
}
static void _pdRobotSetRightHandLinkID(pdRobot *robot)
{
  const char *right_hand_cell_names[] = {
    PD_ROBOT_IKCELL_NAME_RH_POS,
    PD_ROBOT_IKCELL_NAME_RH_ATT,
    NULL,
  };
  pdRobotRHID( robot ) = pdRobotFindLinkIDByIKCellName( robot, right_hand_cell_names );
}
static void _pdRobotSetLinkID(pdRobot *robot)
{
  _pdRobotSetTorsoLinkID( robot );
  _pdRobotSetLeftFootLinkID( robot );
  _pdRobotSetRightFootLinkID( robot );
  _pdRobotSetLeftHandLinkID( robot );
  _pdRobotSetRightHandLinkID( robot );
}

static bool _pdRobotBindLink(pdRobot *robot, const char *linkname, const char *cellname, int priority, rkIKCell *(*register_ik_cell_fp)(rkChain*,const char*,int,rkIKAttr*,ubyte))
{
  rkLink *link;
  rkIKAttr attr;
  ubyte mask;

  rkIKAttrInit( &attr );
  mask = RK_IK_ATTR_MASK_NONE;
  if( linkname ){
    if( !( link = rkChainFindLink( pdRobotChain( robot ), linkname ) ) )
      return false;
    attr.id = link - rkChainRoot( pdRobotChain( robot ) );
    mask |= RK_IK_ATTR_MASK_ID;
  }
  if( !register_ik_cell_fp( pdRobotChain( robot ), cellname, priority, &attr, mask) )
    return false;
  return true;
}

static bool _pdRobotAllocSR(pdRobot *robot){
  rkLink *foot;
  zShape3D s;
  zShapeListCell *sp;
  int n_vert_lf, n_vert_rf;

  /* create a set of vertices of feet */
  /* left foot */
  n_vert_lf = 0;
  foot = rkChainLink( pdRobotChain(robot), pdRobotLFID(robot) );
  zListForEach( rkLinkShapeList(foot), sp ){
    if( sp->data->com == &zeo_shape3d_ph_com ){
      n_vert_lf += zShape3DVertNum( sp->data );
    } else{
      zShape3DClone( sp->data, &s, NULL );
      if( !zShape3DToPH( &s ) ) return false;
      n_vert_lf += zShape3DVertNum( &s );
      zShape3DDestroy( &s );
    }
  }
  if( !zVec3DDataInitArray( pdRobotSRLFVert( robot ), n_vert_lf ) ){
    ZRUNERROR( "cannot allocate vertices of left foot support region" );
    return false;
  }
  /* right foot */
  n_vert_rf = 0;
  foot = rkChainLink( pdRobotChain(robot), pdRobotRFID(robot) );
  zListForEach( rkLinkShapeList(foot), sp ){
    if( sp->data->com == &zeo_shape3d_ph_com ){
      n_vert_rf += zShape3DVertNum( sp->data );
    } else{
      zShape3DClone( sp->data, &s, NULL );
      if( !zShape3DToPH( &s ) ) return false;
      n_vert_rf += zShape3DVertNum( &s );
      zShape3DDestroy( &s );
    }
  }
  if( !zVec3DDataInitArray( pdRobotSRRFVert( robot ), n_vert_rf ) ){
    ZRUNERROR( "cannot allocate vertices of right foot support region" );
    return false;
  }
  /* both feet */
  if( !zVec3DDataInitArray( pdRobotSRVert( robot ), n_vert_lf + n_vert_rf ) ){
    ZRUNERROR( "cannot allocate vertices of  support region" );
    return false;
  }
  return true;
}

bool pdRobotLoad(pdRobot *robot, const char model_file[])
{
  /* load robot model file */
  pdRobotInit( robot );
  if( !rkChainReadZTK( pdRobotChain( robot ), model_file ) ){
    ZRUNERROR( "cannot load model file: %s", model_file );
    goto ERROR;
  }

  /* create IK solver */
  if( !rkChainIKConfReadZTK( pdRobotChain( robot ), model_file ) ){
    ZRUNERROR( "failed to read IK solver configuration: %s", model_file );
    goto ERROR;
  }
  if( pdRobotIKCellListSize( robot ) > 0 )
    _pdRobotSetLinkID( robot );
  else{
    if( !pdRobotRegisterIKJointAll( robot, 0.001 ) ) goto ERROR;
    if( !_pdRobotBindLink( robot, NULL, PD_ROBOT_IKCELL_NAME_COM,
                           PD_ROBOT_DEFAULT_PRIORITY_COM, rkChainRegisterIKCellCOM ) ) goto ERROR;
  }
  rkChainDisableIK( pdRobotChain( robot ) );

  /* joint displacement vector */
  if( !( pdRobotJointDis( robot ) = zVecAlloc( pdRobotJointSize( robot ) ) ) ){
    ZRUNERROR( "cannot allocate joint displacement vector" );
    goto ERROR;
  }
  /* old joint displacement vector */
  if( !( robot->disold = zVecAlloc( pdRobotJointSize( robot ) ) ) ){
    ZRUNERROR( "cannot allocate old joint displacement vector" );
    goto ERROR;
  }
  return true;

 ERROR:
  pdRobotDestroy( robot );
  return false;
}

void pdRobotDestroy(pdRobot *robot)
{
  zVecFree( pdRobotJointDis( robot ) );
  zVecFree( robot->disold );
  zVec3DDataDestroy( pdRobotSRLFVert( robot ) );
  zVec3DDataDestroy( pdRobotSRRFVert( robot ) );
  zVec3DDataDestroy( pdRobotSRVert( robot ) );
  rkChainDestroy( pdRobotChain( robot ) );
}

bool pdRobotBindTorso(pdRobot *robot, const char torso[])
{
  if( !_pdRobotBindLink( robot, torso,
                         PD_ROBOT_IKCELL_NAME_TORSO_ATT,
                         PD_ROBOT_DEFAULT_PRIORITY_TORSO_ATT,
                         rkChainRegisterIKCellWldAtt ) ) return false;
  rkChainDisableIK( pdRobotChain( robot ) );
  _pdRobotSetTorsoLinkID( robot );
  return true;
}

bool pdRobotBindFeet(pdRobot *robot, const char left_foot[], const char right_foot[])
{
  if( !_pdRobotBindLink( robot, left_foot,
                         PD_ROBOT_IKCELL_NAME_LF_POS,
                         PD_ROBOT_DEFAULT_PRIORITY_LF_POS,
                         rkChainRegisterIKCellWldPos ) ) return false;
  if( !_pdRobotBindLink( robot, left_foot,
                         PD_ROBOT_IKCELL_NAME_LF_ATT,
                         PD_ROBOT_DEFAULT_PRIORITY_LF_ATT,
                         rkChainRegisterIKCellWldAtt ) ) return false;
  _pdRobotSetLeftFootLinkID( robot );

  if( !_pdRobotBindLink( robot, right_foot,
                         PD_ROBOT_IKCELL_NAME_RF_POS,
                         PD_ROBOT_DEFAULT_PRIORITY_RF_POS,
                         rkChainRegisterIKCellWldPos ) ) return false;
  if( !_pdRobotBindLink( robot, right_foot,
                         PD_ROBOT_IKCELL_NAME_RF_ATT,
                         PD_ROBOT_DEFAULT_PRIORITY_RF_ATT,
                         rkChainRegisterIKCellWldAtt ) ) return false;
  _pdRobotSetRightFootLinkID( robot );

  rkChainDisableIK( pdRobotChain( robot ) );

  if( pdRobotLFID( robot ) > -1 && pdRobotRFID( robot ) > -1 )
    if( !_pdRobotAllocSR( robot ) ) return false;
  return true;
}

bool pdRobotBindHands(pdRobot *robot, const char left_hand[], const char right_hand[])
{
  if( !_pdRobotBindLink( robot, left_hand,
                         PD_ROBOT_IKCELL_NAME_LH_POS,
                         PD_ROBOT_DEFAULT_PRIORITY_LH_POS,
                         rkChainRegisterIKCellWldPos ) ) return false;
  if( !_pdRobotBindLink( robot, left_hand,
                         PD_ROBOT_IKCELL_NAME_LH_ATT,
                         PD_ROBOT_DEFAULT_PRIORITY_LH_ATT,
                         rkChainRegisterIKCellWldAtt ) ) return false;
  _pdRobotSetLeftHandLinkID( robot );

  if( !_pdRobotBindLink( robot, right_hand,
                         PD_ROBOT_IKCELL_NAME_RH_POS,
                         PD_ROBOT_DEFAULT_PRIORITY_RH_POS,
                         rkChainRegisterIKCellWldPos ) ) return false;
  if( !_pdRobotBindLink( robot, right_hand,
                         PD_ROBOT_IKCELL_NAME_RH_ATT,
                         PD_ROBOT_DEFAULT_PRIORITY_RH_ATT,
                         rkChainRegisterIKCellWldAtt ) ) return false;
  _pdRobotSetRightHandLinkID( robot );

  rkChainDisableIK( pdRobotChain( robot ) );
  return true;
}

rkIKCell *pdRobotFindIKCellByName(pdRobot *robot, const char *name)
{
  return rkChainFindIKCellByName( pdRobotChain( robot ), name );
}

int pdRobotFindLinkIDByIKCellName(pdRobot *robot, const char *ikcell_names[])
{
  int i;
  rkIKCell *cell;

  for( i=0; ikcell_names[i] != NULL; i++ )
    if( ( cell = pdRobotFindIKCellByName( robot, ikcell_names[i] ) ) )
      return cell->data.attr.id;
  return -1;
}

void pdRobotLinkJointSetDis(pdRobot *robot, int id, double *dis)
{
  rkChainLinkJointSetDis( pdRobotChain(robot), id, dis );
  rkChainGetJointDisAll( pdRobotChain(robot), pdRobotJointDis(robot) );
}

void pdRobotSetJointDis(pdRobot *robot, zIndex index, zVec dis)
{
  rkChainSetJointDis( pdRobotChain(robot), index, dis );
  rkChainGetJointDisAll( pdRobotChain(robot), pdRobotJointDis(robot) );
}

void pdRobotGetJointDiffAll(pdRobot *robot, zVec v)
{
  zVecSub( robot->disold, robot->dis, v );
}

void pdRobotGetJointVelAll(pdRobot *robot, double dt, zVec v)
{
  pdRobotGetJointDiffAll( robot, v );
  zVecDivDRC( v, dt );
}

void pdRobotFK(pdRobot *robot, zVec dis)
{
  double torso, foot;

  rkChainFK( pdRobotChain(robot), dis );
  torso = rkChainLinkWldPos(pdRobotChain(robot),pdRobotTorsoID(robot))->c.z;
  foot = zMin( rkChainLinkWldPos(pdRobotChain(robot),pdRobotLFID(robot))->c.z,
               rkChainLinkWldPos(pdRobotChain(robot),pdRobotRFID(robot))->c.z );
  zVecSetElem( dis, zZ, torso - foot );
  rkChainFK( pdRobotChain(robot), dis );
  rkChainGetJointDisAll( pdRobotChain(robot), pdRobotJointDis(robot) );
  pdRobotGetJointDisAll( robot, robot->disold );
}

void pdRobotFKIndex(pdRobot *robot, zIndex index, zVec dis)
{
  register int i;

  for( i=0; i<(int)zArraySize(index); i++ )
    zVecSetElem( pdRobotJointDis(robot), zIndexElem(index,i), zVecElem(dis,i) );
  pdRobotFK( robot, pdRobotJointDis( robot ) );
}

void pdRobotResetJointDis(pdRobot *robot)
{
  zVec v;

  v = zVecAlloc( pdRobotJointSize(robot) );
  zVecZero( v );
  pdRobotFK( robot, v );
  zVecFree( v );
}

bool pdRobotSetRef(pdRobot *robot, const char *ikcell_name, double v1, double v2, double v3){
  rkIKCell *cell;

  if( !( cell = pdRobotFindIKCellByName( robot, ikcell_name ) ) ){
    ZRUNERROR( "constraint '%s' is not bound", ikcell_name );
    return false;
  }
  rkIKCellSetRef( cell, v1, v2, v3 );
  return true;
}

bool pdRobotSetRefVec(pdRobot *robot, const char *ikcell_name, zVec3D *vec){
  return pdRobotSetRef( robot, ikcell_name, vec->c.x, vec->c.y, vec->c.z );
}

bool pdRobotSetRefAtt(pdRobot *robot, const char *ikcell_name, zMat3D *att){
  rkIKCell *cell;

  if( !( cell = pdRobotFindIKCellByName( robot, ikcell_name ) ) ){
    ZRUNERROR( "constraint '%s' is not bound", ikcell_name );
    return false;
  }
  rkIKCellSetRefAtt( cell, att );
  return true;
}

zVec3D *pdRobotGetRefPos(pdRobot *robot, const char *ikcell_name, zVec3D *pos){
  rkIKCell *cell;

  if( !( cell = pdRobotFindIKCellByName( robot, ikcell_name ) ) ) return NULL;
  zVec3DCopy( rkIKCellRefPos( cell ), pos );
  return pos;
}

zVec3D *pdRobotGetRefZYX(pdRobot *robot, const char *ikcell_name, zVec3D *zyx){
  rkIKCell *cell;

  if( !( cell = pdRobotFindIKCellByName( robot, ikcell_name ) ) ) return NULL;
  return zMat3DToZYX( rkIKCellRefAtt( cell ), zyx );
}

zMat3D *pdRobotGetRefAtt(pdRobot *robot, const char *ikcell_name, zMat3D *att){
  rkIKCell *cell;

  if( !( cell = pdRobotFindIKCellByName( robot, ikcell_name ) ) ) return NULL;
  zMat3DCopy( rkIKCellRefAtt( cell ), att );
  return att;
}

void pdRobotSolveIK(pdRobot *robot, int iter)
{
  pdRobotGetJointDisAll( robot, robot->disold );
  rkChainIK( pdRobotChain( robot ), pdRobotJointDis( robot ), zTOL, iter );
  rkChainDisableIK( pdRobotChain( robot ) );
}

void pdRobotCOMPos(pdRobot *robot, zVec3D *com)
{
  zVec3DCopy( rkChainWldCOM( pdRobotChain( robot ) ), com );
}

void pdRobotTorsoZYX(pdRobot *robot, zVec3D *zyx)
{
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotTorsoID( robot ) ), zyx );
}

void pdRobotTorsoAtt(pdRobot *robot, zMat3D *att)
{
  zMat3DCopy( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotTorsoID( robot ) ), att );
}

void pdRobotFootPos(pdRobot *robot, zVec3D *lf, zVec3D *rf)
{
  zVec3DCopy( rkChainLinkWldPos( pdRobotChain( robot ), pdRobotLFID( robot ) ), lf );
  zVec3DCopy( rkChainLinkWldPos( pdRobotChain( robot ), pdRobotRFID( robot ) ), rf );
}

void pdRobotFootZYX(pdRobot *robot, zVec3D *lf, zVec3D *rf)
{
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotLFID( robot ) ), lf );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotRFID( robot ) ), rf );
}

void pdRobotFootAtt(pdRobot *robot, zMat3D *lf, zMat3D *rf)
{
  zMat3DCopy( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotLFID( robot ) ), lf );
  zMat3DCopy( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotRFID( robot ) ), rf );
}

void pdRobotHandPos(pdRobot *robot, zVec3D *lh, zVec3D *rh)
{
  zVec3DCopy( rkChainLinkWldPos( pdRobotChain( robot ), pdRobotLHID( robot ) ), lh );
  zVec3DCopy( rkChainLinkWldPos( pdRobotChain( robot ), pdRobotRHID( robot ) ), rh );
}

void pdRobotHandZYX(pdRobot *robot, zVec3D *lh, zVec3D *rh)
{
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotLHID( robot ) ), lh );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotRHID( robot ) ), rh );
}

void pdRobotHandAtt(pdRobot *robot, zMat3D *lh, zMat3D *rh)
{
  zMat3DCopy( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotLHID( robot ) ), lh );
  zMat3DCopy( rkChainLinkWldAtt( pdRobotChain( robot ), pdRobotRHID( robot ) ), rh );
}

void pdRobotBipedDefaultInit(pdRobot *robot, pdBiped *biped, pdState *state)
{
  pdRobotUpdateState( robot, state );
  pdBipedDefaultPoseInit( biped, state );
  pdRobotBipedSetRefVec( robot, biped );
  pdRobotSolveIK( robot, 0 );
  pdRobotGetJointDisAll( robot, robot->disold );
  pdRobotUpdateState( robot, state );
}

void pdRobotBipedResetPose(pdRobot *robot, pdBiped *biped, pdState *state, zVec dis)
{
  double offset;

  pdRobotFK( robot, dis );
  pdRobotUpdateState( robot, state );
  zVec3DZero( &state->com_vel );
  zVec3DZero( &state->com_acc );
  state->lf_pos.c.z = 0;
  state->rf_pos.c.z = 0;
  zVec3DMid( &state->lf_pos, &state->rf_pos, &state->deszmp );
  zVec3DCopy( &state->deszmp, &state->zmp );
  offset = zPI_2;
  pdBipedCmd(biped)->thetad = state->torso_att.e[0] - offset;
  pdBipedDefaultPoseInit( biped, state );
  pdModeInit( &biped->mode );
}

void pdRobotBipedSetRefVec(pdRobot *robot, pdBiped *biped)
{
  pdRobotSetRefCOM( robot, pdBipedRefCOMPos(biped) );
  pdRobotSetRefTorsoZYX( robot, pdBipedRefTorsoAtt(biped) );
  pdRobotSetRefLFPos( robot, pdBipedRefLFPos(biped) );
  pdRobotSetRefLFZYX( robot, pdBipedRefLFAtt(biped) );
  pdRobotSetRefRFPos( robot, pdBipedRefRFPos(biped) );
  pdRobotSetRefRFZYX( robot, pdBipedRefRFAtt(biped) );
}

#define PD_ROBOT_TOL (1.0e-4)
bool pdRobotSupportRegion(pdRobot *robot, zLoop3D *sr_lf, zLoop3D *sr_rf, zLoop3D *sr)
{
  int i;
  rkLink *foot;
  zShape3D s;
  zShapeListCell *sp;
  zVec3D v;
  bool result;

  zVec3DDataDestroy( &robot->_sr_vert );
  zVec3DDataInitList( &robot->_sr_vert );
  /* left foot */
  foot = rkChainLink( pdRobotChain( robot ), pdRobotLFID( robot ) );
  zVec3DDataDestroy( &robot->_sr_lf_vert );
  zVec3DDataInitList( &robot->_sr_lf_vert );
  result = true;
  zListForEach( rkLinkShapeList(foot), sp ){
    if( sp->data->com == &zeo_shape3d_ph_com ){
      for( i=0; i<zShape3DVertNum(sp->data); i++ ){
        zXform3D( rkLinkWldFrame(foot), zShape3DVert(sp->data,i), &v );
        if( v.c.z < PD_ROBOT_TOL ){
          if( !zVec3DDataAdd( &robot->_sr_lf_vert, &v ) ) return false;
          if( !zVec3DDataAdd( &robot->_sr_vert, &v ) ) return false;
        }
      }
    } else{
      zShape3DClone( sp->data, &s, NULL );
      if( !zShape3DToPH( &s ) ){
        result = false;
      } else{
        for( i=0; i<zShape3DVertNum(&s); i++ ){
          zXform3D( rkLinkWldFrame(foot), zShape3DVert(&s,i), &v );
          if( v.c.z < PD_ROBOT_TOL ){
            if( !zVec3DDataAdd( &robot->_sr_lf_vert, &v ) ) return false;
            if( !zVec3DDataAdd( &robot->_sr_vert, &v ) ) return false;
          }
        }
      }
      zShape3DDestroy( &s );
      if( !result ) return false;
    }
  }
  /* right foot */
  foot = rkChainLink( pdRobotChain( robot ), pdRobotRFID( robot ) );
  zVec3DDataDestroy( &robot->_sr_rf_vert );
  zVec3DDataInitList( &robot->_sr_rf_vert );
  result = true;
  zListForEach( rkLinkShapeList(foot), sp ){
    if( sp->data->com == &zeo_shape3d_ph_com ){
      for( i=0; i<zShape3DVertNum(sp->data); i++ ){
        zXform3D( rkLinkWldFrame(foot), zShape3DVert(sp->data,i), &v );
        if( v.c.z < PD_ROBOT_TOL ){
          if( !zVec3DDataAdd( &robot->_sr_rf_vert, &v ) ) return false;
          if( !zVec3DDataAdd( &robot->_sr_vert, &v ) ) return false;
        }
      }
    } else{
      zShape3DClone( sp->data, &s, NULL );
      if( !zShape3DToPH( &s ) ){
        result = false;
      } else{
        for( i=0; i<zShape3DVertNum(&s); i++ ){
          zXform3D( rkLinkWldFrame(foot), zShape3DVert(&s,i), &v );
          if( v.c.z < PD_ROBOT_TOL ){
            if( !zVec3DDataAdd( &robot->_sr_rf_vert, &v ) ) return false;
            if( !zVec3DDataAdd( &robot->_sr_vert, &v ) ) return false;
          }
        }
      }
      zShape3DDestroy( &s );
      if( !result ) return false;
    }
  }
  /* supporting region */
  zLoop3DDestroy( sr_lf ); zListInit( sr_lf );
  zLoop3DDestroy( sr_rf ); zListInit( sr_rf );
  zLoop3DDestroy( sr ); zListInit( sr );
  if( zVec3DDataSize( &robot->_sr_lf_vert ) > 0 ){
    if( !zVec3DDataConvexHull2D( &robot->_sr_lf_vert, sr_lf ) ) return false;
  }
  if( zVec3DDataSize( &robot->_sr_rf_vert ) > 0 ){
    if( !zVec3DDataConvexHull2D( &robot->_sr_rf_vert, sr_rf ) ) return false;
  }
  if( zVec3DDataSize( &robot->_sr_vert ) > 0 ){
    if( !zVec3DDataConvexHull2D( &robot->_sr_vert, sr ) ) return false;
  }
  return true;
}

void pdRobotUpdateState(pdRobot *robot, pdState *state)
{
  pdRobotCOMPos( robot, &state->com_pos );
  pdRobotTorsoZYX( robot, &state->torso_att );
  pdRobotFootPos( robot, &state->lf_pos, &state->rf_pos );
  pdRobotFootZYX( robot, &state->lf_att, &state->rf_att );
  pdRobotHandPos( robot, &state->lh_pos, &state->rh_pos );
  pdRobotHandZYX( robot, &state->lh_att, &state->rh_att );
  pdRobotSupportRegion( robot, &state->sr_lf, &state->sr_rf, &state->sr );
}

void pdRobotFPrint(FILE *fp, pdRobot *r)
{
  rkIKCell *cell;

  /* for debug */
  fprintf( fp, "==========\n" );
  fprintf( fp, "          Torso ID: %d\n", pdRobotTorsoID(r) );
  fprintf( fp, "Left/Right foot ID: %d/%d\n", pdRobotLFID(r), pdRobotRFID(r) );
  fprintf( fp, "Left/Right hand ID: %d/%d\n", pdRobotLHID(r), pdRobotRHID(r) );
  fprintf( fp, "IK cell size: %d\n", pdRobotIKCellListSize(r) );
  zListForEach( pdRobotIKCellList(r), cell ){
    fprintf( fp, "----------\n" );
    fprintf( fp, "IK Cell Name: %s (%s)\n", zName(&cell->data), cell->data.constraint->typestr );
    if( cell->data.constraint->ref_fp == &rkIKRefSetPos ){
      fprintf( fp, " Ref (position): " );
      zVec3DFPrint( fp, &cell->data.ref.pos );
    } else{
      fprintf( fp, " Ref (attitude): " );
      zMat3DFPrint( fp, &cell->data.ref.att );
    }
    fprintf( fp, "Link ID(Sub ID): %d (%d)\n", cell->data.attr.id, cell->data.attr.id_sub );
    fprintf( fp, "Attention Point: " );
    zVec3DFPrint( fp, &cell->data.attr.attention_point );
    fprintf( fp, "         Weight: " );
    zVec3DFPrint( fp, &cell->data.attr.weight );
    fprintf( fp, " Attribute Mask: %#04x\n", cell->data.attr.mask );
    fprintf( fp, "       Priority: %d\n", cell->data.priority );
    fprintf( fp, "      Cell Mode: %#04x\n", cell->data.mode );
    fprintf( fp, "       Enabled?: %s\n", rkIKCellIsEnabled(cell) ? "true" : "false" );
  }
}
