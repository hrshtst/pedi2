#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( pdRobotChainPtr( robot ) );
  pdRobotJointDis( robot ) = NULL;
  robot->_cell = NULL;
  pdRobotCellNum( robot ) = 0;
  robot->_ref_vec = NULL;
  robot->_ref_set_flag = NULL;
  robot->_base_id = -1;
  robot->_lf_id = -1;
  robot->_rf_id = -1;
  robot->_lh_id = -1;
  robot->_rh_id = -1;
  robot->_sr_lf_vert = NULL;
  robot->_sr_rf_vert = NULL;
  robot->_sr_vert = NULL;
}

int _pdRobotCheckLinkID(pdRobot *robot, pdRobotIKCellID pos_id, pdRobotIKCellID att_id )
{
  if( robot->_cell[pos_id]->data.attr.id == robot->_cell[att_id]->data.attr.id )
    return robot->_cell[pos_id]->data.attr.id;
  else{
    ZRUNERROR( "Link ID is mismatched!" );
    return -1;
  }
}

bool _pdRobotSetLinkID(pdRobot *robot)
{
  robot->_base_id = robot->_cell[PD_ROBOT_IKCELL_ID_BASE_ATT]->data.attr.id;
  robot->_lf_id = _pdRobotCheckLinkID( robot,
                                       PD_ROBOT_IKCELL_ID_LF_POS,
                                       PD_ROBOT_IKCELL_ID_LF_ATT );
  robot->_rf_id = _pdRobotCheckLinkID( robot,
                                       PD_ROBOT_IKCELL_ID_RF_POS,
                                       PD_ROBOT_IKCELL_ID_RF_ATT );
  robot->_lh_id = _pdRobotCheckLinkID( robot,
                                       PD_ROBOT_IKCELL_ID_LH_POS,
                                       PD_ROBOT_IKCELL_ID_LH_ATT );
  robot->_rh_id = _pdRobotCheckLinkID( robot,
                                       PD_ROBOT_IKCELL_ID_RH_POS,
                                       PD_ROBOT_IKCELL_ID_RH_ATT );
  if( robot->_base_id < 0 ||
      robot->_lf_id < 0 || robot->_rf_id < 0 ||
      robot->_lh_id < 0 || robot->_rh_id < 0 )
    return false;
  else
    return true;
}

static struct _pdRobotLookup{
  rkIKCMat_fp cmat_fp;
  char *str;
} __pd_robot_lookup[] = {
  { rkIKJacobiLinkWldLin, "world_pos" },
  { rkIKJacobiLinkWldAng, "world_att" },
  { rkIKJacobiLinkL2LLin, "l2l_pos"   },
  { rkIKJacobiLinkL2LAng, "l2l_att"   },
  { rkIKJacobiCOM,        "com"       },
  { rkIKJacobiAM,         "am"        },
  { rkIKJacobiAMCOM,      "amcom"     },
  { NULL, NULL },
};

struct _pdRobotLookup *_pdRobotLookupCell(rkIKCMat_fp cmat_fp)
{
  struct _pdRobotLookup *lookup;

  for( lookup=__pd_robot_lookup; lookup->cmat_fp; lookup++ )
    if( cmat_fp == lookup->cmat_fp ) return lookup;
  ZRUNERROR( "unknown Jacobian matrix" );
  return NULL;
}

bool _pdRobotInitRefVec(pdRobot *robot)
{
  struct _pdRobotLookup *lookup;
  int id;
  register int i;

  for( i=0; i<pdRobotCellNum( robot ); i++ ){
    lookup = _pdRobotLookupCell( robot->_cell[i]->data._cmat_fp );
    id = robot->_cell[i]->data.attr.id;
    if( strcmp( lookup->str, "world_pos" ) == 0 ) {
      zVec3DCopy( rkChainLinkWldPos(pdRobotChainPtr(robot), id ),
                  &robot->_ref_vec[i] );
    } else if( strcmp( lookup->str, "world_att" ) == 0 ) {
      zMat3DToZYX( rkChainLinkWldAtt(pdRobotChainPtr(robot), id ),
                   &robot->_ref_vec[i] );
    } else if( strcmp( lookup->str, "l2l_pos" ) == 0 ) {
      ZRUNERROR( "Sorry, not implemented! (l2l_pos)" );
      return false;
    } else if( strcmp( lookup->str, "l2l_att" ) == 0 ) {
      ZRUNERROR( "Sorry, not implemented! (l2l_att)" );
      return false;
    } else if( strcmp( lookup->str, "com" ) == 0 ) {
      zVec3DCopy( rkChainWldCOM(pdRobotChainPtr(robot) ),
                  &robot->_ref_vec[i] );
    } else if( strcmp( lookup->str, "am" ) == 0 ) {
      ZRUNERROR( "Sorry, not implemented! (am)" );
      return false;
    } else if( strcmp( lookup->str, "amcom" ) == 0 ) {
      ZRUNERROR( "Sorry, not implemented! (amcom)" );
      return false;
    }
  }
  return true;
}

bool pdRobotLoad(pdRobot *robot, const char model_file[])
{
  register int i;
  rkLink *foot;
  zShape3D *sole;
  int n_vert_lf, n_vert_rf;

  /* load robot model file */
  if( !rkChainReadFile( pdRobotChainPtr( robot ), (char *)model_file ) ){
    ZRUNERROR( "cannot load %s", model_file );
    goto ERROR;
  }

  /* IK solver */
  if( !rkIKCreate( pdRobotIKPtr( robot ), pdRobotChainPtr( robot ) ) ){
    ZRUNERROR( "failed to create IK solver" );
    goto ERROR;
  }
  if( !rkIKConfReadFile( pdRobotIKPtr( robot ), pdRobotChainPtr( robot ), (char *) model_file ) ){
    ZRUNERROR( "failed to read IK conf file %s", model_file );
    goto ERROR;
  }
  if( ( pdRobotCellNum( robot ) = zListNum( &pdRobotIKPtr( robot )->clist ) ) < PD_ROBOT_REQUIRED_CONST_NUM ){
    ZRUNERROR( "lack sufficient constraints, you need %d constraints but only %d specified",
               PD_ROBOT_REQUIRED_CONST_NUM, pdRobotCellNum( robot ) );
    goto ERROR;
  }
  if( !( robot->_cell = zAlloc( rkIKCell*, pdRobotCellNum( robot ) ) ) ){
    ZRUNERROR( "cannot allocate IK cell" );
    goto ERROR;
  }
  for( i=0; i<pdRobotCellNum( robot ); i++ )
    robot->_cell[i] = rkIKFindCell( pdRobotIKPtr(robot), i );
  /* initialize reference vector */
  if( !( robot->_ref_vec = zAlloc( zVec3D, pdRobotCellNum( robot ) ) ) ){
    ZRUNERROR( "cannot allocate reference vectors" );
    goto ERROR;
  }
  if( !( robot->_ref_set_flag = zAlloc( bool, pdRobotCellNum( robot ) ) ) ){
    ZRUNERROR( "cannot allocate reference set flags" );
    goto ERROR;
  }
  pdRobotUnsetAllFlags( robot );
  if( !_pdRobotSetLinkID( robot ) ){
    ZRUNERROR( "failed to set link IDs" );
    goto ERROR;
  }
  if( !_pdRobotInitRefVec( robot ) ){
    ZRUNERROR( "failed to init referential vector" );
    goto ERROR;
  }

  /* vertices of supporting region */
  /* left foot */
  foot = rkChainLink( pdRobotChainPtr(robot), pdRobotLFID(robot) );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  if( zShape3DType( sole ) != ZSHAPE_PH )
    sole = zShape3DToPH( sole );
  n_vert_lf = zShape3DVertNum( sole );
  if( !( robot->_sr_lf_vert = zAlloc( zVec3D, n_vert_lf ) ) ){
    ZRUNERROR( "cannot allocate vertices of left foot support region" );
    goto ERROR;
  }
  /* right foot */
  foot = rkChainLink( pdRobotChainPtr(robot), pdRobotRFID(robot) );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  if( zShape3DType( sole ) != ZSHAPE_PH )
    sole = zShape3DToPH( sole );
  n_vert_rf = zShape3DVertNum( sole );
  if( !( robot->_sr_rf_vert = zAlloc( zVec3D, n_vert_rf ) ) ){
    ZRUNERROR( "cannot allocate vertices of right foot support region" );
    goto ERROR;
  }
  /* both feet */
  if( !( robot->_sr_vert = zAlloc( zVec3D, n_vert_lf + n_vert_rf ) ) ){
    ZRUNERROR( "cannot allocate vertices of support region" );
    goto ERROR;
  }

  /* joint displacement vector */
  if( !( pdRobotJointDis( robot ) = zVecAlloc( pdRobotJointSize( robot ) ) ) ){
    ZRUNERROR( "cannot allocate joint displacement vector" );
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
  zFree( robot->_sr_vert );
  zFree( robot->_sr_rf_vert );
  zFree( robot->_sr_lf_vert );
  zFree( robot->_ref_set_flag );
  zFree( robot->_ref_vec );
  zFree( robot->_cell );
  pdRobotCellNum( robot ) = 0;
  rkIKDestroy( pdRobotIKPtr( robot ) );
  rkChainDestroy( pdRobotChainPtr( robot ) );
}

void pdRobotUnsetAllFlags(pdRobot *robot)
{
  register int i;

  for( i=0; i<pdRobotCellNum( robot ); i++ )
    robot->_ref_set_flag[i] = false;
}

bool pdRobotSetRefVec(pdRobot *robot, zVec3D *ref, int id)
{
  if( id < pdRobotCellNum( robot ) ){
    zVec3DCopy( ref, &robot->_ref_vec[id] );
    robot->_ref_set_flag[id] = true;
    return true;
  } else {
    ZRUNERROR( "IK Cell id %d is invalid.", id );
    return false;
  }
}

void pdRobotSetBipedRefVec(pdRobot *robot, pdBiped *biped)
{
  pdRobotSetRefCOM( robot, pdBipedRefCOMPos(biped) );
  pdRobotSetRefBaseAtt( robot, pdBipedRefBaseAtt(biped) );
  pdRobotSetRefLFPos( robot, pdBipedRefLFPos(biped) );
  pdRobotSetRefLFAtt( robot, pdBipedRefLFAtt(biped) );
  pdRobotSetRefRFPos( robot, pdBipedRefRFPos(biped) );
  pdRobotSetRefRFAtt( robot, pdBipedRefRFAtt(biped) );
}

void pdRobotSolveIK(pdRobot *robot)
{
  register int i;

  rkIKDeactivate( &robot->_ik );
  for( i=0; i<pdRobotCellNum( robot ); i++ )
    if( pdRobotFlagIsOn( robot, i ) )
      rkIKCellSetRefVec( robot->_cell[i], &robot->_ref_vec[i] );
  rkIKSolve( pdRobotIKPtr(robot), pdRobotJointDis(robot), zTOL, 0 );
  pdRobotUnsetAllFlags( robot );
}

void pdRobotCOMPos(pdRobot *robot, zVec3D *com)
{
  zVec3DCopy( rkChainWldCOM( pdRobotChainPtr( robot ) ), com );
}

void pdRobotBaseAtt(pdRobot *robot, zVec3D *att)
{
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( robot ), pdRobotBaseID( robot ) ), att );
}

void pdRobotFootPos(pdRobot *robot, zVec3D *lf, zVec3D *rf)
{
  zVec3DCopy( rkChainLinkWldPos( pdRobotChainPtr( robot ), pdRobotLFID( robot ) ), lf );
  zVec3DCopy( rkChainLinkWldPos( pdRobotChainPtr( robot ), pdRobotRFID( robot ) ), rf );
}

void pdRobotFootAtt(pdRobot *robot, zVec3D *lf, zVec3D *rf)
{
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( robot ), pdRobotLFID( robot ) ), lf );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( robot ), pdRobotRFID( robot ) ), rf );
}

void pdRobotHandPos(pdRobot *robot, zVec3D *lh, zVec3D *rh)
{
  zVec3DCopy( rkChainLinkWldPos( pdRobotChainPtr( robot ), pdRobotLHID( robot ) ), lh );
  zVec3DCopy( rkChainLinkWldPos( pdRobotChainPtr( robot ), pdRobotRHID( robot ) ), rh );
}

void pdRobotHandAtt(pdRobot *robot, zVec3D *lh, zVec3D *rh)
{
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( robot ), pdRobotLHID( robot ) ), lh );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( robot ), pdRobotRHID( robot ) ), rh );
}

#define PD_ROBOT_TOL (1.0e-4)
void pdRobotSupportRegion(pdRobot *robot, zVec3DList *sr_lf, zVec3DList *sr_rf, zVec3DList *sr)
{
  int i, nl, nr, n;
  rkLink *foot;
  zShape3D *sole;
  zVec3D v;

  nl = nr = n = 0;
  /* left foot */
  foot = rkChainLink( pdRobotChainPtr( robot ), pdRobotLFID( robot ) );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  if( zShape3DType( sole ) != ZSHAPE_PH )
    sole = zShape3DToPH( sole );
  for( i=0; i<(int)zShape3DVertNum(sole); i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < PD_ROBOT_TOL ){
      zVec3DCopy( &v, &robot->_sr_lf_vert[nl++] );
      zVec3DCopy( &v, &robot->_sr_vert[n++] );
    }
  }
  /* right foot */
  foot = rkChainLink( pdRobotChainPtr( robot ), pdRobotRFID( robot ) );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  if( zShape3DType( sole ) != ZSHAPE_PH )
    sole = zShape3DToPH( sole );
  for( i=0; i<(int)zShape3DVertNum(sole); i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < PD_ROBOT_TOL ){
      zVec3DCopy( &v, &robot->_sr_rf_vert[nr++] );
      zVec3DCopy( &v, &robot->_sr_vert[n++] );
    }
  }
  /* supporting region */
  zVec3DListDestroy( sr_lf, false );
  zVec3DListDestroy( sr_rf, false );
  zVec3DListDestroy( sr, false );
  if( nl > 0 ) zCH2D( sr_lf, robot->_sr_lf_vert, nl );
  if( nr > 0 ) zCH2D( sr_rf, robot->_sr_rf_vert, nr );
  if( n  > 0 ) zCH2D( sr, robot->_sr_vert, n );
}

void pdRobotUpdateState(pdRobot *robot, pdState *state)
{
  pdRobotCOMPos( robot, &state->com_pos );
  pdRobotBaseAtt( robot, &state->base_att );
  pdRobotFootPos( robot, &state->lf_pos, &state->rf_pos );
  pdRobotFootAtt( robot, &state->lf_att, &state->rf_att );
  pdRobotHandPos( robot, &state->lh_pos, &state->rh_pos );
  pdRobotHandAtt( robot, &state->lh_att, &state->rh_att );
  pdRobotSupportRegion( robot, &state->sr_lf, &state->sr_rf, &state->sr );
}

void pdRobotFWrite(FILE *fp, pdRobot *r)
{
  register int i;

  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "cell num:%d\n", pdRobotCellNum(r) );
  for( i=0; i<pdRobotCellNum(r); i++ ){
    fprintf( fp, "ref %d(%s):", i, r->_ref_set_flag[i] ? "true" : "false" );
    zVec3DFWrite( fp, &r->_ref_vec[i] );
  }
  fprintf( fp, "base id:%d\n", pdRobotBaseID(r) );
  fprintf( fp, "lf id:%d, rf id:%d\n", pdRobotLFID(r), pdRobotRFID(r) );
  fprintf( fp, "lh id:%d, rh id:%d\n", pdRobotLHID(r), pdRobotRHID(r) );
}
