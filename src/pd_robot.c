#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( pdRobotChainPtr( robot ) );
  pdRobotJointDis( robot ) = NULL;
  robot->_cell = NULL;
  robot->_num_cell = 0;
  robot->_base_id = -1;
  robot->_lf_id = -1;
  robot->_rf_id = -1;
  robot->_lh_id = -1;
  robot->_rh_id = -1;
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

  for( i=0; i<robot->_num_cell; i++ ){
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

  /* load robot model file */
  if( !rkChainReadFile( pdRobotChainPtr( robot ), (char *)model_file ) ){
    ZRUNERROR( "cannot load %s", model_file );
    goto ERROR;
  }

  /* IK solver */
  if( !rkIKCreate( pdRobotIKPtr( robot ), pdRobotChainPtr( robot ) ) )
    goto ERROR;
  if( !rkIKConfReadFile( pdRobotIKPtr( robot ), pdRobotChainPtr( robot ), (char *) model_file ) )
    goto ERROR;
  if( ( robot->_num_cell = zListNum( &pdRobotIKPtr( robot )->clist ) ) < PD_ROBOT_REQUIRED_CONST_NUM )
    goto ERROR;
  if( !( robot->_cell = zAlloc( rkIKCell*, robot->_num_cell ) ) )
    goto ERROR;
  for( i=0; i<robot->_num_cell; i++ )
    robot->_cell[i] = rkIKFindCell( pdRobotIKPtr(robot), i );
  /* initialize reference vector */
  if( !( robot->_ref_vec = zAlloc( zVec3D, robot->_num_cell ) ) )
    goto ERROR;
  if( !_pdRobotSetLinkID( robot ) )
    goto ERROR;
  if( !_pdRobotInitRefVec( robot ) )
    goto ERROR;

  /* joint displacement vector */
  if( !( pdRobotJointDis( robot ) = zVecAlloc( pdRobotJointSize( robot ) ) ) )
    goto ERROR;
  return true;
 ERROR:
  pdRobotDestroy( robot );
  return false;
}

void pdRobotDestroy(pdRobot *robot)
{
  zVecFree( pdRobotJointDis( robot ) );
  zFree( robot->_cell );
  robot->_num_cell = 0;
  rkIKDestroy( pdRobotIKPtr( robot ) );
  rkChainDestroy( pdRobotChainPtr( robot ) );
}

void pdRobotSolveIK(pdRobot *robot)
{
  zVec3D v;

  rkIKDeactivate( &robot->_ik );
  zVec3DCreate( &v, 0.0, 0.0, 0.26 );
  rkIKCellSetRefVec( robot->_cell[0], &v );
  zVec3DCreate( &v, 0.0, 0.0, 0.0 );
  rkIKCellSetRefVec( robot->_cell[1], &v );
  zVec3DCreate( &v, 0.0, 0.042, 0.0 );
  rkIKCellSetRefVec( robot->_cell[2], &v );
  zVec3DCreate( &v, 0.0, 0.0, 0.0 );
  rkIKCellSetRefVec( robot->_cell[3], &v );
  zVec3DCreate( &v, 0.0, -0.042, 0.0 );
  rkIKCellSetRefVec( robot->_cell[4], &v );
  zVec3DCreate( &v, 0.0, 0.0, 0.0 );
  rkIKCellSetRefVec( robot->_cell[5], &v );
  rkIKSolve( pdRobotIKPtr(robot), pdRobotJointDis(robot), zTOL, 0 );
}
