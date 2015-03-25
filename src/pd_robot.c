#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( &robot->chain );

  robot->body_id = -1;
  robot->lf_id = -1;
  robot->rf_id = -1;

  zListInit( &robot->sr_lf );
  zListInit( &robot->sr_rf );
  zListInit( &robot->sr );
}

void pdRobotLoad(pdRobot *robot, const char* model_file, const char* conf_file)
{
  register int i;

  /* load robot model_file */
  if( !rkChainReadFile( &robot->chain, (char *)model_file ) )
    exit( EXIT_FAILURE );

  /* find attending links by name */
  for( i=0; i<rkChainNum(&robot->chain); i++ ){
    if( !strcmp( "body", rkChainLinkName(&robot->chain,i) ) )
      robot->body_id = i;
    if( !strcmp( "left_foot", rkChainLinkName(&robot->chain,i) ) )
      robot->lf_id = i;
    if( !strcmp( "right_foot", rkChainLinkName(&robot->chain,i) ) )
      robot->rf_id = i;
  }
  if( robot->body_id < 0 || robot->lf_id < 0 || robot->rf_id < 0 ){
    ZRUNERROR( "Check if links named 'body', 'left_foot' and 'right_foot' are exist in %s", model_file );
    exit( EXIT_FAILURE );
  }

  /* initialize constraints for IK solver */
  zVec3DCopy( rkChainWldCOM(&robot->chain), &robot->d_com_pos );
  zVec3DCopy( rkChainLinkWldPos(&robot->chain,robot->lf_id), &robot->d_lf_pos );
  zVec3DCopy( rkChainLinkWldPos(&robot->chain,robot->rf_id), &robot->d_rf_pos );
  zMat3DToZYX( rkChainLinkWldAtt(&robot->chain,robot->body_id), &robot->d_body_att );
  zMat3DToZYX( rkChainLinkWldAtt(&robot->chain,robot->lf_id), &robot->d_lf_att );
  zMat3DToZYX( rkChainLinkWldAtt(&robot->chain,robot->rf_id), &robot->d_rf_att );

  /* IK solver */
  if( !rkIKCreate( &robot->ik, &robot->chain ) )
    exit( EXIT_FAILURE );
  if( !rkIKConfReadFile( &robot->ik, (char *)conf_file ) )
    exit( EXIT_FAILURE );
  for( i=0; i<6; i++ )
    robot->cell[i] = rkIKFindCell( &robot->ik, i );

  /* joint displacement vector */
  if( !( robot->dis = zVecAlloc( rkChainJointSize(&robot->chain) ) ) )
    exit( EXIT_FAILURE );
}

void dmRobotExit(pdRobot *robot)
{
  zVecFree( robot->dis );
  rkIKDestroy( &robot->ik );
  zVec3DListDestroy( &robot->sr_lf, false );
  zVec3DListDestroy( &robot->sr_rf, false );
  zVec3DListDestroy( &robot->sr, false );
  rkChainDestroy( &robot->chain );
}
