#include <pedi2/pd_robot.h>

void _pdRobotWriteRefVec(pdRobot *robot)
{
  /* for debug */
  printf("d_com_pos:");zVec3DWrite(&robot->d_com_pos);
  printf("d_lf_pos :");zVec3DWrite(&robot->d_lf_pos);
  printf("d_rf_pos :");zVec3DWrite(&robot->d_rf_pos);
  printf("d_bod_att:");zVec3DWrite(&robot->d_body_att);
  printf("d_lf_att :");zVec3DWrite(&robot->d_lf_att);
  printf("d_rf_att :");zVec3DWrite(&robot->d_rf_att);
}

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
    else if( !strcmp( "waist_base", rkChainLinkName(&robot->chain,i) ) )
      robot->body_id = i;       /* for hydra */
    else if( !strcmp( "left_foot", rkChainLinkName(&robot->chain,i) ) )
      robot->lf_id = i;
    else if( !strcmp( "right_foot", rkChainLinkName(&robot->chain,i) ) )
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

void pdRobotExit(pdRobot *robot)
{
  zVecFree( robot->dis );
  rkIKDestroy( &robot->ik );
  zVec3DListDestroy( &robot->sr_lf, false );
  zVec3DListDestroy( &robot->sr_rf, false );
  zVec3DListDestroy( &robot->sr, false );
  rkChainDestroy( &robot->chain );
}

void pdRobotSolveIK(pdRobot *robot)
{
  /* _pdRobotWriteRefVec( robot ); */
  rkIKDeactivate( &robot->ik );
  rkIKCellSetRefVec( robot->cell[0], &robot->d_com_pos );
  rkIKCellSetRefVec( robot->cell[1], &robot->d_body_att );
  rkIKCellSetRefVec( robot->cell[2], &robot->d_lf_pos );
  rkIKCellSetRefVec( robot->cell[3], &robot->d_lf_att );
  rkIKCellSetRefVec( robot->cell[4], &robot->d_rf_pos );
  rkIKCellSetRefVec( robot->cell[5], &robot->d_rf_att );
  rkIKSolve( &robot->ik, robot->dis, zTOL, 0 );
}

#define PD_ROBOT_TOL (1.0e-3)
void pdRobotSupportRegion(pdRobot *robot)
{
  int i, nl, nr, n;
  rkLink *foot;
  zShape3D *sole;
  zVec3D v;

  nl = nr = n = 0;
  /* left foot */
  foot = rkChainLink( &robot->chain, robot->lf_id );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < PD_ROBOT_TOL ){
      zVec3DCopy( &v, &robot->sr_lf_vert[nl++] );
      zVec3DCopy( &v, &robot->sr_vert[n++] );
    }
  }
  /* right foot */
  foot = rkChainLink( &robot->chain, robot->rf_id );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < PD_ROBOT_TOL ){
      zVec3DCopy( &v, &robot->sr_rf_vert[nr++] );
      zVec3DCopy( &v, &robot->sr_vert[n++] );
    }
  }
  /* supporting region */
  zVec3DListDestroy( &robot->sr_lf, false );
  zVec3DListDestroy( &robot->sr_rf, false );
  zVec3DListDestroy( &robot->sr, false );
  if( nl > 0 ) zCH2D( &robot->sr_lf, robot->sr_lf_vert, nl );
  if( nr > 0 ) zCH2D( &robot->sr_rf, robot->sr_rf_vert, nr );
  if( n  > 0 ) zCH2D( &robot->sr, robot->sr_vert, n );
}

void pdRobotFootPos(pdRobot *robot, zVec3D *lf, zVec3D *rf)
{
  zVec3DCopy( rkChainLinkWldPos(&robot->chain,robot->lf_id), lf );
  zVec3DCopy( rkChainLinkWldPos(&robot->chain,robot->rf_id), rf );
}

void pdRobotFootAtt(pdRobot *robot, zVec3D *lf, zVec3D *rf)
{
  zMat3DToZYX( rkChainLinkWldAtt(&robot->chain,robot->lf_id), lf );
  zMat3DToZYX( rkChainLinkWldAtt(&robot->chain,robot->rf_id), rf );
}
