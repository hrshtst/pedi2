#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( pdRobotChainPtr( robot ) );
  pdRobotJointDis( robot ) = NULL;
}

void pdRobotLoad(pdRobot *robot, const char model_file[])
{
  /* load robot model file */
  if( !rkChainReadFile( pdRobotChainPtr( robot ), (char *)model_file ) ){
    ZRUNERROR( "cannot load %s", model_file );
    exit( EXIT_FAILURE );
  }

  /* IK solver */
  if( !rkIKCreate( pdRobotIKPtr( robot ), pdRobotChainPtr( robot ) ) )
    exit( EXIT_FAILURE );
  if( !rkIKConfReadFile( pdRobotIKPtr( robot ), pdRobotChainPtr( robot ), (char *) model_file ) )
    exit( EXIT_FAILURE );

  /* joint displacement vector */
  if( !( pdRobotJointDis( robot ) = zVecAlloc( pdRobotJointSize( robot ) ) ) )
    exit( EXIT_FAILURE );
}

void pdRobotDestroy(pdRobot *robot)
{
  zVecFree( pdRobotJointDis( robot ) );
  rkIKDestroy( pdRobotIKPtr( robot ) );
  rkChainDestroy( pdRobotChainPtr( robot ) );
}

void pdRobotSolveIK(pdRobot *robot)
{
  rkIKCell *cell[6];
  int i;
  zVec3D v;

  for( i=0; i<6; i++ )
    cell[i] = rkIKFindCell( pdRobotIKPtr(robot), i );
  rkIKDeactivate( &robot->_ik );
  zVec3DCreate( &v, 0.0, 0.0, 0.26 );
  rkIKCellSetRefVec( cell[0], &v );
  zVec3DCreate( &v, 0.0, 0.0, 0.0 );
  rkIKCellSetRefVec( cell[1], &v );
  zVec3DCreate( &v, 0.0, 0.042, 0.0 );
  rkIKCellSetRefVec( cell[2], &v );
  zVec3DCreate( &v, 0.0, 0.0, 0.0 );
  rkIKCellSetRefVec( cell[3], &v );
  zVec3DCreate( &v, 0.0, -0.042, 0.0 );
  rkIKCellSetRefVec( cell[4], &v );
  zVec3DCreate( &v, 0.0, 0.0, 0.0 );
  rkIKCellSetRefVec( cell[5], &v );
  rkIKSolve( pdRobotIKPtr(robot), pdRobotJointDis(robot), zTOL, 0 );
}
