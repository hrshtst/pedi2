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
