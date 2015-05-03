#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( pdRobotChainPtr( robot ) );
}

void pdRobotLoad(pdRobot *robot, const char model_file[])
{
  /* load robot model file */
  if( !rkChainReadFile( pdRobotChainPtr( robot ), (char *)model_file ) ){
    ZRUNERROR( "cannot load %s", model_file );
    exit( EXIT_FAILURE );
  }
}

void pdRobotDestroy(pdRobot *robot)
{
  rkChainDestroy( pdRobotChainPtr( robot ) );
}
