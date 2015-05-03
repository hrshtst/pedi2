#include <pedi2/pd_robot.h>

void pdRobotInit(pdRobot *robot)
{
  rkChainInit( pdRobotChainPtr( robot ) );
}

void pdRobotDestroy(pdRobot *robot)
{
  rkChainDestroy( pdRobotChainPtr( robot ) );
}
