#include <pedi2/pd_core.h>

void pdCoreInit(pdCore *core, pdCmd *cmd, double dt)
{
  pdCoreResetTime( core );
  pdCoreSetTimeStep( core, dt );
  pdCZInit( pdCoreCZPtr( core ), pdCoreTimeStep( core ) );
  pdFootInit( pdCoreLFPtr( core ), pdCZHrzPtr( pdCoreCZPtr( core ) ),
              PD_FOOT_LEFT,  pdCoreTimeStep( core ) );
  pdFootInit( pdCoreRFPtr( core ), pdCZHrzPtr( pdCoreCZPtr( core ) ),
              PD_FOOT_RIGHT, pdCoreTimeStep( core ) );
  pdRobotInit( pdCoreRobotPtr( core ) );
  pdStateInit( pdCoreStatePtr( core ) );
  core->cmd = cmd;
}

bool pdCoreLoad(pdCore *core, char *filename)
{
  return pdRobotLoad( pdCoreRobotPtr( core ), filename );
}

void pdCoreDestroy(pdCore *core)
{
  core->cmd = NULL;
  pdStateDestroy( pdCoreStatePtr( core ) );
  pdRobotDestroy( pdCoreRobotPtr( core ) );
  pdFootDestroy( pdCoreLFPtr( core ) );
  pdFootDestroy( pdCoreRFPtr( core ) );
  pdCZDestroy( pdCoreCZPtr( core ) );
  pdCoreSetTimeStep( core, 0.0 );
  pdCoreResetTime( core );
}
