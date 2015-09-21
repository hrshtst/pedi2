#include <pedi2/pd_sensor.h>

void pdSensorDestroyDefault(pdSensor *sensor)
{
  zNameDestroy( sensor );
  pdFilterArrayDestroy( pdSensorFilterArray( sensor ) );
  zVecFree( pdSensorInput( sensor ) );
  zVecFree( pdSensorOutput( sensor ) );
  zFree( sensor->_prm );
  pdSensorInit( sensor );
}
