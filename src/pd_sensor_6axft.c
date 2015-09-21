#include <pedi2/pd_sensor.h>

void pdSensorDestroy6AxisFT(pdSensor *sensor)
{
  pdSensorDestroyDefault( sensor );
}

zVec pdSensorProcess6AxisFT(pdSensor *sensor, double dt)
{
  register int i;

  pdFilterArrayUpdate( pdSensorFilterArray(sensor), dt );
  for( i=0; i<pdSensorSize(sensor); i++ )
    pdSensorOutputVal( sensor, i ) = pdFilterOutput( zArrayElem(pdSensorFilterArray(sensor),i) );
  return pdSensorOutput( sensor );
}

pdSensor *pdSensorFRead6AxisFT(FILE *fp, pdSensor *sensor)
{
}

pdSensorMethod pd_sensor_6axisft_met = {
  type: "6axisft",
  destroy: pdSensorDestroy6AxisFT,
  process: pdSensorProcess6AxisFT,
  fread: pdSensorFRead6AxisFT,
};

#define PD_SENSOR_6AXIS_SIZE 6
bool pdSensorCreate6AxisFT(pdSensor *sensor)
{
  pdSensorInit( sensor );
  pdSensorSize( sensor ) = PD_SENSOR_6AXIS_SIZE;
  pdSensorInput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  pdSensorOutput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  pdFilterArrayAlloc( pdSensorFilterArray(sensor), pdSensorSize(sensor) );
  sensor->_met = &pd_sensor_6axisft_met;
  return true;
}
