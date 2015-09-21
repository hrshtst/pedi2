#include <pedi2/pd_sensor.h>

void pdSensorDestroy6AxisFT(pdSensor *sensor)
{
  pdSensorDestroyDefault( sensor );
}

zVec pdSensorProcess6AxisFT(pdSensor *sensor, double dt)
{
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

bool pdSensorCreate6AxisFT(pdSensor *sensor)
{
  pdSensorInit( sensor );
  sensor->_met = &pd_sensor_6axisft_met;
}
