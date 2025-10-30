#ifndef __PD_SENSOR_DUMMY_H__
#define __PD_SENSOR_DUMMY_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__PEDI2_EXPORT pdSensor *pdSensorDummyCreate(pdSensor *sensor, const char linkname[], zFrame3D *linkframe);

extern pdSensorCom pd_sensor_dummy_com;

__END_DECLS

#endif /* __PD_SENSOR_DUMMY_H__ */
