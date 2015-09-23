#ifndef __PD_SENSOR_6FT_H__
#define __PD_SENSOR_6FT_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT bool pdSensorCreate6FT(pdSensor *sensor, zFrame3D *frame, pdFilterArray *arr);

__EXPORT zVec3D *pdSensor6FTGetF(pdSensor *sensor, zVec3D *f);
__EXPORT zVec3D *pdSensor6FTGetT(pdSensor *sensor, zVec3D *tau);
__EXPORT zVec3D *pdSensor6FTGetWldF(pdSensor *sensor, zVec3D *f);
__EXPORT zVec3D *pdSensor6FTGetWldT(pdSensor *sensor, zVec3D *tau);

extern pdSensorMethod pd_sensor_6ft_met;

__END_DECLS

#endif /* __PD_SENSOR_6FT_H__ */
