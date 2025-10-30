#ifndef __PD_SENSOR_6FT_H__
#define __PD_SENSOR_6FT_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__PEDI2_EXPORT pdSensor *pdSensor6FTCreate(pdSensor *sensor, const char linkname[], zFrame3D *linkframe, pdFilterArray *farray);

__PEDI2_EXPORT zVec3D *pdSensor6FTGetF(pdSensor *sensor, zVec3D *f);
__PEDI2_EXPORT zVec3D *pdSensor6FTGetT(pdSensor *sensor, zVec3D *tau);
__PEDI2_EXPORT zVec3D *pdSensor6FTGetWldF(pdSensor *sensor, zVec3D *f);
__PEDI2_EXPORT zVec3D *pdSensor6FTGetWldT(pdSensor *sensor, zVec3D *tau);

extern pdSensorCom pd_sensor_6ft_com;

__END_DECLS

#endif /* __PD_SENSOR_6FT_H__ */
