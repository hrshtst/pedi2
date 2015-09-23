#ifndef __PD_SENSOR_6FT_H__
#define __PD_SENSOR_6FT_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT bool pdSensorCreate6FT(pdSensor *sensor, zFrame3D *frame, pdFilterArray *arr);

extern pdSensorMethod pd_sensor_6ft_met;

__END_DECLS

#endif /* __PD_SENSOR_6FT_H__ */
