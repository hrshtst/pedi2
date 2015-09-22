#ifndef __PD_EST_ZMP_H__
#define __PD_EST_ZMP_H__

#include <pedi2/pd_filter.h>
#include <pedi2/pd_sensor.h>

__BEGIN_DECLS

typedef struct{
  Z_NAMED_CLASS;
  pdFilterArray _farray;
  pdSensorArray _sarray;

  pdSensor **_lfsensor;
  pdSensor **_rfsensor;
  int _lfsensor_num;
  int _rfsensor_num;

  zVec3D estforce;
  zVec3D estzmp;
} pdEstZMP;

/* c'tor and d'tor */
__EXPORT void pdEstZMPInit(pdEstZMP *e);
__EXPORT void pdEstZMPDestroy(pdEstZMP *e);

/* methods to get parameters */
#define pdEstZMPEstZMPVec(e)   ( &(e)->estzmp )
#define pdEstZMPEstZMPX(e)     zVec3DElem( pdEstZMPEstZMPVec(e), zX )
#define pdEstZMPEstZMPY(e)     zVec3DElem( pdEstZMPEstZMPVec(e), zY )
#define pdEstZMPEstZMPZ(e)     zVec3DElem( pdEstZMPEstZMPVec(e), zZ )
#define pdEstZMPEstForceVec(e) ( &(e)->estforce )
#define pdEstZMPEstForceX(e)   zVec3DElem( pdEstZMPEstForceVec(e), zX )
#define pdEstZMPEstForceY(e)   zVec3DElem( pdEstZMPEstForceVec(e), zY )
#define pdEstZMPEstForceZ(e)   zVec3DElem( pdEstZMPEstForceVec(e), zZ )

/* methods to set parameters */

__EXPORT bool pdEstZMPConfFRead(FILE *fp, pdEstZMP *e);
__EXPORT bool pdEstZMPConfReadFile(pdEstZMP *e, const char *filename);

__END_DECLS

#endif /* __PD_EST_ZMP_H__ */
