#ifndef __PD_EST_ZMP_H__
#define __PD_EST_ZMP_H__

#include <pedi2/pd_filter.h>
#include <pedi2/pd_sensor.h>

__BEGIN_DECLS

zArrayClass( pdSensorPtrArray, pdSensor* );

typedef struct{
  Z_NAMED_CLASS;
  pdFilterArray _farray;
  pdSensorArray _sarray;

  pdSensorPtrArray _lfsensor;
  pdSensorPtrArray _rfsensor;

  zVec3D estforce;
  zVec3D estzmp;
} pdEstZMP;

/* c'tor and d'tor */
__EXPORT void pdEstZMPInit(pdEstZMP *e);
__EXPORT void pdEstZMPDestroy(pdEstZMP *e);

/* methods to get parameters */
#define pdEstZMPFilterArray(e) ( &(e)->_farray )
#define pdEstZMPSensorArray(e) ( &(e)->_sarray )
#define pdEstZMPEstZMPVec(e)   ( &(e)->estzmp )
#define pdEstZMPEstZMPX(e)     zVec3DElem( pdEstZMPEstZMPVec(e), zX )
#define pdEstZMPEstZMPY(e)     zVec3DElem( pdEstZMPEstZMPVec(e), zY )
#define pdEstZMPEstZMPZ(e)     zVec3DElem( pdEstZMPEstZMPVec(e), zZ )
#define pdEstZMPEstForceVec(e) ( &(e)->estforce )
#define pdEstZMPEstForceX(e)   zVec3DElem( pdEstZMPEstForceVec(e), zX )
#define pdEstZMPEstForceY(e)   zVec3DElem( pdEstZMPEstForceVec(e), zY )
#define pdEstZMPEstForceZ(e)   zVec3DElem( pdEstZMPEstForceVec(e), zZ )

/* methods to set parameters */
__EXPORT pdSensor *pdEstZMPFindSensor(pdEstZMP *e, const char *name);
__EXPORT void pdEstZMPSetData(pdEstZMP *e, const char *name, zVec data);

__EXPORT zVec3D *pdEstZMPCalcFootForce(pdEstZMP *e, pdSensorPtrArray *s, zVec3D *f);
__EXPORT zVec3D *pdEstZMPCalcForce(pdEstZMP *e, zVec3D *f);
__EXPORT zVec3D *pdEstZMPCalcFootZMP(pdEstZMP *e, pdSensorPtrArray *s, double pz, zVec3D *zmp, double *fz);
__EXPORT zVec3D *pdEstZMPCalcZMP(pdEstZMP *e, double pz, zVec3D *zmp);

__EXPORT bool pdEstZMPConfFRead(FILE *fp, pdEstZMP *e);
__EXPORT bool pdEstZMPConfReadFile(pdEstZMP *e, const char *filename);

__END_DECLS

#endif /* __PD_EST_ZMP_H__ */
