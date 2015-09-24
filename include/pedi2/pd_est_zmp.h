#ifndef __PD_EST_ZMP_H__
#define __PD_EST_ZMP_H__

#include <pedi2/pd_filter.h>
#include <pedi2/pd_sensor.h>

__BEGIN_DECLS

zArrayClass( pdSensorPtrArray, pdSensor* );

typedef struct{
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
#define pdEstZMPEstZMP(e)   ( &(e)->estzmp )
#define pdEstZMPEstZMPX(e)     zVec3DElem( pdEstZMPEstZMP(e), zX )
#define pdEstZMPEstZMPY(e)     zVec3DElem( pdEstZMPEstZMP(e), zY )
#define pdEstZMPEstZMPZ(e)     zVec3DElem( pdEstZMPEstZMP(e), zZ )
#define pdEstZMPEstForce(e) ( &(e)->estforce )
#define pdEstZMPEstForceX(e)   zVec3DElem( pdEstZMPEstForce(e), zX )
#define pdEstZMPEstForceY(e)   zVec3DElem( pdEstZMPEstForce(e), zY )
#define pdEstZMPEstForceZ(e)   zVec3DElem( pdEstZMPEstForce(e), zZ )

/* methods to set parameters */
__EXPORT pdSensor *pdEstZMPNameFindSensor(pdEstZMP *e, const char *name);
__EXPORT void pdEstZMPSetData(pdEstZMP *e, const char *name, zVec data);

__EXPORT zVec3D *pdEstZMPCalcFootForce(pdEstZMP *e, pdSensorPtrArray *s, zVec3D *f);
__EXPORT zVec3D *pdEstZMPCalcForce(pdEstZMP *e, zVec3D *f);
__EXPORT zVec3D *pdEstZMPCalcFootZMP(pdEstZMP *e, pdSensorPtrArray *s, double pz, zVec3D *zmp, double *fz);
__EXPORT zVec3D *pdEstZMPCalcZMP(pdEstZMP *e, double pz, zVec3D *zmp);

__EXPORT void pdEstZMPUpdate(pdEstZMP *e, zFrame3D *lfframe, zFrame3D *rfframe, double pz, double dt);

__EXPORT void pdEstZMPFWrite(FILE *fp, pdEstZMP *e);
#define pdEstZMPWrite(e) pdEstZMPFWrite( stdout, e )
__EXPORT void pdEstZMPDataFWrite(FILE *fp, pdEstZMP *e);
#define pdEstZMPDataWrite(e) pdEstZMPDataFWrite( stdout, e )

__EXPORT bool pdEstZMPConfFRead(FILE *fp, pdEstZMP *e);
__EXPORT bool pdEstZMPConfReadFile(pdEstZMP *e, const char *filename);

__END_DECLS

#endif /* __PD_EST_ZMP_H__ */
