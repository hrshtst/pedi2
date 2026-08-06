#ifndef __PD_EST_ZMP_H__
#define __PD_EST_ZMP_H__

#include <pedi2/pd_filter.h>
#include <pedi2/pd_sensor.h>

__BEGIN_DECLS

ZEDA_DEF_LIST_CLASS( pdSensorList, pdSensorListCell, pdSensor* );

typedef struct{
  pdFilterArray _farray;
  pdSensorArray _sarray;

  pdSensorList _lfsensor;
  pdSensorList _rfsensor;

  zVec3D estforce;
  zVec3D estzmp;
} pdEstZmp;

/* c'tor and d'tor */
__PEDI2_EXPORT void pdEstZmpInit(pdEstZmp *e);
__PEDI2_EXPORT void pdEstZmpDestroy(pdEstZmp *e);

/* methods to get parameters */
#define pdEstZmpFilterArray(e) ( &(e)->_farray )
#define pdEstZmpSensorArray(e) ( &(e)->_sarray )
#define pdEstZmpEstZMP(e)      ( &(e)->estzmp )
#define pdEstZmpEstZMPX(e)     pdEstZmpEstZMP(e)->c.x
#define pdEstZmpEstZMPY(e)     pdEstZmpEstZMP(e)->c.y
#define pdEstZmpEstZMPZ(e)     pdEstZmpEstZMP(e)->c.z
#define pdEstZmpEstForce(e)    ( &(e)->estforce )
#define pdEstZmpEstForceX(e)   pdEstZmpEstForce(e)->c.x
#define pdEstZmpEstForceY(e)   pdEstZmpEstForce(e)->c.y
#define pdEstZmpEstForceZ(e)   pdEstZmpEstForce(e)->c.z

#define pdSensorListCellSensor(c)      (c)->data
#define pdSensorListCellSetSensor(c,s) ( pdSensorListCellSensor(c) = (s) )
#define pdSensorListCellInit(c) do{\
  zListCellInit( c );\
  pdSensorListCellSetSensor( c, NULL );\
} while(0)
__PEDI2_EXPORT pdSensorListCell *pdSensorListPush(pdSensorList *list, pdSensor *sensor);
__PEDI2_EXPORT pdSensor *pdSensorListPop(pdSensorList *list);
#define pdSensorListDestroy(list) zListDestroy( pdSensorListCell, list )

#define pdEstZmpSensorListPush(e,foot,s) pdSensorListPush( &(e)->_##foot##sensor, s )

/* methods to set parameters */
__PEDI2_EXPORT pdSensor *pdEstZmpNameFindSensor(pdEstZmp *e, const char *name);
__PEDI2_EXPORT void pdEstZmpSetData(pdEstZmp *e, const char *name, zVec data);

__PEDI2_EXPORT zVec3D *pdEstZmpCalcFootForce(pdEstZmp *e, pdSensorList *s, zVec3D *f);
__PEDI2_EXPORT zVec3D *pdEstZmpCalcForce(pdEstZmp *e, zVec3D *f);
__PEDI2_EXPORT zVec3D *pdEstZmpCalcFootZMP(pdEstZmp *e, pdSensorList *s, double pz, zVec3D *zmp, double *fz);
__PEDI2_EXPORT zVec3D *pdEstZmpCalcZMP(pdEstZmp *e, double pz, zVec3D *zmp);

__PEDI2_EXPORT void pdEstZmpUpdate(pdEstZmp *e, zFrame3D *lfframe, zFrame3D *rfframe, double pz, double dt);

__PEDI2_EXPORT const pdEstZmp *pdEstZmpValueFPrint(FILE *fp, const pdEstZmp *e);
#define pdEstZmpDataPrint(e) pdEstZmpValueFPrint( stdout, e )
__PEDI2_EXPORT const pdEstZmp *pdEstZmpValueNLFPrint(FILE *fp, const pdEstZmp *e);
#define pdEstZmpValueNLPrint(e) pdEstZmpValueNLFPrint( stdout, e )
__PEDI2_EXPORT const pdEstZmp *pdEstZmpFPrint(FILE *fp, const pdEstZmp *e);
#define pdEstZmpPrint(e) pdEstZmpFPrint( stdout, e )

#define ZTK_TAG_PEDI2_ESTIMATOR           "pedi2::estimator"

#define ZTK_KEY_PEDI2_ESTIMATOR_TYPE      "type"
#define ZTK_KEY_PEDI2_ESTIMATOR_LEFTFOOT  "leftfoot"
#define ZTK_KEY_PEDI2_ESTIMATOR_RIGHTFOOT "rightfoot"

__PEDI2_EXPORT pdEstZmp *pdEstZmpFromZTK(pdEstZmp *e, ZTK *ztk);
__PEDI2_EXPORT void pdEstZmpFPrintZTK(FILE *fp, pdEstZmp *e);

__PEDI2_EXPORT pdEstZmp *pdEstZmpReadZTK(pdEstZmp *e, const char filename[]);
__PEDI2_EXPORT bool pdEstZmpWriteZTK(pdEstZmp *e, const char filename[]);

__END_DECLS

#endif /* __PD_EST_ZMP_H__ */
