#ifndef __PD_SENSOR_H__
#define __PD_SENSOR_H__

#include <zeo/zeo_frame.h>
#include <pedi2/pd_filter.h>
#include <pedi2/pd_exportdecl.h>

__BEGIN_DECLS

struct _pdSensor;

typedef struct{
  const char *type;
  void (*destroy)(struct _pdSensor*);
  zVec (*process)(struct _pdSensor*, double dt);
  void (*frameupdate)(struct _pdSensor*,zFrame3D*);
  struct _pdSensor *(*fread)(FILE *fp, struct _pdSensor*, pdFilterArray*);
} pdSensorMethod;

typedef struct _pdSensor{
  Z_NAMED_CLASS
  int size;
  zVec input;
  zVec output;
  zFrame3D linkframe;  /* transformation from sensor to link frame */
  zFrame3D wldframe;   /* transformation from sensor to world frame */
  pdFilterArray arr;
  char linkname[BUFSIZ];
  void *_prm;
  pdSensorMethod *_met;
} pdSensor;

#define pdSensorSize(s)        ( (s)->size )
#define pdSensorInput(s)       ( (s)->input )
#define pdSensorOutput(s)      ( (s)->output )
#define pdSensorLinkFrame(s)   ( &(s)->linkframe )
#define pdSensorPos(s)         zFrame3DPos( pdSensorLinkFrame(s) )
#define pdSensorAtt(s)         zFrame3DAtt( pdSensorLinkFrame(s) )
#define pdSensorWldFrame(s)    ( &(s)->wldframe )
#define pdSensorWldPos(s)      zFrame3DPos( pdSensorWldFrame(s) )
#define pdSensorWldAtt(s)      zFrame3DAtt( pdSensorWldFrame(s) )
#define pdSensorFilterArray(s) ( &(s)->arr )
#define pdSensorLinkName(s)    ( (s)->linkname )

#define pdSensorInputVal(s,i)   zVecElem( pdSensorInput(s), i )
#define pdSensorOutputVal(s,i)  zVecElem( pdSensorOutput(s), i )
#define pdSensorFilterElem(s,i) zArrayElem( pdSensorFilterArray(s), i )

#define pdSensorSetInput(s,i)  zVecCopy( i, pdSensorInput(s) )
#define pdSensorGetOutput(s,o) zVecCopy( pdSensorOutput(s), o )

#define pdSensorInit(s) do{\
  zNameSet( s, NULL );\
  pdSensorSize(s) = 0;\
  pdSensorInput(s) = NULL;\
  pdSensorOutput(s) = NULL;\
  zFrame3DIdent( pdSensorLinkFrame(s) );\
  zFrame3DIdent( pdSensorWldFrame(s) );\
  zArrayInit( pdSensorFilterArray(s) );\
  (s)->linkname[0] = '\0';\
  (s)->_prm = NULL;\
  (s)->_met = NULL;\
} while(0)

#define pdSensorDestroy(s)       (s)->_met->destroy( s )
#define pdSensorProcess(s,h)     (s)->_met->process( s, h )
#define pdSensorFrameUpdate(s,f) (s)->_met->frameupdate( s, f )

__EXPORT void pdSensorDestroyDefault(pdSensor *sensor);
__EXPORT zVec pdSensorProcessDefault(pdSensor *sensor, double dt);
__EXPORT void pdSensorFrameUpdateDefault(pdSensor *sensor, zFrame3D *frame);

#define PD_SENSOR_TAG "sensor"
__EXPORT pdSensor *pdSensorFRead(FILE *fp, pdSensor *sensor, pdFilterArray *srcfarr);

zArrayClass( pdSensorArray, pdSensor );

__EXPORT bool pdSensorArrayAlloc(pdSensorArray *arr, int n);
__EXPORT void pdSensorArrayDestroy(pdSensorArray *arr);

__EXPORT pdSensor *pdSensorArrayNameFind(pdSensorArray *arr, const char *name);

__EXPORT void pdSensorArrayProcess(pdSensorArray *arr, double dt);

__EXPORT bool pdSensorArrayFRead(FILE *fp, pdSensorArray *arr, pdFilterArray *srcfarr);

__END_DECLS

#include <pedi2/pd_sensor_6ft.h> /* six-axis force/torque sensor */

#endif /* __PD_SENSOR_H__ */
