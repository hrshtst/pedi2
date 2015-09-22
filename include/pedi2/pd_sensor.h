#ifndef __PD_SENSOR_H__
#define __PD_SENSOR_H__

#include <zeo/zeo_frame.h>
#include <pedi2/pd_filter.h>

__BEGIN_DECLS

struct _pdSensor;

typedef struct{
  const char *type;
  void (*destroy)(struct _pdSensor*);
  zVec (*process)(struct _pdSensor*, double dt);
  struct _pdSensor *(*fread)(FILE *fp, struct _pdSensor*);
} pdSensorMethod;

typedef struct _pdSensor{
  Z_NAMED_CLASS;
  int size;
  zVec input;
  zVec output;
  zFrame3D frame;
  pdFilterArray arr;
  void *_prm;
  pdSensorMethod *_met;
} pdSensor;

#define pdSensorSize(s)        ( (s)->size )
#define pdSensorInput(s)       ( (s)->input )
#define pdSensorOutput(s)      ( (s)->output )
#define pdSensorFrame(s)       ( &(s)->frame )
#define pdSensorFilterArray(s) ( &(s)->arr )

#define pdSensorInputVal(s,i)   zVecElem( pdSensorInput(s), i )
#define pdSensorOutputVal(s,i)  zVecElem( pdSensorOutput(s), i )
#define pdSensorFilterElem(s,i) zArrayElem( pdSensorFilterArray(s), i )

#define pdSensorInit(s) do{\
  zNameSet( s, NULL );\
  pdSensorSize(s) = 0;\
  pdSensorInput(s) = NULL;\
  pdSensorOutput(s) = NULL;\
  zFrame3DIdent( pdSensorFrame(s) );\
  zArrayInit( pdSensorFilterArray(s) );\
  (s)->_prm = NULL;\
  (s)->_met = NULL;\
} while(0)

#define pdSensorDestroy(s)   (s)->_met->destroy( s )
#define pdSensorProcess(s,h) (s)->_met->process( s, h )

__EXPORT void pdSensorDestroyDefault(pdSensor *sensor);

#define PD_SENSOR_TAG "sensor"
__EXPORT pdSensor *pdSensorFRead(FILE *fp, pdSensor *sensor);


__END_DECLS

#include <pedi2/pd_sensor_6axft.h> /* six-axis force/torque sensor */

#endif /* __PD_SENSOR_H__ */
