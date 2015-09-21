#ifndef __PD_SENSOR_H__
#define __PD_SENSOR_H__

#include <pedi2/pd_filter.h>

__BEGIN_DECLS

struct _pdSensor;

typedef struct{
  const char *type;
  void (*destroy)(struct _pdSensor*);
  void (*process)(struct _pdSensor*, double dt);
  struct _pdSensor *(*fread)(FILE *fp, struct _pdSensor*);
} pdSensorMethod;

typedef struct _pdSensor{
  Z_NAMED_CLASS;
  int size;
  zVec input;
  zVec output;
  void *_prm;
  pdSensorMethod *_met;
} pdSensor;

#define pdSensorSize(s)   ( (s)->size )
#define pdSensorInput(s)  ( (s)->input )
#define pdSensorOutput(s) ( (s)->output )

#define pdSensorInit(s) do{\
  zNameSet( s, NULL );\
  pdSensorSize(s) = 0;\
  pdSensorInput(s) = NULL;\
  pdSensorOutput(s) = NULL;\
  (s)->_prm = NULL;\
  (s)->_met = NULL;\
} while(0)

#define pdSensorDestroy(s)  (s)->_met->destroy( s )
#define pdSensorUpdate(s,h) (s)->_met->update( s, h )

__EXPORT void pdSensorDestroyDefault(pdSensor *sensor);

#define PD_SENSOR_TAG "sensor"
__EXPORT pdSensor *pdSensorFRead(FILE *fp, pdSensor *sensor);


__END_DECLS

#endif /* __PD_SENSOR_H__ */
