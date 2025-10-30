#ifndef __PD_SENSOR_H__
#define __PD_SENSOR_H__

#include <zeo/zeo_frame3d.h>
#include <pedi2/pd_filter.h>

__BEGIN_DECLS

struct _pdSensor;

typedef struct{
  const char *typestr;
  void (* _destroy)(struct _pdSensor*);
  zVec (* _process)(struct _pdSensor*, double);
  void (* _frameupdate)(struct _pdSensor*, zFrame3D*);
  struct _pdSensor *(* _fromZTK)(struct _pdSensor*, pdFilterArray*, ZTK*);
  void (* _fprintZTK)(FILE*, struct _pdSensor*);
} pdSensorCom;

typedef struct _pdSensor{
  Z_NAMED_CLASS;
  /* common properties */
  int size;              /* sensor size */
  char linkname[BUFSIZ]; /* attached link name */
  zFrame3D linkframe;    /* transform from sensor to attached link frame */
  /* common states */
  zVec rawdata;          /* raw data */
  zVec data;             /* processed data */
  pdFilterArray filters; /* filter array */
  zFrame3D wldframe;     /* transform from sensor to world frame */
  /* sensor specific properties */
  void *prp;
  /* common methods */
  pdSensorCom *com;
} pdSensor;

#define pdSensorSize(s)              ( (s)->size )
#define pdSensorLinkName(s)          ( (s)->linkname )
#define pdSensorLinkFrame(s)         ( &(s)->linkframe )
#define pdSensorLinkPos(s)           zFrame3DPos( pdSensorLinkFrame(s) )
#define pdSensorLinkAtt(s)           zFrame3DAtt( pdSensorLinkFrame(s) )
#define pdSensorRawData(s)           ( (s)->rawdata )
#define pdSensorRawDataVal(s,i)      zVecElem( pdSensorRawData(s), i )
#define pdSensorRawDataSetVal(s,i,v) zVecSetElemNC( pdSensorRawData(s), i, v )
#define pdSensorData(s)              ( (s)->data )
#define pdSensorDataVal(s,i)         zVecElem( pdSensorData(s), i )
#define pdSensorDataSetVal(s,i,v)    zVecSetElemNC( pdSensorData(s), i, v )
#define pdSensorFilters(s)           ( &(s)->filters )
#define pdSensorFilterElem(s,i)      zArrayElem( pdSensorFilters(s), i )
#define pdSensorFilterOutput(s,i)    pdFilterArrayOutput( pdSensorFilters(s), i )
#define pdSensorWldFrame(s)          ( &(s)->wldframe )
#define pdSensorWldPos(s)            zFrame3DPos( pdSensorWldFrame(s) )
#define pdSensorWldAtt(s)            zFrame3DAtt( pdSensorWldFrame(s) )

#define pdSensorSetRawData(s,i)      zVecCopy( i, pdSensorRawData(s) )
#define pdSensorGetData(s,o)         zVecCopy( pdSensorData(s), o )

#define pdSensorInit(s) do{\
  zNameSet( s, NULL );\
  pdSensorSize(s) = 0;\
  pdSensorRawData(s) = NULL;\
  pdSensorData(s) = NULL;\
  zFrame3DIdent( pdSensorLinkFrame(s) );\
  zFrame3DIdent( pdSensorWldFrame(s) );\
  zArrayInit( pdSensorFilters(s) );\
  (s)->linkname[0] = '\0';\
  (s)->prp = NULL;\
  (s)->com = NULL;\
} while(0)

#define pdSensorDestroy(s)       (s)->com->_destroy( s )
#define pdSensorProcess(s,h)     (s)->com->_process( s, h )
#define pdSensorFrameUpdate(s,f) (s)->com->_frameupdate( s, f )

__PEDI2_EXPORT void pdSensorDefaultDestroy(pdSensor *sensor);
__PEDI2_EXPORT zVec pdSensorDefaultProcess(pdSensor *sensor, double dt);
__PEDI2_EXPORT void pdSensorDefaultFrameUpdate(pdSensor *sensor, zFrame3D *frame);

#define ZTK_TAG_PEDI2_SENSOR          "pedi2::sensor"

#define ZTK_KEY_PEDI2_SENSOR_NAME     "name"
#define ZTK_KEY_PEDI2_SENSOR_TYPE     "type"
#define ZTK_KEY_PEDI2_SENSOR_LINK     "link"
#define ZTK_KEY_PEDI2_SENSOR_FRAME    "frame"
#define ZTK_KEY_PEDI2_SENSOR_FILTER   "filter"
#define ZTK_KEY_PEDI2_SENSOR_FILTERFX "filterfx"
#define ZTK_KEY_PEDI2_SENSOR_FILTERFY "filterfy"
#define ZTK_KEY_PEDI2_SENSOR_FILTERFZ "filterfz"
#define ZTK_KEY_PEDI2_SENSOR_FILTERTX "filtertx"
#define ZTK_KEY_PEDI2_SENSOR_FILTERTY "filterty"
#define ZTK_KEY_PEDI2_SENSOR_FILTERTZ "filtertz"

__PEDI2_EXPORT pdSensor *pdSensorFromZTK(pdSensor *sensor, pdFilterArray *filters, ZTK *ztk);
__PEDI2_EXPORT void pdSensorFPrintZTK(FILE* fp, pdSensor *sensor);

zArrayClass( pdSensorArray, pdSensor );

__PEDI2_EXPORT pdSensorArray *pdSensorArrayAlloc(pdSensorArray *arr, int size);
__PEDI2_EXPORT void pdSensorArrayDestroy(pdSensorArray *arr);

__PEDI2_EXPORT pdSensor *pdSensorArrayNameFind(pdSensorArray *arr, const char *name);

__PEDI2_EXPORT void pdSensorArrayProcess(pdSensorArray *arr, double dt);

__PEDI2_EXPORT pdSensorArray *pdSensorArrayFromZTK(pdSensorArray *arr, pdFilterArray *farray, ZTK *ztk);
__PEDI2_EXPORT void pdSensorArrayFPrintZTK(FILE *fp, pdSensorArray *arr);

__PEDI2_EXPORT pdSensorArray *pdSensorArrayReadZTK(pdSensorArray *arr, pdFilterArray *farray, char filename[]);
__PEDI2_EXPORT bool pdSensorArrayWriteZTK(pdSensorArray *arr, char filename[]);

__END_DECLS

#include <pedi2/pd_sensor_dummy.h> /* dummy sensor */
#include <pedi2/pd_sensor_6ft.h>   /* six-axis force/torque sensor */

__BEGIN_DECLS

/* add a handle to the following list when you create a new sensor class. */
#define PD_SENSOR_COM_ARRAY \
pdSensorCom *pd_sensor_com[] = { \
  &pd_sensor_dummy_com, \
  &pd_sensor_6ft_com, \
  NULL, \
}

__END_DECLS

#endif /* __PD_SENSOR_H__ */
