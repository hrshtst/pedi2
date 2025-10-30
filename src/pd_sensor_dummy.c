#include <pedi2/pd_sensor.h>
#include <zeo/zeo_vec3d.h>

static void _pdSensorDummyDestroy(pdSensor *sensor)
{
  zFree( sensor->prp );
  pdSensorDefaultDestroy( sensor );
}

static zVec _pdSensorDummyProcess(pdSensor *sensor, double dt)
{
  pdSensorDefaultProcess( sensor, dt );
  return pdSensorData( sensor );
}

void _pdSensorDummyFrameUpdate(pdSensor *sensor, zFrame3D *frame)
{
  pdSensorDefaultFrameUpdate( sensor, frame );
}

#define PD_SENSOR_DUMMY_SIZE 0
typedef struct{
  char linkname[BUFSIZ];
  zFrame3D linkframe;
} _pdSensorDummyPrp;

static void *_pdSensorDummyLinkFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zStrCopy( ((_pdSensorDummyPrp*)arg)->linkname, ZTKVal(ztk), BUFSIZ ) ? obj : NULL;
}
static void *_pdSensorDummyFrameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DFromZTK( &((_pdSensorDummyPrp*)arg)->linkframe, ztk );
  return obj;
}

static bool _pdSensorDummyLinkFPrintZTK(FILE *fp, int i, void *obj){
  if( pdSensorLinkName((pdSensor*)obj)[0] == '\0' ) return false;
  fprintf( fp, "%s\n", pdSensorLinkName((pdSensor*)obj) );
  return true;
}
static bool _pdSensorDummyFrameFPrintZTK(FILE *fp, int i, void *obj){
  zFrame3DFPrint( fp, pdSensorLinkFrame((pdSensor*)obj) );
  return true;
}

static const ZTKPrp __ztk_prp_pdsensor_dummy[] = {
  { ZTK_KEY_PEDI2_SENSOR_LINK,     1, _pdSensorDummyLinkFromZTK,     _pdSensorDummyLinkFPrintZTK     },
  { ZTK_KEY_PEDI2_SENSOR_FRAME,    1, _pdSensorDummyFrameFromZTK,    _pdSensorDummyFrameFPrintZTK    },
};

static pdSensor *_pdSensorDummyFromZTK(pdSensor *sensor, pdFilterArray *filterarray, ZTK *ztk)
{
  _pdSensorDummyPrp prp;

  zFrame3DIdent( &prp.linkframe );
  if( !_ZTKEvalKey( sensor, &prp, ztk, __ztk_prp_pdsensor_dummy ) ) return NULL;
  return pdSensorDummyCreate( sensor, prp.linkname, &prp.linkframe ) ? sensor : NULL;
}

static void _pdSensorDummyFPrintZTK(FILE *fp, pdSensor *sensor)
{
  _ZTKPrpKeyFPrint( fp, sensor, __ztk_prp_pdsensor_dummy );
}

pdSensorCom pd_sensor_dummy_com = {
  .typestr = "dummy",
  ._destroy = _pdSensorDummyDestroy,
  ._process = _pdSensorDummyProcess,
  ._frameupdate = _pdSensorDummyFrameUpdate,
  ._fromZTK = _pdSensorDummyFromZTK,
  ._fprintZTK = _pdSensorDummyFPrintZTK,
};

pdSensor *pdSensorDummyCreate(pdSensor *sensor, const char linkname[], zFrame3D *linkframe)
{
  pdSensorInit( sensor );
  pdSensorSize( sensor ) = PD_SENSOR_DUMMY_SIZE;
  zStrCopy( pdSensorLinkName(sensor), linkname, BUFSIZ );
  zFrame3DCopy( linkframe, pdSensorLinkFrame(sensor) );
  sensor->com = &pd_sensor_dummy_com;
  return sensor;
}
