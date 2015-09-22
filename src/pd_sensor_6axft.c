#include <pedi2/pd_sensor.h>

void pdSensorDestroy6AxisFT(pdSensor *sensor)
{
  pdSensorDestroyDefault( sensor );
}

zVec pdSensorProcess6AxisFT(pdSensor *sensor, double dt)
{
  return pdSensorProcessDefault( sensor, dt );
}

#define PD_SENSOR_6AXIS_SIZE 6
typedef struct{
  zFrame3D frame;
  pdFilterArray *srcfarr;
  pdFilterArray farr;
  int nf;
} _pdSensor6AxisFTParam;

static bool __pdSensorFRead6AxisFT(FILE *fp, _pdSensor6AxisFTParam *prm, bool *success);
static bool _pdSensorFRead6AxisFT(FILE *fp, void *prm, char *buf, bool *success);

bool __pdSensorFRead6AxisFT(FILE *fp, _pdSensor6AxisFTParam *prm, bool *success)
{
  char name[BUFSIZ];
  pdFilter *src;

  if( !( zFToken( fp, name, BUFSIZ ) ) )
    *success = false;
  src = pdFilterArrayNameFind( prm->srcfarr, name );
  pdFilterClone( src, zArrayElem( &prm->farr, prm->nf ) );
  return true;
}

bool _pdSensorFRead6AxisFT(FILE *fp, void *prm, char *buf, bool *success)
{
  register int i;

  if( strcmp( buf, "frame" ) == 0 ){
    zFrame3DFRead( fp, &((_pdSensor6AxisFTParam *)prm)->frame );
  } else
  if( strcmp( buf, "filter" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 0;
    for( i=0; i<PD_SENSOR_6AXIS_SIZE; i++ ){
      __pdSensorFRead6AxisFT( fp, prm, success );
      ((_pdSensor6AxisFTParam *)prm)->nf++;
    }
  } else
  if( strcmp( buf, "filterfx" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 0;
    __pdSensorFRead6AxisFT( fp, prm, success );
  } else
  if( strcmp( buf, "filterfy" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 1;
    __pdSensorFRead6AxisFT( fp, prm, success );
  } else
  if( strcmp( buf, "filterfz" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 2;
    __pdSensorFRead6AxisFT( fp, prm, success );
  } else
  if( strcmp( buf, "filtertx" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 3;
    __pdSensorFRead6AxisFT( fp, prm, success );
  } else
  if( strcmp( buf, "filterty" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 4;
    __pdSensorFRead6AxisFT( fp, prm, success );
  } else
  if( strcmp( buf, "filtertz" ) == 0 ){
    ((_pdSensor6AxisFTParam *)prm)->nf = 5;
    __pdSensorFRead6AxisFT( fp, prm, success );
  } else
    return false;
  return true;
}

pdSensor *pdSensorFRead6AxisFT(FILE *fp, pdSensor *sensor, pdFilterArray *srcfarr)
{
  _pdSensor6AxisFTParam prm;

  zFrame3DIdent( &prm.frame );
  prm.srcfarr = srcfarr;
  if( !pdFilterArrayAlloc( &prm.farr, PD_SENSOR_6AXIS_SIZE ) )
    return NULL;
  prm.nf = 0;
  zFieldFRead( fp, _pdSensorFRead6AxisFT, &prm );
  return pdSensorCreate6AxisFT( sensor, &prm.frame, &prm.farr ) ? sensor : NULL;
}

pdSensorMethod pd_sensor_6axisft_met = {
  type: "6axisft",
  destroy: pdSensorDestroy6AxisFT,
  process: pdSensorProcess6AxisFT,
  fread: pdSensorFRead6AxisFT,
};

bool pdSensorCreate6AxisFT(pdSensor *sensor, zFrame3D *frame, pdFilterArray *arr)
{
  pdSensorInit( sensor );
  pdSensorSize( sensor ) = PD_SENSOR_6AXIS_SIZE;
  pdSensorInput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  pdSensorOutput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  zFrame3DSetPos( pdSensorFrame(sensor), zFrame3DPos(frame) );
  zFrame3DSetAtt( pdSensorFrame(sensor), zFrame3DAtt(frame) );
  if( zArrayNum(arr) == PD_SENSOR_6AXIS_SIZE ){
    zArraySetNum( pdSensorFilterArray(sensor), PD_SENSOR_6AXIS_SIZE );
    zArraySetBuf( pdSensorFilterArray(sensor), zArrayBuf(arr) );
  } else {
    ZRUNERROR( "Filter array size is not matched: %d", zArrayNum(arr) );
    return false;
  }
  sensor->_met = &pd_sensor_6axisft_met;
  return true;
}
