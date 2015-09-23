#include <pedi2/pd_sensor.h>

void pdSensorDestroy6FT(pdSensor *sensor)
{
  pdSensorDestroyDefault( sensor );
}

zVec pdSensorProcess6FT(pdSensor *sensor, double dt)
{
  return pdSensorProcessDefault( sensor, dt );
}

#define PD_SENSOR_6FT_SIZE 6
typedef struct{
  zFrame3D frame;
  pdFilterArray *srcfarr;
  pdFilterArray farr;
  int nf;
} _pdSensor6FTParam;

static bool __pdSensorFRead6FT(FILE *fp, _pdSensor6FTParam *prm, bool *success);
static bool _pdSensorFRead6FT(FILE *fp, void *prm, char *buf, bool *success);

bool __pdSensorFRead6FT(FILE *fp, _pdSensor6FTParam *prm, bool *success)
{
  char name[BUFSIZ];
  pdFilter *src;

  if( !( zFToken( fp, name, BUFSIZ ) ) )
    *success = false;
  src = pdFilterArrayNameFind( prm->srcfarr, name );
  pdFilterClone( src, zArrayElem( &prm->farr, prm->nf ) );
  return true;
}

bool _pdSensorFRead6FT(FILE *fp, void *prm, char *buf, bool *success)
{
  register int i;

  if( strcmp( buf, "frame" ) == 0 ){
    zFrame3DFRead( fp, &((_pdSensor6FTParam *)prm)->frame );
  } else
  if( strcmp( buf, "filter" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 0;
    for( i=0; i<PD_SENSOR_6FT_SIZE; i++ ){
      __pdSensorFRead6FT( fp, prm, success );
      ((_pdSensor6FTParam *)prm)->nf++;
    }
  } else
  if( strcmp( buf, "filterfx" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 0;
    __pdSensorFRead6FT( fp, prm, success );
  } else
  if( strcmp( buf, "filterfy" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 1;
    __pdSensorFRead6FT( fp, prm, success );
  } else
  if( strcmp( buf, "filterfz" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 2;
    __pdSensorFRead6FT( fp, prm, success );
  } else
  if( strcmp( buf, "filtertx" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 3;
    __pdSensorFRead6FT( fp, prm, success );
  } else
  if( strcmp( buf, "filterty" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 4;
    __pdSensorFRead6FT( fp, prm, success );
  } else
  if( strcmp( buf, "filtertz" ) == 0 ){
    ((_pdSensor6FTParam *)prm)->nf = 5;
    __pdSensorFRead6FT( fp, prm, success );
  } else
    return false;
  return true;
}

pdSensor *pdSensorFRead6FT(FILE *fp, pdSensor *sensor, pdFilterArray *srcfarr)
{
  _pdSensor6FTParam prm;

  zFrame3DIdent( &prm.frame );
  prm.srcfarr = srcfarr;
  if( !pdFilterArrayAlloc( &prm.farr, PD_SENSOR_6FT_SIZE ) )
    return NULL;
  prm.nf = 0;
  zFieldFRead( fp, _pdSensorFRead6FT, &prm );
  return pdSensorCreate6FT( sensor, &prm.frame, &prm.farr ) ? sensor : NULL;
}

pdSensorMethod pd_sensor_6ft_met = {
  type: "6ft",
  destroy: pdSensorDestroy6FT,
  process: pdSensorProcess6FT,
  fread: pdSensorFRead6FT,
};

bool pdSensorCreate6FT(pdSensor *sensor, zFrame3D *frame, pdFilterArray *arr)
{
  pdSensorInit( sensor );
  pdSensorSize( sensor ) = PD_SENSOR_6FT_SIZE;
  pdSensorInput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  pdSensorOutput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  zFrame3DSetPos( pdSensorFrame(sensor), zFrame3DPos(frame) );
  zFrame3DSetAtt( pdSensorFrame(sensor), zFrame3DAtt(frame) );
  if( zArrayNum(arr) == PD_SENSOR_6FT_SIZE ){
    zArraySetNum( pdSensorFilterArray(sensor), PD_SENSOR_6FT_SIZE );
    zArraySetBuf( pdSensorFilterArray(sensor), zArrayBuf(arr) );
  } else {
    ZRUNERROR( "Filter array size is not matched: %d", zArrayNum(arr) );
    return false;
  }
  sensor->_met = &pd_sensor_6ft_met;
  return true;
}
