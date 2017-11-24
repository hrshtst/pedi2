#include <pedi2/pd_sensor.h>

typedef struct{
  zVec3D f;
  zVec3D tau;
} _pdSensor6FT;

void pdSensorDestroy6FT(pdSensor *sensor)
{
  zFree( sensor->_prm );
  pdSensorDestroyDefault( sensor );
}

zVec pdSensorProcess6FT(pdSensor *sensor, double dt)
{
  _pdSensor6FT *ft;

  ft = sensor->_prm;
  pdSensorProcessDefault( sensor, dt );
  zVec3DCreate( &ft->f,
                pdSensorOutputVal(sensor,0),
                pdSensorOutputVal(sensor,1),
                pdSensorOutputVal(sensor,2) );
  zVec3DCreate( &ft->tau,
                pdSensorOutputVal(sensor,3),
                pdSensorOutputVal(sensor,4),
                pdSensorOutputVal(sensor,5) );
  return pdSensorOutput( sensor );
}

void pdSensorFrameUpdate6FT(pdSensor *sensor, zFrame3D *frame)
{
  pdSensorFrameUpdateDefault( sensor, frame );
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
  "6ft",
  pdSensorDestroy6FT,
  pdSensorProcess6FT,
  pdSensorFrameUpdate6FT,
  pdSensorFRead6FT,
};

bool pdSensorCreate6FT(pdSensor *sensor, zFrame3D *frame, pdFilterArray *arr)
{
  _pdSensor6FT *ft;

  if( !( ft = zAlloc( _pdSensor6FT, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  pdSensorInit( sensor );
  pdSensorSize( sensor ) = PD_SENSOR_6FT_SIZE;
  pdSensorInput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  pdSensorOutput( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  zFrame3DSetPos( pdSensorLinkFrame(sensor), zFrame3DPos(frame) );
  zFrame3DSetAtt( pdSensorLinkFrame(sensor), zFrame3DAtt(frame) );
  if( zArrayNum(arr) == PD_SENSOR_6FT_SIZE ){
    zArraySetNum( pdSensorFilterArray(sensor), PD_SENSOR_6FT_SIZE );
    zArraySetBuf( pdSensorFilterArray(sensor), zArrayBuf(arr) );
  } else {
    ZRUNERROR( "Filter array size is not matched: %d", zArrayNum(arr) );
    return false;
  }
  sensor->_prm = ft;
  sensor->_met = &pd_sensor_6ft_met;
  return true;
}

zVec3D *pdSensor6FTGetF(pdSensor *sensor, zVec3D *f)
{
  _pdSensor6FT *ft;

  ft = sensor->_prm;
  zVec3DCopy( &ft->f, f );
  return f;
}

zVec3D *pdSensor6FTGetT(pdSensor *sensor, zVec3D *tau)
{
  _pdSensor6FT *ft;

  ft = sensor->_prm;
  zVec3DCopy( &ft->tau, tau );
  return tau;
}

zVec3D *pdSensor6FTGetWldF(pdSensor *sensor, zVec3D *f)
{
  _pdSensor6FT *ft;

  ft = sensor->_prm;
  zMulMatVec3D( pdSensorWldAtt(sensor), &ft->f, f );
  return f;
}

zVec3D *pdSensor6FTGetWldT(pdSensor *sensor, zVec3D *tau)
{
  _pdSensor6FT *ft;

  ft = sensor->_prm;
  zMulMatVec3D( pdSensorWldAtt(sensor), &ft->tau, tau );
  return tau;
}
