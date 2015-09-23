#include <pedi2/pd_sensor.h>

void pdSensorDestroyDefault(pdSensor *sensor)
{
  zNameDestroy( sensor );
  pdFilterArrayDestroy( pdSensorFilterArray( sensor ) );
  zVecFree( pdSensorInput( sensor ) );
  zVecFree( pdSensorOutput( sensor ) );
  zFree( sensor->_prm );
  pdSensorInit( sensor );
}

zVec pdSensorProcessDefault(pdSensor *sensor, double dt)
{
  register int i;

  for( i=0; i<pdSensorSize(sensor); i++ )
    pdFilterArrayInput( pdSensorFilterArray(sensor), i ) = pdSensorInputVal( sensor, i );
  pdFilterArrayUpdate( pdSensorFilterArray(sensor), dt );
  for( i=0; i<pdSensorSize(sensor); i++ )
    pdSensorOutputVal( sensor, i ) = pdFilterArrayOutput( pdSensorFilterArray(sensor), i );
  return pdSensorOutput( sensor );
}

void pdSensorFrameUpdateDefault(pdSensor *sensor, zFrame3D *frame)
{
  zFrame3DCascade( frame, pdSensorLinkFrame(sensor), pdSensorWldFrame(sensor) );
}

static pdSensorMethod *_pdSensorMethodByStr(char str[]);

pdSensorMethod *_pdSensorMethodByStr(char str[])
{
  static pdSensorMethod *met_array[] = {
    &pd_sensor_6ft_met,
    NULL,
  };
  register int i;

  for( i=0; met_array[i]; i++ ){
    if( strcmp( met_array[i]->type, str ) == 0 ) return met_array[i];
  }
  ZRUNERROR( "cannot find a sensor type %s", str );
  return NULL;
}

typedef struct{
  pdSensorMethod *met;
  char name[BUFSIZ];
  char linkname[BUFSIZ];
} _pdSensorParam;

bool _pdSensorFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "type" ) == 0 ){
    if( !( ((_pdSensorParam *)instance)->met = _pdSensorMethodByStr( zFToken(fp,buf,BUFSIZ) ) ) )
      *success = false;
  } else
  if( strcmp( buf, "name" )  == 0 ){
    if( !zFToken( fp, ((_pdSensorParam *)instance)->name, BUFSIZ ) )
      *success = false;
  } else
  if( strcmp( buf, "link" )  == 0 ){
    if( !zFToken( fp, ((_pdSensorParam *)instance)->linkname, BUFSIZ ) )
      *success = false;
  } else
    return false;
  return true;
}

pdSensor *pdSensorFRead(FILE *fp, pdSensor *sensor, pdFilterArray *srcfarr)
{
  _pdSensorParam prm;
  int cur;

  prm.met = NULL;
  prm.name[0] = '\0';
  prm.linkname[0] = '\0';
  cur = ftell( fp );
  zFieldFRead( fp, _pdSensorFRead, &prm );
  if( !prm.met ){
    ZRUNERROR( "type not specified" );
    return NULL;
  }
  fseek( fp, cur, SEEK_SET );
  if( prm.met->fread( fp, sensor, srcfarr ) ){
    if( !zNameSet( sensor, prm.name ) ){
      ZALLOCERROR();
      return NULL;
    }
    zStrCopy( pdSensorLinkName(sensor), prm.linkname, BUFSIZ );
    return sensor;
  }
  return NULL;
}

static bool _pdSensorFAlloc(FILE *fp, pdSensorArray *arr);

bool pdSensorArrayAlloc(pdSensorArray *arr, int n)
{
  zArrayAlloc( arr, pdSensor, n );
  if( !zArrayBuf(arr) ){
    ZALLOCERROR();
    return false;
  }
  return true;
}

void pdSensorArrayDestroy(pdSensorArray *arr)
{
  register int i;

  for( i=0; i<zArrayNum(arr); i++ )
    pdSensorDestroy( zArrayElem(arr,i) );
  zArrayFree( arr );
}

bool _pdSensorFAlloc(FILE *fp, pdSensorArray *arr)
{
  int n;

  n = zFCountTag( fp, PD_SENSOR_TAG );
  return pdSensorArrayAlloc( arr, n );
}

pdSensor *pdSensorArrayNameFind(pdSensorArray *arr, const char *name)
{
  pdSensor *sensor = NULL;

  zArrayFindName( arr, name, sensor );
  if( !sensor ){
    ZRUNWARN( "invalid sensor name %s", name );
    return NULL;
  }
  return sensor;
}

void pdSensorArrayProcess(pdSensorArray *arr, double dt)
{
  register uint i;

  for( i=0; i<zArrayNum(arr); i++ )
    pdSensorProcess( zArrayElem(arr,i), dt );
}

typedef struct{
  pdFilterArray *srcfarr;
  pdSensorArray *arr;
  int count;
} _pdSensorArrayParam;

bool _pdSensorArrayFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _pdSensorArrayParam *prm;

  prm = instance;
  if( strcmp( buf, PD_SENSOR_TAG ) == 0){
    if( !pdSensorFRead( fp, zArrayElem(prm->arr,prm->count++), prm->srcfarr ) ){
      *success = false;
      return false;
    }
  } else
    return false;
  return true;
}

bool pdSensorArrayFRead(FILE *fp, pdSensorArray *arr, pdFilterArray *srcfarr)
{
  _pdSensorArrayParam prm;

  zArrayInit( arr );
  if( !_pdSensorFAlloc( fp, arr ) ) return false;
  rewind( fp );
  prm.count = 0;
  prm.arr = arr;
  prm.srcfarr = srcfarr;
  return zTagFRead( fp, _pdSensorArrayFRead, &prm );
}
