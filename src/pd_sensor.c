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

static pdSensorMethod *_pdSensorMethodByStr(char str[]);

pdSensorMethod *_pdSensorMethodByStr(char str[])
{
  static pdSensorMethod *met_array[] = {
    &pd_sensor_6axisft_met,
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
} _pdSensorParam;

bool _pdSensorFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "type" ) == 0 ){
    if( !( ((_pdSensorParam *)instance)->met = _pdSensorMethodByStr( zFToken(fp,buf,BUFSIZ) ) ) )
      *success = false;
  } else if( strcmp( buf, "name" )  == 0 ){
    if( !zFToken( fp, ((_pdSensorParam *)instance)->name, BUFSIZ ) )
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
    return sensor;
  }
  return NULL;
}
