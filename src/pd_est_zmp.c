#include <pedi2/pd_est_zmp.h>

void pdEstZMPInit(pdEstZMP *e)
{
  zNameSet( e, NULL );
  e->_lfsensor = NULL;
  e->_rfsensor = NULL;
  e->_lfsensor_num = 0;
  e->_rfsensor_num = 0;
  zVec3DClear( &e->estforce );
  zVec3DClear( &e->estzmp );
}

void pdEstZMPDestroy(pdEstZMP *e)
{
  zNameDestroy( e );
  zFree( e->_lfsensor );
  zFree( e->_rfsensor );
  e->_lfsensor_num = 0;
  e->_rfsensor_num = 0;
  zVec3DClear( &e->estforce );
  zVec3DClear( &e->estzmp );
  pdFilterArrayDestroy( pdEstZMPFilterArray( e ) );
}

typedef struct {
  char name[BUFSIZ];
  pdSensorArray *sarray;
  pdSensor **lfsensor;
  pdSensor **rfsensor;
  int lfsensor_num;
  int rfsensor_num;
} _pdEstZMPParam;

int _pdEstZMPCountSensorName(FILE *fp, pdSensorArray *sarray)
{
  pdSensor *sp;
  char buf[BUFSIZ];
  int cur, cnt;

  cnt = 0;
  cur = ftell( fp );
  while( !feof(fp) ){
    if( !zFSkipDefaultComment( fp ) ) break;
    if( !zFToken( fp, buf, BUFSIZ ) ) break;
    zArrayFindName( sarray, buf, sp );
    if( !sp ) break;
    cnt++;
  }
  fseek( fp, cur, SEEK_SET );
  return cnt;
}

void _pdEstZMPConnectSensor(FILE *fp, pdSensor **sensor, int num, pdSensorArray *sarray)
{
  register int i;
  char buf[BUFSIZ];

  for( i=0; i<num; i++ ){
    if( !zFSkipDefaultComment( fp ) ) break;
    if( !zFToken( fp, buf, BUFSIZ ) ) break;
    sensor[i] = pdSensorArrayNameFind( sarray, buf );
  }
}

bool _pdEstZMPFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _pdEstZMPParam *prm;

  prm = instance;
  if( strcmp( buf, "name" ) == 0 ){
    if( strlen( zFToken( fp, prm->name, BUFSIZ ) ) >= BUFSIZ ){
      prm->name[BUFSIZ-1] = '\0';
      ZRUNWARN( "too long name, truncated to %s", buf );
    }
  } else
  if( strcmp( buf, "type" ) == 0 ){
    if( !zFToken( fp, buf, BUFSIZ ) )
      *success = false;
    if( strcmp( buf, "zmp" ) != 0 ){
      ZRUNERROR( "invalid estimator type %s", buf );
      *success = false;
    }
  } else
  if( strcmp( buf, "leftfoot" ) == 0 ){
    prm->lfsensor_num = _pdEstZMPCountSensorName( fp, prm->sarray );
    prm->lfsensor = zAlloc( pdSensor*, prm->lfsensor_num );
    if( !prm->lfsensor ){
      ZALLOCERROR();
      *success = false;
    }
    _pdEstZMPConnectSensor( fp, prm->lfsensor, prm->lfsensor_num, prm->sarray );
  } else
  if( strcmp( buf, "rightfoot" ) == 0 ){
    prm->rfsensor_num = _pdEstZMPCountSensorName( fp, prm->sarray );
    prm->rfsensor = zAlloc( pdSensor*, prm->rfsensor_num );
    if( !prm->rfsensor ){
      ZALLOCERROR();
      *success = false;
    }
    _pdEstZMPConnectSensor( fp, prm->rfsensor, prm->rfsensor_num, prm->sarray );
  } else
    return false;
  return true;
}

pdEstZMP *pdEstZMPFRead(FILE *fp, pdEstZMP *e)
{
  _pdEstZMPParam prm;

  prm.name[0] = '\0';
  prm.sarray = pdEstZMPSensorArray( e );
  prm.lfsensor = NULL;
  prm.rfsensor = NULL;
  prm.lfsensor_num = 0;
  prm.rfsensor_num = 0;
  if( !zFieldFRead( fp, _pdEstZMPFRead, &prm ) )
    return NULL;
  zNameSet( e, prm.name );
  e->_lfsensor_num = prm.lfsensor_num;
  e->_rfsensor_num = prm.rfsensor_num;
  e->_lfsensor = prm.lfsensor;
  e->_rfsensor = prm.rfsensor;
  return e;
}

bool _pdEstZMPConfFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "estimator" ) == 0 ) {
    if( !pdEstZMPFRead( fp, instance ) )
      return ( *success = false );
  }
  return true;
}

bool pdEstZMPConfFRead(FILE *fp, pdEstZMP *e)
{
  pdEstZMPInit( e );
  if( !pdFilterArrayFRead( fp, pdEstZMPFilterArray(e) ) ){
    ZRUNERROR( "failed to allocate filter array" );
    return false;
  }
  rewind( fp );
  if( !pdSensorArrayFRead( fp, pdEstZMPSensorArray(e), pdEstZMPFilterArray(e) ) ){
    ZRUNERROR( "failed to allocate sensor array" );
    return false;
  }
  rewind( fp );
  zTagFRead( fp, _pdEstZMPConfFRead, e );
  return true;
}

bool pdEstZMPConfReadFile(pdEstZMP *e, const char *filename)
{
  FILE *fp;
  bool result;

  if( !( fp = fopen( filename, "r" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  result = pdEstZMPConfFRead( fp, e );
  fclose( fp );
  return result;
}
