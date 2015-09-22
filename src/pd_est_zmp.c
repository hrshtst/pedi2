#include <pedi2/pd_est_zmp.h>

void pdEstZMPInit(pdEstZMP *e)
{
  zVec3DClear( &e->estforce );
  zVec3DClear( &e->estzmp );
}

void pdEstZMPDestroy(pdEstZMP *e)
{
  pdEstZMPInit( e );
}

#if 0
void pdEstZMPInit(pdEstZMP *e_zmp, double dt)
{
  e_zmp->_t = 0;
  e_zmp->_dt = dt;
}

void pdEstZMPDestroy(pdEstZMP *e_zmp)
{
  pdEstZMPInit( e_zmp, 0 );
}

bool _pdEstZMPConfFReadFilterField(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "name" ) == 0 ){
    if( !zFToken( fp, ((filter_t*)instance)->name, BUFSIZ ) ){
      *success = false;
      return false;
    }
  } else {
    ZRUNERROR( "unknown field: %s\n", buf );
    return false;
  }
  return true;
}

pdEstZMP *_pdEstZMPFilterFRead(FILE *fp, pdEstZMP *e, int i)
{
  if( !zFieldFRead( fp, _pdEstZMPConfFReadFilterField, &(e->filter[i]) ) )
    return NULL;
  return e;
}

bool _pdEstZMPConfFReadSensorField(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "name" ) == 0 ){
    if( !zFToken( fp, ((sensor_t*)instance)->name, BUFSIZ ) ){
      *success = false;
      return false;
    }
  } else {
    ZRUNERROR( "unknown field: %s\n", buf );
    return false;
  }
  return true;
}

pdEstZMP *_pdEstZMPSensorFRead(FILE *fp, pdEstZMP *e, int i)
{
  if( !zFieldFRead( fp, _pdEstZMPConfFReadSensorField, &(e->sensor[i]) ) )
    return NULL;
  return e;
}

typedef struct {
  pdEstZMP *e;
  int fc;
  int sc;
} _pdEstZMPParam;

bool _pdEstZMPConfFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _pdEstZMPParam *prm;

  prm = instance;
  if( strcmp( buf, "filter" ) == 0 ) {
    if( !_pdEstZMPFilterFRead( fp, prm->e, prm->fc++ ) )
      return ( *success = false );
  } else if( strcmp( buf, "sensor" ) == 0 ) {
    if( !_pdEstZMPSensorFRead( fp, prm->e, prm->sc++ ) )
      return ( *success = false );
  }
  return true;
}

bool pdEstZMPConfFRead(FILE *fp, pdEstZMP *e_zmp)
{
  _pdEstZMPParam prm;

  prm.e = e_zmp;
  prm.fc = 0;
  prm.sc = 0;
  rewind( fp );
  return zTagFRead( fp, _pdEstZMPConfFRead, &prm );
}

bool pdEstZMPConfReadFile(pdEstZMP *e_zmp, const char *filename)
{
  FILE *fp;
  bool result;

  if( !( fp = fopen( filename, "r" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  result = pdEstZMPConfFRead( fp, e_zmp );
  fclose( fp );
  return result;
}
#endif
