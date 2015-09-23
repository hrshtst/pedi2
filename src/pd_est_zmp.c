#include <pedi2/pd_est_zmp.h>

void pdEstZMPInit(pdEstZMP *e)
{
  zNameSet( e, NULL );
  zArrayInit( &e->_lfsensor );
  zArrayInit( &e->_rfsensor );
  zVec3DClear( &e->estforce );
  zVec3DClear( &e->estzmp );
}

void pdEstZMPDestroy(pdEstZMP *e)
{
  zNameDestroy( e );
  zArrayFree( &e->_lfsensor );
  zArrayFree( &e->_rfsensor );
  zVec3DClear( &e->estforce );
  zVec3DClear( &e->estzmp );
  pdFilterArrayDestroy( pdEstZMPFilterArray( e ) );
}

pdSensor *pdEstZMPNameFindSensor(pdEstZMP *e, const char *name)
{
  return pdSensorArrayNameFind( pdEstZMPSensorArray(e), name );
}

void pdEstZMPSetData(pdEstZMP *e, const char *name, zVec data)
{
  pdSensorSetInput( pdEstZMPFindSensor(e, name), data );
}

zVec3D *pdEstZMPCalcFootForce(pdEstZMP *e, pdSensorPtrArray *s, zVec3D *f)
{
  register uint i;
  zVec3D v;

  zVec3DClear( f );
  for( i=0; i<zArrayNum(s); i++ ){
    pdSensor6FTGetWldF( zArrayBuf(s)[i], &v );
    zVec3DAddDRC( f, &v );
  }
  return f;
}

zVec3D *pdEstZMPCalcForce(pdEstZMP *e, zVec3D *f)
{
  zVec3D lff, rff;

  pdEstZMPCalcFootForce( e, &e->_lfsensor, &lff );
  pdEstZMPCalcFootForce( e, &e->_rfsensor, &rff );
  return zVec3DAdd( &lff, &rff, f );
}

zVec3D *pdEstZMPCalcFootZMP(pdEstZMP *e, pdSensorPtrArray *s, double pz, zVec3D *zmp, double *fz)
{
  register uint i;
  zVec3D p, f, t;
  double px, py;

  px = py = 0;
  *fz = 0;
  for( i=0; i<zArrayNum(s); i++ ){
    zVec3DCopy( pdSensorWldPos(zArrayBuf(s)[i]), &p );
    pdSensor6FTGetWldF( zArrayBuf(s)[i], &f );
    pdSensor6FTGetWldT( zArrayBuf(s)[i], &t );
    px += -t.e[zY] - (p.e[zZ]-pz)*f.e[zX] + p.e[zX]*f.e[zZ];
    py +=  t.e[zX] - (p.e[zZ]-pz)*f.e[zY] + p.e[zY]*f.e[zZ];
    *fz += f.e[zZ];
  }
  if( zIsTiny(*fz) ) px = py = 0;
  else{
    px = px / *fz;
    py = py / *fz;
  }
  return zVec3DCreate( zmp, px, py, pz );
}

zVec3D *pdEstZMPCalcZMP(pdEstZMP *e, double pz, zVec3D *zmp)
{
  zVec3D lfzmp, rfzmp;
  double lffz, rffz;
  double px, py, fz;

  pdEstZMPCalcFootZMP( e, &e->_lfsensor, pz, &lfzmp, &lffz );
  pdEstZMPCalcFootZMP( e, &e->_rfsensor, pz, &rfzmp, &rffz );
  px = lfzmp.e[zX]*lffz + rfzmp.e[zX]*rffz;
  py = lfzmp.e[zY]*lffz + rfzmp.e[zY]*rffz;
  fz = lffz + rffz;
  if( zIsTiny(fz) ) px = py = 0;
  else{
    px = px / fz;
    py = py / fz;
  }
  return zVec3DCreate( zmp, px, py, pz );
}

void pdEstZMPUpdate(pdEstZMP *e, zFrame3D *lfframe, zFrame3D *rfframe, double pz, double dt)
{
  register uint i;

  pdSensorArrayProcess( pdEstZMPSensorArray(e), dt );
  for( i=0; i<zArrayNum(&e->_lfsensor); i++ )
    pdSensorFrameUpdate( zArrayBuf(&e->_lfsensor)[i], lfframe );
  for( i=0; i<zArrayNum(&e->_rfsensor); i++ )
    pdSensorFrameUpdate( zArrayBuf(&e->_rfsensor)[i], rfframe );
  pdEstZMPCalcForce( e, pdEstZMPEstForceVec(e) );
  pdEstZMPCalcZMP( e, pz, pdEstZMPEstZMPVec(e) );
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
  zArraySetNum( &e->_lfsensor, prm.lfsensor_num );
  zArraySetNum( &e->_rfsensor, prm.rfsensor_num );
  zArraySetBuf( &e->_lfsensor, prm.lfsensor );
  zArraySetBuf( &e->_rfsensor, prm.rfsensor );
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
