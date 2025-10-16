#include <pedi2/pd_est_zmp.h>

void pdEstZmpInit(pdEstZmp *e)
{
  zListInit( &e->_lfsensor );
  zListInit( &e->_rfsensor );
  zVec3DZero( &e->estforce );
  zVec3DZero( &e->estzmp );
}

void pdEstZmpDestroy(pdEstZmp *e)
{
  pdSensorListDestroy( &e->_lfsensor );
  pdSensorListDestroy( &e->_rfsensor );
  zVec3DZero( &e->estforce );
  zVec3DZero( &e->estzmp );
  pdFilterArrayDestroy( pdEstZmpFilterArray( e ) );
}

pdSensorListCell *pdSensorListPush(pdSensorList *list, pdSensor *sensor)
{
  pdSensorListCell *cp;

  if( !( cp = zAlloc( pdSensorListCell, 1 ) ) ) return NULL;
  pdSensorListCellSetSensor( cp, sensor );
  zListInsertTail( list, cp );
  return cp;
}

pdSensor *pdSensorListPop(pdSensorList *list)
{
  pdSensorListCell *cp;
  pdSensor *sensor;

  if( zListIsEmpty(list) ) return NULL;
  zListDeleteTail( list, &cp );
  sensor = pdSensorListCellSensor( cp );
  zFree( cp );
  return sensor;
}

pdSensor *pdEstZmpNameFindSensor(pdEstZmp *e, const char *name)
{
  return pdSensorArrayNameFind( pdEstZmpSensorArray(e), name );
}

void pdEstZmpSetData(pdEstZmp *e, const char *name, zVec data)
{
  pdSensorSetRawData( pdEstZmpNameFindSensor(e, name), data );
}

zVec3D *pdEstZmpCalcFootForce(pdEstZmp *e, pdSensorList *s, zVec3D *f)
{
  pdSensorListCell *sp;
  zVec3D v;

  zVec3DZero( f );
  zListForEach( s, sp ){
    pdSensor6FTGetWldF( sp->data, &v );
    zVec3DAddDRC( f, &v );
  }
  return f;
}

zVec3D *pdEstZmpCalcForce(pdEstZmp *e, zVec3D *f)
{
  zVec3D lff, rff;

  pdEstZmpCalcFootForce( e, &e->_lfsensor, &lff );
  pdEstZmpCalcFootForce( e, &e->_rfsensor, &rff );
  return zVec3DAdd( &lff, &rff, f );
}

zVec3D *pdEstZmpCalcFootZMP(pdEstZmp *e, pdSensorList *s, double pz, zVec3D *zmp, double *fz)
{
  pdSensorListCell *sp;
  zVec3D p, f, t;
  double px, py;

  px = py = 0;
  *fz = 0;
  zListForEach( s, sp ){
    zVec3DCopy( pdSensorWldPos( sp->data ), &p );
    pdSensor6FTGetWldF( sp->data, &f );
    pdSensor6FTGetWldT( sp->data, &t );
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

zVec3D *pdEstZmpCalcZMP(pdEstZmp *e, double pz, zVec3D *zmp)
{
  zVec3D lfzmp, rfzmp;
  double lffz, rffz;
  double px, py, fz;

  pdEstZmpCalcFootZMP( e, &e->_lfsensor, pz, &lfzmp, &lffz );
  pdEstZmpCalcFootZMP( e, &e->_rfsensor, pz, &rfzmp, &rffz );
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

void pdEstZmpUpdate(pdEstZmp *e, zFrame3D *lfframe, zFrame3D *rfframe, double pz, double dt)
{
  pdSensorListCell *sp;

  pdSensorArrayProcess( pdEstZmpSensorArray(e), dt );

  zListForEach( &e->_lfsensor, sp )
    pdSensorFrameUpdate( sp->data, lfframe );
  zListForEach( &e->_rfsensor, sp )
    pdSensorFrameUpdate( sp->data, rfframe );
  pdEstZmpCalcForce( e, pdEstZmpEstForce(e) );
  pdEstZmpCalcZMP( e, pz, pdEstZmpEstZMP(e) );
}

const pdEstZmp *pdEstZmpValueFPrint(FILE *fp, const pdEstZmp *e)
{
  if( !e ) return NULL;
  fprintf( fp, "%.10g %.10g %.10g %.10g %.10g %.10g",
           pdEstZmpEstForceX(e), pdEstZmpEstForceY(e), pdEstZmpEstForceZ(e),
           pdEstZmpEstZMPX(e), pdEstZmpEstZMPY(e), pdEstZmpEstZMPZ(e));
  return e;
}

const pdEstZmp *pdEstZmpValueNLFPrint(FILE *fp, const pdEstZmp *e)
{
  if( !pdEstZmpValueFPrint( fp, e ) ) return NULL;
  fprintf( fp, "\n" );
  return e;
}

const pdEstZmp *pdEstZmpFPrint(FILE *fp, const pdEstZmp *e)
{
  if( !e )
    fprintf( fp, "Invalid estimator\n" );
  else{
    fprintf( fp, "Force: ");
    zVec3DFPrint( fp, pdEstZmpEstForce(e) );
    fprintf( fp, "ZMP: ");
    zVec3DFPrint( fp, pdEstZmpEstZMP(e) );
  }
  return e;
}


static void *_pdEstZmpTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  if( strcmp( ZTKVal(ztk), "zmp" ) != 0 ){
    ZRUNERROR( "invalid estimator type %s", ZTKVal(ztk) );
    return NULL;
  }
  return obj;
}
static void *_pdEstZmpLeftFootFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  pdSensor *s;
  do{
    zArrayFindName( pdEstZmpSensorArray((pdEstZmp*)obj), ZTKVal(ztk), s );
    if( !s ){
      ZRUNERROR( "%s: unknown sensor", ZTKVal(ztk) );
      return NULL;
    }
    if( !pdEstZmpSensorListPush( (pdEstZmp*)obj, lf, s ) ) return NULL;
  } while( ZTKValNext( ztk ) );
  return obj;
}
static void *_pdEstZmpRightFootFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  pdSensor *s;
  do{
    zArrayFindName( pdEstZmpSensorArray((pdEstZmp*)obj), ZTKVal(ztk), s );
    if( !s ){
      ZRUNERROR( "%s: unknown sensor", ZTKVal(ztk) );
      return NULL;
    }
    if( !pdEstZmpSensorListPush( (pdEstZmp*)obj, rf, s ) ) return NULL;
  } while( ZTKValNext( ztk ) );
  return obj;
}

static bool _pdEstZmpTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", "zmp" );
  return true;
}

static const ZTKPrp __ztk_prp_pdestzmp[] = {
  { ZTK_KEY_PEDI2_ESTIMATOR_TYPE,      1, _pdEstZmpTypeFromZTK,      _pdEstZmpTypeFPrintZTK },
  { ZTK_KEY_PEDI2_ESTIMATOR_LEFTFOOT,  1, _pdEstZmpLeftFootFromZTK,  NULL },
  { ZTK_KEY_PEDI2_ESTIMATOR_RIGHTFOOT, 1, _pdEstZmpRightFootFromZTK, NULL },
};

static void *_pdEstZmpFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return _ZTKEvalKey( (pdEstZmp*)obj, arg, ztk, __ztk_prp_pdestzmp ) ? obj : NULL;
}

static bool _pdEstZmpFPrintZTK(FILE *fp, int i, void *obj){
  pdSensorList *list;
  pdSensorListCell *cp;

  _ZTKPrpKeyFPrint( fp, (pdEstZmp*)obj, __ztk_prp_pdestzmp );
  list = &((pdEstZmp*)obj)->_lfsensor;
  if( !zListIsEmpty(list) ){
    fprintf( fp, "%s:", ZTK_KEY_PEDI2_ESTIMATOR_LEFTFOOT );
    zListForEachRew( list, cp )
      fprintf( fp, " %s", zName(pdSensorListCellSensor(cp) ) );
    fprintf( fp, "\n" );
  }
  list = &((pdEstZmp*)obj)->_rfsensor;
  if( !zListIsEmpty(list) ){
    fprintf( fp, "%s:", ZTK_KEY_PEDI2_ESTIMATOR_RIGHTFOOT );
    zListForEachRew( list, cp )
      fprintf( fp, " %s", zName(pdSensorListCellSensor(cp) ) );
    fprintf( fp, "\n" );
  }
  return true;
}

static const ZTKPrp __ztk_prp_tag_pedi2_estimator[] = {
  { ZTK_TAG_PEDI2_ESTIMATOR, 1, _pdEstZmpFromZTK, _pdEstZmpFPrintZTK },
};

pdEstZmp *pdEstZmpFromZTK(pdEstZmp *e, ZTK *ztk){
  pdEstZmpInit( e );
  if( !pdFilterArrayFromZTK( pdEstZmpFilterArray(e), ztk ) ) return NULL;
  if( !pdSensorArrayFromZTK( pdEstZmpSensorArray(e), pdEstZmpFilterArray(e), ztk ) ) return NULL;
  if( !_ZTKEvalTag( e, NULL, ztk, __ztk_prp_tag_pedi2_estimator ) ) return NULL;
  return e;
}

void pdEstZmpFPrintZTK(FILE *fp, pdEstZmp *e){
  pdFilterArrayFPrintZTK( fp, pdEstZmpFilterArray(e) );
  pdSensorArrayFPrintZTK( fp, pdEstZmpSensorArray(e) );
  _ZTKPrpTagFPrint( fp, e, __ztk_prp_tag_pedi2_estimator );
}

pdEstZmp *pdEstZmpReadZTK(pdEstZmp *e, const char filename[]){
  ZTK ztk;

  ZTKInit( &ztk );
  if( ZTKParse( &ztk, filename ) )
    e = pdEstZmpFromZTK( e, &ztk );
  ZTKDestroy( &ztk );
  return e;
}

bool pdEstZmpWriteZTK(pdEstZmp *e, const char filename[]){
  FILE *fp;

  if( !( fp = zOpenZTKFile( filename, "w" ) ) ) return false;
  pdEstZmpFPrintZTK( fp, e );
  fclose(fp);
  return true;
}
