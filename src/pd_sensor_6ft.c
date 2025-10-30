#include <pedi2/pd_sensor.h>
#include <zeo/zeo_vec3d.h>

typedef struct{
  zVec3D f;
  zVec3D tau;
} _pdSensor6FT;

static void _pdSensor6FTDestroy(pdSensor *sensor)
{
  zFree( sensor->prp );
  pdSensorDefaultDestroy( sensor );
}

static zVec _pdSensor6FTProcess(pdSensor *sensor, double dt)
{
  _pdSensor6FT *ft;

  ft = sensor->prp;
  pdSensorDefaultProcess( sensor, dt );
  zVec3DCreate( &ft->f,
                pdSensorDataVal(sensor,0),
                pdSensorDataVal(sensor,1),
                pdSensorDataVal(sensor,2) );
  zVec3DCreate( &ft->tau,
                pdSensorDataVal(sensor,3),
                pdSensorDataVal(sensor,4),
                pdSensorDataVal(sensor,5) );
  return pdSensorData( sensor );
}

void _pdSensor6FTFrameUpdate(pdSensor *sensor, zFrame3D *frame)
{
  pdSensorDefaultFrameUpdate( sensor, frame );
}

#define PD_SENSOR_6FT_SIZE 6
typedef struct{
  char linkname[BUFSIZ];
  zFrame3D linkframe;
  pdFilterArray *filterarray;
  pdFilterArray filters;
  int num_filter;
} _pdSensor6FTPrp;

static pdFilter *_pdSensor6FTFilterClone(pdFilterArray *filterarray, const char *name, _pdSensor6FTPrp *prp){
  pdFilter *filter;
  zArrayFindName( filterarray, name, filter );
  if( !filter ){
    ZRUNERROR( "%s: unknown filter", name );
    return NULL;
  }
  pdFilterClone( filter, zArrayElem( &prp->filters, prp->num_filter ) );
  return filter;
}

static void *_pdSensor6FTLinkFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zStrCopy( ((_pdSensor6FTPrp*)arg)->linkname, ZTKVal(ztk), BUFSIZ ) ? obj : NULL;
}
static void *_pdSensor6FTFrameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DFromZTK( &((_pdSensor6FTPrp*)arg)->linkframe, ztk );
  return obj;
}
static void *_pdSensor6FTFilterFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  do{
    _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
    ((_pdSensor6FTPrp*)arg)->num_filter++;
  } while( ZTKValNext( ztk ) );
  return obj;
}
static void *_pdSensor6FTFilterFxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_pdSensor6FTPrp*)arg)->num_filter = 0;
  _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
  return obj;
}
static void *_pdSensor6FTFilterFyFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_pdSensor6FTPrp*)arg)->num_filter = 1;
  _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
  return obj;
}
static void *_pdSensor6FTFilterFzFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_pdSensor6FTPrp*)arg)->num_filter = 2;
  _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
  return obj;
}
static void *_pdSensor6FTFilterTxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_pdSensor6FTPrp*)arg)->num_filter = 3;
  _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
  return obj;
}
static void *_pdSensor6FTFilterTyFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_pdSensor6FTPrp*)arg)->num_filter = 4;
  _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
  return obj;
}
static void *_pdSensor6FTFilterTzFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_pdSensor6FTPrp*)arg)->num_filter = 5;
  _pdSensor6FTFilterClone( ((_pdSensor6FTPrp*)arg)->filterarray, ZTKVal(ztk), (_pdSensor6FTPrp*)arg );
  return obj;
}

static bool _pdSensor6FTLinkFPrintZTK(FILE *fp, int i, void *obj){
  if( pdSensorLinkName((pdSensor*)obj)[0] == '\0' ) return false;
  fprintf( fp, "%s\n", pdSensorLinkName((pdSensor*)obj) );
  return true;
}
static bool _pdSensor6FTFrameFPrintZTK(FILE *fp, int i, void *obj){
  zFrame3DFPrint( fp, pdSensorLinkFrame((pdSensor*)obj) );
  return true;
}
static bool _pdSensor6FTFilterFxFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( pdSensorFilterElem((pdSensor*)obj, 0)) );
  return true;
}
static bool _pdSensor6FTFilterFyFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( pdSensorFilterElem((pdSensor*)obj, 1)) );
  return true;
}
static bool _pdSensor6FTFilterFzFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( pdSensorFilterElem((pdSensor*)obj, 2)) );
  return true;
}
static bool _pdSensor6FTFilterTxFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( pdSensorFilterElem((pdSensor*)obj, 3)) );
  return true;
}
static bool _pdSensor6FTFilterTyFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( pdSensorFilterElem((pdSensor*)obj, 4)) );
  return true;
}
static bool _pdSensor6FTFilterTzFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( pdSensorFilterElem((pdSensor*)obj, 5)) );
  return true;
}

static const ZTKPrp __ztk_prp_pdsensor_6ft[] = {
  { ZTK_KEY_PEDI2_SENSOR_LINK,     1, _pdSensor6FTLinkFromZTK,     _pdSensor6FTLinkFPrintZTK     },
  { ZTK_KEY_PEDI2_SENSOR_FRAME,    1, _pdSensor6FTFrameFromZTK,    _pdSensor6FTFrameFPrintZTK    },
  { ZTK_KEY_PEDI2_SENSOR_FILTER,   1, _pdSensor6FTFilterFromZTK,   NULL                          },
  { ZTK_KEY_PEDI2_SENSOR_FILTERFX, 1, _pdSensor6FTFilterFxFromZTK, _pdSensor6FTFilterFxFPrintZTK },
  { ZTK_KEY_PEDI2_SENSOR_FILTERFY, 1, _pdSensor6FTFilterFyFromZTK, _pdSensor6FTFilterFyFPrintZTK },
  { ZTK_KEY_PEDI2_SENSOR_FILTERFZ, 1, _pdSensor6FTFilterFzFromZTK, _pdSensor6FTFilterFzFPrintZTK },
  { ZTK_KEY_PEDI2_SENSOR_FILTERTX, 1, _pdSensor6FTFilterTxFromZTK, _pdSensor6FTFilterTxFPrintZTK },
  { ZTK_KEY_PEDI2_SENSOR_FILTERTY, 1, _pdSensor6FTFilterTyFromZTK, _pdSensor6FTFilterTyFPrintZTK },
  { ZTK_KEY_PEDI2_SENSOR_FILTERTZ, 1, _pdSensor6FTFilterTzFromZTK, _pdSensor6FTFilterTzFPrintZTK },
};

static pdSensor *_pdSensor6FTFromZTK(pdSensor *sensor, pdFilterArray *filterarray, ZTK *ztk)
{
  _pdSensor6FTPrp prp;

  zFrame3DIdent( &prp.linkframe );
  prp.filterarray = filterarray;
  if( !pdFilterArrayAlloc( &prp.filters, PD_SENSOR_6FT_SIZE ) )
    return NULL;
  prp.num_filter = 0;
  if( !_ZTKEvalKey( sensor, &prp, ztk, __ztk_prp_pdsensor_6ft ) ) return NULL;
  return pdSensor6FTCreate( sensor, prp.linkname, &prp.linkframe, &prp.filters ) ? sensor : NULL;
}

static void _pdSensor6FTFPrintZTK(FILE *fp, pdSensor *sensor)
{
  _ZTKPrpKeyFPrint( fp, sensor, __ztk_prp_pdsensor_6ft );
}

pdSensorCom pd_sensor_6ft_com = {
  .typestr = "6ft",
  ._destroy = _pdSensor6FTDestroy,
  ._process = _pdSensor6FTProcess,
  ._frameupdate = _pdSensor6FTFrameUpdate,
  ._fromZTK = _pdSensor6FTFromZTK,
  ._fprintZTK = _pdSensor6FTFPrintZTK,
};

pdSensor *pdSensor6FTCreate(pdSensor *sensor, const char linkname[], zFrame3D *linkframe, pdFilterArray *farray)
{
  _pdSensor6FT *ft;

  if( !( ft = zAlloc( _pdSensor6FT, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  pdSensorInit( sensor );
  pdSensorSize( sensor ) = PD_SENSOR_6FT_SIZE;
  pdSensorRawData( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  pdSensorData( sensor ) = zVecAlloc( pdSensorSize(sensor) );
  zStrCopy( pdSensorLinkName(sensor), linkname, BUFSIZ );
  zFrame3DCopy( linkframe, pdSensorLinkFrame(sensor) );
  if( zArraySize(farray) == PD_SENSOR_6FT_SIZE ){
    zArrayAssign( pdSensorFilters(sensor), zArrayBuf(farray), zArraySize(farray) );
  } else {
    ZRUNERROR( "unmatched filter array size (%d) with sensor size (%d)", zArraySize(farray), PD_SENSOR_6FT_SIZE );
    return NULL;
  }
  sensor->prp = ft;
  sensor->com = &pd_sensor_6ft_com;
  return sensor;
}

zVec3D *pdSensor6FTGetF(pdSensor *sensor, zVec3D *f)
{
  _pdSensor6FT *ft;

  ft = sensor->prp;
  zVec3DCopy( &ft->f, f );
  return f;
}

zVec3D *pdSensor6FTGetT(pdSensor *sensor, zVec3D *tau)
{
  _pdSensor6FT *ft;

  ft = sensor->prp;
  zVec3DCopy( &ft->tau, tau );
  return tau;
}

zVec3D *pdSensor6FTGetWldF(pdSensor *sensor, zVec3D *f)
{
  _pdSensor6FT *ft;

  ft = sensor->prp;
  zMulMat3DVec3D( pdSensorWldAtt(sensor), &ft->f, f );
  return f;
}

zVec3D *pdSensor6FTGetWldT(pdSensor *sensor, zVec3D *tau)
{
  _pdSensor6FT *ft;

  ft = sensor->prp;
  zMulMat3DVec3D( pdSensorWldAtt(sensor), &ft->tau, tau );
  return tau;
}
