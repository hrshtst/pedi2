#include <pedi2/pd_sensor.h>

void pdSensorDefaultDestroy(pdSensor *sensor)
{
  zNameFree( sensor );
  pdFilterArrayDestroy( pdSensorFilters( sensor ) );
  zVecFree( pdSensorRawData( sensor ) );
  zVecFree( pdSensorData( sensor ) );
  zFree( sensor->prp );
  pdSensorInit( sensor );
}

zVec pdSensorDefaultProcess(pdSensor *sensor, double dt)
{
  register int i;

  for( i=0; i<pdSensorSize(sensor); i++ )
    pdFilterArrayInput( pdSensorFilters(sensor), i ) = pdSensorRawDataVal( sensor, i );
  pdFilterArrayUpdate( pdSensorFilters(sensor), dt );
  for( i=0; i<pdSensorSize(sensor); i++ )
    pdSensorDataSetVal( sensor, i, pdSensorFilterOutput( sensor, i ) );
  return pdSensorData( sensor );
}

void pdSensorDefaultFrameUpdate(pdSensor *sensor, zFrame3D *frame)
{
  zFrame3DCascade( frame, pdSensorLinkFrame(sensor), pdSensorWldFrame(sensor) );
}

static pdSensor *_pdSensorAssignByStr(pdSensor *sensor, const char *str)
{
  PD_SENSOR_COM_ARRAY;
  register int i;

  for( i=0; pd_sensor_com[i]; i++ ){
    if( strcmp( pd_sensor_com[i]->typestr, str ) == 0 ){
      sensor->com = pd_sensor_com[i];
      return sensor;
    }
  }
  ZRUNERROR( "cannot find a sensor type %s", str );
  return NULL;
}

static void *_pdSensorNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zNameSet( (pdSensor*)obj, ZTKVal(ztk) ) ? obj : NULL;
}
static void *_pdSensorTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return _pdSensorAssignByStr( (pdSensor*)obj, ZTKVal(ztk) ) ? obj : NULL;
}

static bool _pdSensorNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((pdSensor*)obj) );
  return true;
}
static bool _pdSensorTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", ((pdSensor*)obj)->com ? ((pdSensor*)obj)->com->typestr : "unknown" );
  return true;
}

static const ZTKPrp __ztk_prp_pdsensor[] = {
  { ZTK_KEY_PEDI2_SENSOR_NAME, 1, _pdSensorNameFromZTK, _pdSensorNameFPrintZTK },
  { ZTK_KEY_PEDI2_SENSOR_TYPE, 1, _pdSensorTypeFromZTK, _pdSensorTypeFPrintZTK },
};

pdSensor *pdSensorFromZTK(pdSensor *sensor, pdFilterArray *filters, ZTK *ztk)
{
  char *name;
  if( !_ZTKEvalKey( sensor, NULL, ztk, __ztk_prp_pdsensor ) ) return NULL;
  name = zNamePtr(sensor);
  if( !sensor->com || !sensor->com->_fromZTK( sensor, filters, ztk ) ) return NULL;
  zNameSet( sensor, name );
  return sensor;
}

void pdSensorFPrintZTK(FILE *fp, pdSensor *sensor)
{
  _ZTKPrpKeyFPrint( fp, sensor, __ztk_prp_pdsensor );
  if( sensor->com )
    sensor->com->_fprintZTK( fp, sensor );
}

pdSensorArray *pdSensorArrayAlloc(pdSensorArray *arr, int size)
{
  zArrayAlloc( arr, pdSensor, size );
  if( !zArrayBuf(arr) ){
    ZALLOCERROR();
    return NULL;
  }
  return arr;
}

void pdSensorArrayDestroy(pdSensorArray *arr)
{
  register int i;

  for( i=0; i<zArraySize(arr); i++ )
    pdSensorDestroy( zArrayElem(arr,i) );
  zArrayFree( arr );
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
  register int i;

  for( i=0; i<zArraySize(arr); i++ )
    pdSensorProcess( zArrayElem(arr,i), dt );
}

static void *_pdSensorArraySensorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return pdSensorFromZTK( zArrayElemNC((pdSensorArray*)obj,i), (pdFilterArray*)arg, ztk ) ? obj : NULL;
}

static const ZTKPrp __ztk_prp_tag_pedi2_sensor[] = {
  { ZTK_TAG_PEDI2_SENSOR, -1, _pdSensorArraySensorFromZTK, NULL },
};

pdSensorArray *pdSensorArrayFromZTK(pdSensorArray *arr, pdFilterArray *farray, ZTK *ztk)
{
  int num;

  if( ( num = ZTKCountTag( ztk, ZTK_TAG_PEDI2_SENSOR ) ) == 0 ){
    ZRUNWARN( "empty array of sensors specified." );
    return NULL;
  }
  if( !pdSensorArrayAlloc( arr, num ) ) return NULL;
  _ZTKEvalTag( arr, farray, ztk, __ztk_prp_tag_pedi2_sensor );
  return arr;
}

void pdSensorArrayFPrintZTK(FILE *fp, pdSensorArray *arr)
{
  int i;

  for( i=0; i<zArraySize(arr); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_PEDI2_SENSOR );
    pdSensorFPrintZTK( fp, zArrayElemNC(arr,i) );
    fprintf( fp, "\n" );
  }
  _ZTKPrpTagFPrint( fp, arr, __ztk_prp_tag_pedi2_sensor );
}

pdSensorArray *pdSensorArrayReadZTK(pdSensorArray *arr, pdFilterArray *farray, char filename[])
{
  ZTK ztk;

  ZTKInit( &ztk );
  zArrayInit( arr );
  if( ZTKParse( &ztk, filename ) )
    arr = pdSensorArrayFromZTK( arr, farray, &ztk );
  ZTKDestroy( &ztk );
  return arr;
}

bool pdSensorArrayWriteZTK(pdSensorArray *arr, char filename[])
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( filename, "w" ) ) ) return false;
  pdSensorArrayFPrintZTK( fp, arr );
  fclose(fp);
  return true;
}
