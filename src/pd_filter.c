#include <pedi2/pd_filter.h>

void pdFilterDefaultDestroy(pdFilter *filter)
{
  zNameFree( filter );
  zFree( filter->prp );
  pdFilterInit( filter );
}

void pdFilterDefaultRefresh(pdFilter *filter){}

static pdFilter *_pdFilterAssignByStr(pdFilter *filter, const char *str)
{
  PD_FILTER_COM_ARRAY;
  register int i;

  for( i=0; pd_filter_com[i]; i++ ){
    if( strcmp( pd_filter_com[i]->typestr, str ) == 0 ){
      filter->com = pd_filter_com[i];
      return filter;
    }
  }
  ZRUNERROR( "cannot find a filter type %s", str );
  return NULL;
}

static void *_pdFilterNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zNameSet( (pdFilter*)obj, ZTKVal(ztk) ) ? obj : NULL;
}
static void *_pdFilterTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return _pdFilterAssignByStr( (pdFilter*)obj, ZTKVal(ztk) ) ? obj : NULL;
}

static bool _pdFilterNameFPrintZTK(FILE *fp, int i, const void *obj){
  fprintf( fp, "%s\n", zName((pdFilter*)obj) );
  return true;
}
static bool _pdFilterTypeFPrintZTK(FILE *fp, int i, const void *obj){
  fprintf( fp, "%s\n", ((pdFilter*)obj)->com ? ((pdFilter*)obj)->com->typestr : "unknown" );
  return true;
}

static const ZTKPrp __ztk_prp_pdfilter[] = {
  { ZTK_KEY_PEDI2_FILTER_NAME, 1, _pdFilterNameFromZTK, _pdFilterNameFPrintZTK },
  { ZTK_KEY_PEDI2_FILTER_TYPE, 1, _pdFilterTypeFromZTK, _pdFilterTypeFPrintZTK },
};

pdFilter *pdFilterFromZTK(pdFilter *filter, ZTK *ztk)
{
  char *name;
  if( !_ZTKEvalKey( filter, NULL, ztk, __ztk_prp_pdfilter ) ) return NULL;
  name = zNamePtr(filter);
  if( !filter->com || !filter->com->_fromZTK( filter, ztk ) ) return NULL;
  zNameSet( filter, name );
  return filter;
}

void pdFilterFPrintZTK(FILE *fp, pdFilter *filter)
{
  _ZTKPrpKeyFPrint( fp, filter, __ztk_prp_pdfilter );
  if( filter->com )
    filter->com->_fprintZTK( fp, filter );
}

pdFilterArray *pdFilterArrayAlloc(pdFilterArray *arr, int size)
{
  register int i;

  zArrayAlloc( arr, pdFilter, size );
  if( !zArrayBuf(arr) || zArraySize(arr) != size ){
    ZALLOCERROR();
    return NULL;
  }
  for( i=0; i<size; i++ )
    pdFilterInit( zArrayElemNC(arr, i) );
  return arr;
}

void pdFilterArrayDestroy(pdFilterArray *arr)
{
  register int i;

  for( i=0; i<zArraySize(arr); i++ )
    pdFilterDestroy( zArrayElemNC(arr,i) );
  zArrayFree( arr );
}

pdFilter *pdFilterArrayNameFind(pdFilterArray *arr, const char *name)
{
  pdFilter *filter = NULL;

  zArrayFindName( arr, name , filter );
  if( !filter ){
    ZRUNWARN( "invalid filter name %s", name );
    return NULL;
  }
  return filter;
}

void pdFilterArrayUpdate(pdFilterArray *arr, double dt)
{
  register int i;

  for( i=0; i<zArraySize(arr); i++ )
    pdFilterUpdate( zArrayElem(arr,i), dt );
}

static void *_pdFilterArrayFilterFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return pdFilterFromZTK( zArrayElemNC((pdFilterArray*)obj,i), ztk ) ? obj : NULL;
}

static const ZTKPrp __ztk_prp_tag_pedi2_filter[] = {
  { ZTK_TAG_PEDI2_FILTER, -1, _pdFilterArrayFilterFromZTK, NULL },
};

pdFilterArray *pdFilterArrayFromZTK(pdFilterArray *arr, ZTK *ztk)
{
  int num;

  if( ( num = ZTKCountTag( ztk, ZTK_TAG_PEDI2_FILTER ) ) == 0 ){
    ZRUNWARN( "empty array of filters specified." );
    return NULL;
  }
  if( !pdFilterArrayAlloc( arr, num ) ) return NULL;
  _ZTKEvalTag( arr, NULL, ztk, __ztk_prp_tag_pedi2_filter );
  return arr;
}

void pdFilterArrayFPrintZTK(FILE *fp, pdFilterArray *arr)
{
  int i;

  for( i=0; i<zArraySize(arr); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_PEDI2_FILTER );
    pdFilterFPrintZTK( fp, zArrayElemNC(arr,i) );
    fprintf( fp, "\n" );
  }
  _ZTKPrpTagFPrint( fp, arr, __ztk_prp_tag_pedi2_filter );
}

pdFilterArray *pdFilterArrayReadZTK(pdFilterArray *arr, char filename[])
{
  ZTK ztk;

  ZTKInit( &ztk );
  zArrayInit( arr );
  if( ZTKParse( &ztk, filename ) )
    arr = pdFilterArrayFromZTK( arr, &ztk );
  ZTKDestroy( &ztk );
  return arr;
}

bool pdFilterArrayWriteZTK(pdFilterArray *arr, char filename[])
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( filename, "w" ) ) ) return false;
  pdFilterArrayFPrintZTK( fp, arr );
  fclose(fp);
  return true;
}
