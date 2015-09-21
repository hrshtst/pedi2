#include <pedi2/pd_filter.h>

void pdFilterDestroyDefault(pdFilter *filter)
{
  zNameDestroy( filter );
  zFree( filter->_prm );
  pdFilterInit( filter );
}

void pdFilterRefreshDefault(pdFilter *filter)
{}

static pdFilterMethod *_pdFilterMethodByStr(char str[]);

pdFilterMethod *_pdFilterMethodByStr(char str[])
{
  static pdFilterMethod *met_array[] = {
    &pd_filter_none_met, &pd_filter_bw_met,
    NULL,
  };
  register int i;

  for( i=0; met_array[i]; i++ ){
    if( strcmp( met_array[i]->type, str ) == 0 ) return met_array[i];
  }
  ZRUNERROR( "cannot find a filter type %s", str );
  return NULL;
}

typedef struct{
  pdFilterMethod *met;
  char name[BUFSIZ];
} _pdFilterParam;

bool _pdFilterFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "type" ) == 0 ){
    if( !( ((_pdFilterParam *)instance)->met = _pdFilterMethodByStr( zFToken(fp,buf,BUFSIZ) ) ) )
      *success = false;
  } else
  if( strcmp( buf, "name" ) == 0 ){
    if( !zFToken( fp, ((_pdFilterParam *)instance)->name, BUFSIZ ) )
      *success = false;
  } else
    return false;
  return true;
}

pdFilter *pdFilterFRead(FILE *fp, pdFilter *filter)
{
  _pdFilterParam prm;
  int cur;

  prm.met = NULL;
  prm.name[0] = '\0';
  cur = ftell( fp );
  zFieldFRead( fp, _pdFilterFRead, &prm );
  if( !prm.met ){
    ZRUNERROR( "type not specified" );
    return NULL;
  }
  fseek( fp, cur, SEEK_SET );
  if( prm.met->fread( fp, filter ) ){
    if( !zNameSet( filter, prm.name ) ){
      ZALLOCERROR();
      return NULL;
    }
    return filter;
  }
  return NULL;
}

static bool _pdFilterFAlloc(FILE *fp, pdFilterArray *arr);

void pdFilterArrayDestroy(pdFilterArray *arr)
{
  register uint i;

  for( i=0; i<zArrayNum(arr); i++ )
    pdFilterDestroy( zArrayElem(arr,i) );
  zArrayFree( arr );
}

bool _pdFilterFAlloc(FILE *fp, pdFilterArray *arr)
{
  int n;

  n = zFCountTag( fp, PD_FILTER_TAG );
  zArrayAlloc( arr, pdFilter, n );
  if( !zArrayBuf(arr) ){
    ZALLOCERROR();
    return false;
  }
  return true;
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
  register uint i;

  for( i=0; i<zArrayNum(arr); i++ )
    pdFilterUpdate( zArrayElem(arr,i), dt );
}

typedef struct{
  pdFilterArray *arr;
  int count;
} _pdFilterArrayParam;

bool _pdFilterArrayFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _pdFilterArrayParam *prm;

  prm = instance;
  if( strcmp( buf, PD_FILTER_TAG ) == 0 ){
    if( !pdFilterFRead( fp, zArrayElem(prm->arr,prm->count++) ) ){
      *success = false;
      return false;
    }
  } else
    return false;
  return true;
}

bool pdFilterArrayFRead(FILE *fp, pdFilterArray *arr)
{
  _pdFilterArrayParam prm;

  zArrayInit( arr );
  if( !_pdFilterFAlloc( fp, arr ) ) return false;
  rewind( fp );
  prm.count = 0;
  prm.arr = arr;
  return zTagFRead( fp, _pdFilterArrayFRead, &prm );
}
