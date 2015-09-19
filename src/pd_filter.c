#include <pedi2/pd_filter.h>

void pdFilterDestroyDefault(pdFilter *filter)
{
  zNameDestroy( filter );
  zFree( filter->_prm );
  pdFilterInit( filter );
}


#if 0
static char *__pdfiltertypename[] = {
  "none", "bw",
  NULL,
};

char *pdFilterTypeExpr(byte type)
{
  return __pdfiltertypename[zLimit(type,PD_FILTER_NONE,PD_FILTER_BW)];
}

byte pdFilterTypeFromStr(char *str)
{
  char **fp;
  byte type;

  for( type=PD_FILTER_NONE, fp=__pdfiltertypename; *fp; fp++, type++ )
    if( !strcmp( str, *fp ) ) return type;
  return PD_FILTER_NONE;
}

/* temporary */
pdFilter *pdFilterStateCopy(pdFilter *s, pdFilter *d){ return d; }

static pdFilter *( *pd_filter_create[])(pdFilter*) = {
  pdFilterCreateNone,
  pdFilterCreateBW,
};

pdFilter *pdFilterCreate(pdFilter *f, byte type)
{
  if( type < PD_FILTER_NONE || type > PD_FILTER_BW ){
    ZRUNERROR( "invalid filter type specified - %d", type );
    return NULL;
  }
  pdFilterInit( f );
  if( !pd_filter_create[( f->type = type )]( f ) ){
    ZRUNERROR( "cannot create filter instance" );
    pdFilterDestroy( f );
    return NULL;
  }
  return f;
}

void pdFilterDestroy(pdFilter *f)
{
  zNameDestroy( f );
  zFree( f->prp );
  pdFilterInit( f );
}

pdFilter *pdFilterClone(pdFilter *src, pdFilter *dst)
{
  if( dst->type > PD_FILTER_NONE ) return NULL;
  if( !pdFilterCreate( dst, pdFilterType(src) ) ) return NULL;
  zNameSet( dst, zName(src) );
  pdFilterStateCopy( src, dst );
  return dst;
}
#endif
