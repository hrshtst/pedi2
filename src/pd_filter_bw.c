#include <pedi2/pd_filter.h>

typedef struct{
  dzSys bwf;
  double cf;
  int dim;
} _pdFilterBW;

static void _pdFilterDestroyBW(_pdFilterBW *bw);
static void _pdFilterRefreshBW(_pdFilterBW *bw);
static double _pdFilterUpdateBW(_pdFilterBW *bw, double dt);
static _pdFilterBW *_pdFilterCloneBW(_pdFilterBW *src, _pdFilterBW *dst);

void _pdFilterDestroyBW(_pdFilterBW *bw)
{
  dzSysDestroy( &bw->bwf );
}

void _pdFilterRefreshBW(_pdFilterBW *bw)
{
  dzSysRefresh( &bw->bwf );
}

double _pdFilterUpdateBW(_pdFilterBW *bw, double dt)
{
  return zVecElem( dzSysUpdate( &bw->bwf, dt ), 0 );
}

void pdFilterDestroyBW(pdFilter *filter)
{
  _pdFilterDestroyBW( filter->_prm );
  pdFilterDestroyDefault( filter );
}

void pdFilterRefreshBW(pdFilter *filter)
{
  _pdFilterRefreshBW( filter->_prm );
  pdFilterRefreshBW( filter );
}

double pdFilterUpdateBW(pdFilter *filter, double dt)
{
  pdFilterOutput( filter ) = _pdFilterUpdateBW( filter->_prm, dt );
  return pdFilterOutput( filter );
}

typedef struct{
  double cf;
  int dim;
} _pdFilterBWParam;

static bool _pdFilterFReadBW(FILE *fp, void *prm, char *buf, bool *success);

bool _pdFilterFReadBW(FILE *fp, void *prm, char *buf, bool *success)
{
  if( strcmp( buf, "cf" ) == 0 ){
    ((_pdFilterBWParam *)prm)->cf = zFDouble( fp );
  } else
  if( strcmp( buf, "dim" ) == 0 ){
    ((_pdFilterBWParam *)prm)->dim = zFInt( fp );
  } else
    return false;
  return true;
}

pdFilter *pdFilterCloneBW(pdFilter *src, pdFilter *dst)
{
  _pdFilterBW *bw;

  bw = src->_prm;
  pdFilterCreateBW( dst, bw->cf, bw->dim );
  zNameSet( dst, zNamePtr(src) );
  pdFilterInput( dst ) = pdFilterInput( src );
  pdFilterOutput( dst ) = pdFilterOutput( src );
  return dst;
}

pdFilter *pdFilterFReadBW(FILE *fp, pdFilter *filter)
{
  _pdFilterBWParam prm = { 1.0, 1 };

  zFieldFRead( fp, _pdFilterFReadBW, &prm );
  return pdFilterCreateBW( filter, prm.cf, prm.dim ) ? filter : NULL;
}

pdFilterMethod pd_filter_bw_met = {
  type: "bw",
  destroy: pdFilterDestroyBW,
  refresh: pdFilterRefreshBW,
  update: pdFilterUpdateBW,
  clone: pdFilterCloneBW,
  fread: pdFilterFReadBW,
};

bool pdFilterCreateBW(pdFilter *filter, double cf, int dim)
{
  _pdFilterBW *bw;

  if( !( bw = zAlloc( _pdFilterBW, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  bw->cf = cf;
  bw->dim = dim;
  if( !dzSysCreateBW( &bw->bwf, bw->cf, bw->dim ) ){
    zFree( bw );
    return false;
  }
  pdFilterInit( filter );
  dzSysInputPtr( &bw->bwf, 0 ) = &filter->input;
  filter->_prm = bw;
  filter->_met = &pd_filter_bw_met;
  return true;
}
