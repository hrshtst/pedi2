#include <pedi2/pd_filter.h>

typedef struct{
  dzSys bwf;
  double cf;
  int dim;
} _pdFilterBW;

static void _pdFilterDestroyBW(_pdFilterBW *bw);
static void _pdFilterRefreshBW(_pdFilterBW *bw);
static double _pdFilterUpdateBW(_pdFilterBW *bw, double dt);

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

pdFilterMethod pd_filter_bw_met = {
  type: "bw",
  destroy: pdFilterDestroyBW,
  refresh: pdFilterRefreshBW,
  update: pdFilterUpdateBW,
};

bool pdFilterCreateBW(pdFilter *filter, double cf, int dim)
{
  _pdFilterBW *bw;

  if( !( bw = zAlloc( _pdFilterBW, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  if( !dzSysCreateBW( &bw->bwf, cf, dim ) ){
    zFree( bw );
    return false;
  }
  pdFilterInit( filter );
  dzSysInputPtr( &bw->bwf, 0 ) = &filter->input;
  filter->_prm = bw;
  filter->_met = &pd_filter_bw_met;
  return true;
}
