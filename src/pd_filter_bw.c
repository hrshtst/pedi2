#include <pedi2/pd_filter.h>

typedef struct{
  dzSys bwf;
  double cf;
  uint dim;
} _pdBW;

static void _pdFilterBWDestroy(pdFilter *filter)
{
  _pdBW *bw = filter->prp;

  if( bw ){
    dzSysDestroy( &bw->bwf );
    zFree( bw );
  }
  zNameFree( filter );
  pdFilterInit( filter );
}

static void _pdFilterBWRefresh(pdFilter *filter)
{
  _pdBW *bw = filter->prp;

  dzSysRefresh( &bw->bwf );
  pdFilterOutput( filter ) = 0;
  pdFilterDefaultRefresh( filter );
}

static double _pdFilterBWUpdate(pdFilter *filter, double dt)
{
  _pdBW *bw = filter->prp;

  pdFilterOutput( filter ) = zVecElem( dzSysUpdate( &bw->bwf, dt ), 0 );
  return pdFilterOutput( filter );
}

static pdFilter *_pdFilterBWClone(pdFilter *org, pdFilter *cln)
{
  _pdBW *bw = org->prp;

  pdFilterBWCreate( cln, bw->cf, bw->dim );
  zNameSet( cln, zNamePtr(org) );
  pdFilterInput( cln ) = pdFilterInput( org );
  pdFilterOutput( cln ) = pdFilterOutput( org );
  return cln;
}

typedef struct{
  double cf;
  uint dim;
} _pdBWParam;

static void *_pdFilterBWCFFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdBWParam*)val)->cf = ZTKDouble(ztk);
  return val;
}
static void *_pdFilterBWDimFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdBWParam*)val)->dim = ZTKDouble(ztk);
  return val;
}

static bool _pdFilterBWCFFPrintZTK(FILE *fp, int i, const void *prp){
  fprintf( fp, "%.10g\n", ((_pdBW*)((pdFilter*)prp)->prp)->cf );
  return true;
}
static bool _pdFilterBWDimFPrintZTK(FILE *fp, int i, const void *prp){
  fprintf( fp, "%d\n", ((_pdBW*)((pdFilter*)prp)->prp)->dim );
  return true;
}

static const ZTKPrp __ztk_prp_pdfilter_bw[] = {
  { ZTK_KEY_PEDI2_FILTER_CUTOFFFREQ, 1, _pdFilterBWCFFromZTK,  _pdFilterBWCFFPrintZTK  },
  { ZTK_KEY_PEDI2_FILTER_DIM,        1, _pdFilterBWDimFromZTK, _pdFilterBWDimFPrintZTK },
};

static pdFilter *_pdFilterBWFromZTK(pdFilter *filter, ZTK *ztk)
{
  _pdBWParam prm = { 1.0, 1 };
  if( !_ZTKEvalKey( &prm, NULL, ztk, __ztk_prp_pdfilter_bw ) ) return NULL;
  return pdFilterBWCreate( filter, prm.cf, prm.dim );
}

static void _pdFilterBWFPrintZTK(FILE *fp, pdFilter *filter)
{
  _ZTKPrpKeyFPrint( fp, filter, __ztk_prp_pdfilter_bw );
}

pdFilterCom pd_filter_bw_com = {
  .typestr = "bw",
  ._destroy = _pdFilterBWDestroy,
  ._refresh = _pdFilterBWRefresh,
  ._update = _pdFilterBWUpdate,
  ._clone = _pdFilterBWClone,
  ._fromZTK = _pdFilterBWFromZTK,
  ._fprintZTK = _pdFilterBWFPrintZTK,
};

pdFilter *pdFilterBWCreate(pdFilter *filter, double cf, uint dim)
{
  _pdBW *bw;

  if( !( bw = zAlloc( _pdBW, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  bw->cf = cf;
  bw->dim = dim;
  if( !dzSysBWCreate( &bw->bwf, bw->cf, bw->dim ) ){
    zFree( bw );
    return NULL;
  }
  pdFilterInit( filter );
  dzSysInputPtr( &bw->bwf, 0 ) = &filter->input;
  filter->prp = bw;
  filter->com = &pd_filter_bw_com;
  return filter;
}
