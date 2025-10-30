#include <pedi2/pd_filter.h>

static void _pdFilterNoneDestroy(pdFilter *filter)
{
  pdFilterDefaultDestroy( filter );
}

static void _pdFilterNoneRefresh(pdFilter *filter)
{
  pdFilterDefaultRefresh( filter );
}

static double _pdFilterNoneUpdate(pdFilter *filter, double dt)
{
  pdFilterOutput( filter ) = pdFilterInput( filter );
  return pdFilterOutput( filter );
}

static pdFilter *_pdFilterNoneClone(pdFilter *org, pdFilter *cln)
{
  pdFilterNoneCreate( cln );
  zNameSet( cln, zNamePtr(org) );
  pdFilterInput( cln ) = pdFilterInput( org );
  pdFilterOutput( cln ) = pdFilterOutput( org );
  return cln;
}

static pdFilter *_pdFilterNoneFromZTK(pdFilter *filter, ZTK *ztk){ return filter; }
static void _pdFilterNoneFPrintZTK(FILE *fp, pdFilter *filter){}

pdFilterCom pd_filter_none_com = {
  .typestr = "none",
  ._destroy = _pdFilterNoneDestroy,
  ._refresh = _pdFilterNoneRefresh,
  ._update = _pdFilterNoneUpdate,
  ._clone = _pdFilterNoneClone,
  ._fromZTK = _pdFilterNoneFromZTK,
  ._fprintZTK = _pdFilterNoneFPrintZTK,
};

pdFilter *pdFilterNoneCreate(pdFilter *filter)
{
  pdFilterInit( filter );
  filter->com = &pd_filter_none_com;
  return filter;
}
