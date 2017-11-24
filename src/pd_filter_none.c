#include <pedi2/pd_filter.h>

void pdFilterDestroyNone(pdFilter *filter)
{
  pdFilterDestroyDefault( filter );
}

void pdFilterRefreshNone(pdFilter *filter)
{
  pdFilterRefreshDefault( filter );
}

double pdFilterUpdateNone(pdFilter *filter, double dt)
{
  pdFilterOutput( filter ) = pdFilterInput( filter );
  return pdFilterOutput( filter );
}

pdFilter *pdFilterCloneNone(pdFilter *src, pdFilter *dst)
{
  pdFilterCreateNone( dst );
  zNameSet( dst, zNamePtr(src) );
  pdFilterInput( dst ) = pdFilterInput( src );
  pdFilterOutput( dst ) = pdFilterOutput( src );
  return dst;
}

static bool _pdFilterFReadNone(FILE *fp, void *prm, char *bur, bool *success);

bool _pdFilterFReadNone(FILE *fp, void *prm, char *buf, bool *success)
{
  return true;
}

pdFilter *pdFilterFReadNone(FILE *fp, pdFilter *filter)
{
  zFieldFRead( fp, _pdFilterFReadNone, NULL );
  return pdFilterCreateNone( filter ) ? filter : NULL;
}

pdFilterMethod pd_filter_none_met = {
  "none",
  pdFilterDestroyNone,
  pdFilterRefreshNone,
  pdFilterUpdateNone,
  pdFilterCloneNone,
  pdFilterFReadNone,
};

bool pdFilterCreateNone(pdFilter *filter)
{
  pdFilterInit( filter );
  filter->_met = &pd_filter_none_met;
  return true;
}
