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

pdFilterMethod pd_filter_none_met = {
  type: "none",
  destroy: pdFilterDestroyNone,
  refresh: pdFilterRefreshNone,
  update: pdFilterUpdateNone,
};

bool pdFilterCreateNone(pdFilter *filter)
{
  pdFilterInit( filter );
  filter->_met = &pd_filter_none_met;
  return true;
}
