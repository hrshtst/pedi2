#include <pedi2/pd_filter.h>

void pdFilterDestroyNone(pdFilter *filter)
{
  pdFilterDestroyDefault( filter );
}

double pdFilterUpdateNone(pdFilter *filter, double dt)
{
  pdFilterOutput( filter ) = pdFilterInput( filter );
  return pdFilterOutput( filter );
}

pdFilterMethod pd_filter_none_met = {
  type: "none",
  destroy: pdFilterDestroyNone,
  update: pdFilterUpdateNone,
};

bool pdFilterCreateNone(pdFilter *filter)
{
  pdFilterInit( filter );
  filter->_met = &pd_filter_none_met;
  return true;
}
