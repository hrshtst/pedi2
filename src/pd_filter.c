#include <pedi2/pd_filter.h>

void pdFilterInit(pdFilter *filter, double dt)
{
  filter->_t = 0;
  filter->_dt = dt;
}

void pdFilterDestroy(pdFilter *filter)
{
  pdFilterInit( filter, 0 );
}

