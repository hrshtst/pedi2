#include <pedi2/pd_filter.h>

void *pdFilterVFTable[] = {
  "class pdFilter",
  pdFilterUpdate_Imp,
};

void pdFilterInit(pdFilter *filter, double dt)
{
  filter->_t = 0;
  filter->_dt = dt;
  filter->vftable = pdFilterVFTable;
}

void pdFilterDestroy(pdFilter *filter)
{
  pdFilterInit( filter, 0 );
}

void pdFilterUpdate_Imp(pdFilter *filter, double dt)
{
  ZRUNERROR( "Not implemented virtual function " );
}

