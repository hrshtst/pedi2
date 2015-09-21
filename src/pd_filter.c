#include <pedi2/pd_filter.h>

void pdFilterDestroyDefault(pdFilter *filter)
{
  zNameDestroy( filter );
  zFree( filter->_prm );
  pdFilterInit( filter );
}

void pdFilterRefreshDefault(pdFilter *filter)
{}
