#include <pedi2/pd_filter.h>

void *pdFilterNoneVFTable[] = {
  "class pdFilterNone",
  pdFilterTime_Imp,
  pdFilterTimeStep_Imp,
  pdFilterInput_Imp,
  pdFilterOutput_Imp,
  pdFilterSetTime_Imp,
  pdFilterSetTimeStep_Imp,
  pdFilterSetInput_Imp,
  pdFilterNoneUpdate_Imp,
};

void pdFilterNoneInit(pdFilterNone *f, double dt)
{
  pdFilterInit( &f->base, dt );
  f->base.vftable = pdFilterNoneVFTable;
}

void pdFilterNoneDestroy(pdFilterNone *f)
{
  pdFilterDestroy( &f->base );
}

pdFilterNone *pdFilterNoneAlloc()
{
  pdFilterNone *f;

  if( !( f = zAlloc( pdFilterNone, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  return f;
}

void pdFilterNoneUpdate_Imp(pdFilterNone *f)
{
  f->base._output = pdFilterInput( f );
}
