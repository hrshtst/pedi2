#include <pedi2/pd_filter.h>

void *pdFilterBWVFTable[] = {
  "class pdFilterBW",
  pdFilterTime_Imp,
  pdFilterTimeStep_Imp,
  pdFilterInput_Imp,
  pdFilterOutput_Imp,
  pdFilterSetTime_Imp,
  pdFilterSetTimeStep_Imp,
  pdFilterSetInput_Imp,
  pdFilterBWUpdate_Imp,
};

void pdFilterBWInit(pdFilterBW *bwf, double dt, double cf, int dim)
{
  pdFilterInit( &bwf->base, dt );
  bwf->base.vftable = pdFilterBWVFTable;
  dzSysCreateBW( &bwf->_sys, cf, dim );
  dzSysInputPtr( &bwf->_sys, 0 ) = &bwf->base._input;
}

void pdFilterBWDestroy(pdFilterBW *bwf)
{
  dzSysDestroy( &bwf->_sys );
  pdFilterDestroy( &bwf->base );
}

pdFilterBW *pdFilterBWAlloc()
{
  pdFilterBW *bwf;

  if( !( bwf = zAlloc( pdFilterBW, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  return bwf;
}

void pdFilterBWUpdate_Imp(pdFilterBW *bwf)
{
  bwf->base._output = zVecElem(dzSysUpdate(&bwf->_sys,pdFilterTimeStep(bwf)),0);
}
