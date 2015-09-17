#include <pedi2/pd_filter.h>

#define PD_FILTER_ERR_NOT_IMPL "Not implemented virtual function "

void *pdFilterVFTable[] = {
  "class pdFilter",
  pdFilterTime_Imp,
  pdFilterTimeStep_Imp,
  pdFilterInput_Imp,
  pdFilterOutput_Imp,
  pdFilterSetTime_Imp,
  pdFilterSetTimeStep_Imp,
  pdFilterSetInput_Imp,
  pdFilterAlloc_Imp,
  pdFilterUpdate_Imp,
};

void pdFilterInit(pdFilter *filter, double dt)
{
  filter->_t = 0;
  filter->_dt = dt;
  filter->_input = 0;
  filter->_output = 0;
  filter->vftable = pdFilterVFTable;
}

void pdFilterDestroy(pdFilter *filter)
{
  pdFilterInit( filter, 0 );
}

double pdFilterTime_Imp(pdFilter *filter)
{
  return filter->_t;
}

double pdFilterTimeStep_Imp(pdFilter *filter)
{
  return filter->_dt;
}

double pdFilterInput_Imp(pdFilter *filter)
{
  return filter->_input;
}

double pdFilterOutput_Imp(pdFilter *filter)
{
  return filter->_output;
}

void pdFilterSetTime_Imp(pdFilter *filter, double t)
{
  filter->_t = t;
}

void pdFilterSetTimeStep_Imp(pdFilter *filter, double dt)
{
  filter->_dt = dt;
}

void pdFilterSetInput_Imp(pdFilter *filter, double input)
{
  filter->_input = input;
}

pdFilter *pdFilterAlloc_Imp(pdFilter *filter)
{
  ZRUNERROR( PD_FILTER_ERR_NOT_IMPL );
  return NULL;
}

void pdFilterUpdate_Imp(pdFilter *filter)
{
  ZRUNERROR( PD_FILTER_ERR_NOT_IMPL );
}

