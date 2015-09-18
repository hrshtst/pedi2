#ifndef __PD_FILTER_H__
#define __PD_FILTER_H__

#include <dzco/dz_sys.h>

__BEGIN_DECLS

typedef struct{
  double _t;         /* time */
  double _dt;        /* time step */

  double _input;     /* input value */
  double _output;    /* output value */

  void **vftable;    /* virtual function table */
} pdFilter;

typedef enum{
  pdFilterTimeTag=1,
  pdFilterTimeStepTag,
  pdFilterInputTag,
  pdFilterOutputTag,
  pdFilterSetTimeTag,
  pdFilterSetTimeStepTag,
  pdFilterSetInputTag,
  pdFilterUpdateTag,
} pdFilterVFTableTag;

typedef double pdFilterTimeType(pdFilter*);
typedef double pdFilterTimeStepType(pdFilter*);
typedef double pdFilterInputType(pdFilter*);
typedef double pdFilterOutputType(pdFilter*);
typedef void pdFilterSetTimeType(pdFilter*,double);
typedef void pdFilterSetTimeStepType(pdFilter*,double);
typedef void pdFilterSetInputType(pdFilter*,double);
typedef void pdFilterUpdateType(pdFilter*);

extern void *pdFilterVFTable[];
/* c'tor and d'tor */
__EXPORT void pdFilterInit(pdFilter *filter, double dt);
__EXPORT void pdFilterDestroy(pdFilter *filter);

/* implementations of virtual functions */
__EXPORT double pdFilterTime_Imp(pdFilter *filter);
__EXPORT double pdFilterTimeStep_Imp(pdFilter *filter);
__EXPORT double pdFilterInput_Imp(pdFilter *filter);
__EXPORT double pdFilterOutput_Imp(pdFilter *filter);
__EXPORT void pdFilterSetTime_Imp(pdFilter *filter, double t);
__EXPORT void pdFilterSetTimeStep_Imp(pdFilter *filter, double dt);
__EXPORT void pdFilterSetInput_Imp(pdFilter *filter, double input);
__EXPORT void pdFilterUpdate_Imp(pdFilter *filter);

/* methods */
#define pdFilterTime(self) ((pdFilterTimeType*)((pdFilter*)self)->vftable[pdFilterTimeTag])( (pdFilter*)self )
#define pdFilterTimeStep(self) ((pdFilterTimeStepType*)((pdFilter*)self)->vftable[pdFilterTimeStepTag])( (pdFilter*)self )
#define pdFilterInput(self) ((pdFilterInputType*)((pdFilter*)self)->vftable[pdFilterInputTag])( (pdFilter*)self )
#define pdFilterOutput(self) ((pdFilterOutputType*)((pdFilter*)self)->vftable[pdFilterOutputTag])( (pdFilter*)self )
#define pdFilterSetTime(self,t) ((pdFilterSetTimeType*)((pdFilter*)self)->vftable[pdFilterSetTimeTag])( (pdFilter*)self, t )
#define pdFilterSetTimeStep(self,dt) ((pdFilterSetTimeStepType*)((pdFilter*)self)->vftable[pdFilterSetTimeStepTag])( (pdFilter*)self, dt )
#define pdFilterSetInput(self,input) ((pdFilterSetInputType*)((pdFilter*)self)->vftable[pdFilterSetInputTag])( (pdFilter*)self, input )
#define pdFilterUpdate(self) ((pdFilterUpdateType*)((pdFilter*)self)->vftable[pdFilterUpdateTag])( (pdFilter*)self )

__END_DECLS

#include <pedi2/pd_filter_none.h>
#include <pedi2/pd_filter_bw.h>

#endif /* __PD_FILTER_H__ */
