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
  pdFilterSetTimeTag=1,
  pdFilterSetTimeStepTag,
  pdFilterSetInputTag,
  pdFilterUpdateTag,
} pdFilterVFTableTag;

typedef void pdFilterSetTimeType(pdFilter*,double);
typedef void pdFilterSetTimeStepType(pdFilter*,double);
typedef void pdFilterSetInputType(pdFilter*,double);
typedef void pdFilterUpdateType(pdFilter*);

extern void *pdFilterVFTable[];
/* c'tor and d'tor */
__EXPORT void pdFilterInit(pdFilter *filter, double dt);
__EXPORT void pdFilterDestroy(pdFilter *filter);

/* implementations of virtual functions */
__EXPORT void pdFilterSetTime_Imp(pdFilter *filter, double t);
__EXPORT void pdFilterSetTimeStep_Imp(pdFilter *filter, double dt);
__EXPORT void pdFilterSetInput_Imp(pdFilter *filter, double input);
__EXPORT void pdFilterUpdate_Imp(pdFilter *filter);

/* methods */
#define pdFilterTime(self)     ( ((pdFilter*)self)->_t )
#define pdFilterTimeStep(self) ( ((pdFilter*)self)->_dt )
#define pdFilterInput(self)    ( ((pdFilter*)self)->_input )
#define pdFilterOutput(self)   ( ((pdFilter*)self)->_output )
#define pdFilterSetTime(self,t) ((pdFilterSetTimeType*)((pdFilter*)self)->vftable[pdFilterSetTimeTag])( (pdFilter*)self, t )
#define pdFilterSetTimeStep(self,dt) ((pdFilterSetTimeStepType*)((pdFilter*)self)->vftable[pdFilterSetTimeStepTag])( (pdFilter*)self, dt )
#define pdFilterSetInput(self,input) ((pdFilterSetInputType*)((pdFilter*)self)->vftable[pdFilterSetInputTag])( (pdFilter*)self, input )
#define pdFilterUpdate(self) ((pdFilterUpdateType*)((pdFilter*)self)->vftable[pdFilterUpdateTag])( (pdFilter*)self )

__END_DECLS

#include <pedi2/pd_filter_none.h>
#include <pedi2/pd_filter_bw.h>

#endif /* __PD_FILTER_H__ */
