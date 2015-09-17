#ifndef __PD_FILTER_H__
#define __PD_FILTER_H__

#include <dzco/dz_sys.h>

__BEGIN_DECLS

typedef struct{
  double _t;         /* time */
  double _dt;        /* time step */

  void **vftable;    /* virtual function table */
} pdFilter;

typedef enum{
  pdFilterUpdateTag=1,
} pdFilterVFTableTag;

typedef void pdFilterUpdateType(pdFilter*,double);

extern void *pdFilterVFTable[];
/* c'tor and d'tor */
__EXPORT void pdFilterInit(pdFilter *filter, double dt);
__EXPORT void pdFilterDestroy(pdFilter *filter);

/* implementations of virtual functions */
__EXPORT void pdFilterUpdate_Imp(pdFilter *filter, double dt);

/* virtual functions */
#define pdFilterUpdate(self,dt) ((pdFilterUpdateType*)((pdFilter*)self)->vftable[pdFilterUpdateTag])( (pdFilter*)self, dt )

/* methods */
#define pdFilterTime(f)     (f)->_t
#define pdFilterTimeStep(f) (f)->_dt

#define pdFilterSetTime(f,t)      ( (f)->_t = (t) )
#define pdFilterSetTimeStep(f,dt) ( (f)->_dt = (dt) )

__END_DECLS

#endif /* __PD_FILTER_H__ */
