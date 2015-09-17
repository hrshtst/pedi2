#ifndef __PD_FILTER_H__
#define __PD_FILTER_H__

#include <dzco/dz_sys.h>

__BEGIN_DECLS

typedef struct{
  double _t;
  double _dt;
} pdFilter;

/* c'tor and d'tor */
__EXPORT void pdFilterInit(pdFilter *filter, double dt);
__EXPORT void pdFilterDestroy(pdFilter *filter);

/* methods */
#define pdFilterTime(f)     (f)->_t
#define pdFilterTimeStep(f) (f)->_dt

#define pdFilterSetTime(f,t)      ( (f)->_t = (t) )
#define pdFilterSetTimeStep(f,dt) ( (f)->_dt = (dt) )

__END_DECLS

#endif /* __PD_FILTER_H__ */
