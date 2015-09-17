#ifndef __PD_EST_ZMP_H__
#define __PD_EST_ZMP_H__

#include <zeo/zeo_vec3d.h>

__BEGIN_DECLS

typedef struct{
  double _t;     /* time */
  double _dt;    /* time step */
} pdEstZMP;

/* c'tor and d'tor */
__EXPORT void pdEstZMPInit(pdEstZMP *e_zmp, double dt);
__EXPORT void pdEstZMPDestroy(pdEstZMP *e_zmp);

/* methods to get parameters */
#define pdEstZMPTime(e)     (e)->_t
#define pdEstZMPTimeStep(e) (e)->_dt

/* methods to set parameters */
#define pdEstZMPSetTime(e,t)      ( (e)->_t = (t) )
#define pdEstZMPSetTimeStep(e,dt) ( (e)->_dt = (dt) )

__END_DECLS

#endif /* __PD_EST_ZMP_H__ */
