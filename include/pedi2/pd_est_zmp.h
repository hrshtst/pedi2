#ifndef __PD_EST_ZMP_H__
#define __PD_EST_ZMP_H__

#include <zeo/zeo_vec3d.h>

__BEGIN_DECLS

typedef struct{
  char name[BUFSIZ];
  int dim;
  double freq;
} filter_t;

typedef struct{
  char name[BUFSIZ];
  char type[BUFSIZ];
} sensor_t;

typedef struct{
  double _t;     /* time */
  double _dt;    /* time step */

  filter_t filter[2];
  sensor_t sensor[2];
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

__EXPORT bool pdEstZMPConfFRead(FILE *fp, pdEstZMP *e_zmp);
__EXPORT bool pdEstZMPConfReadFile(pdEstZMP *e_zmp, const char *filename);

__END_DECLS

#endif /* __PD_EST_ZMP_H__ */
