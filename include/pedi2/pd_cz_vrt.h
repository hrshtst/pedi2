#ifndef __PD_CZ_VRT_H__
#define __PD_CZ_VRT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double _zd;      /* desired COM position */
  double _q1, _q2; /* system poles */
  double mass;     /* mass of the robot */

  double _reffz;   /* referential vertical reaction force */
  double _refacc;  /* referential COM acceleration */

  double zeta;

  double z;      /* deprecated */
} pdCZVrt;

/* c'tor and d'tor */
__EXPORT void pdCZVrtInit(pdCZVrt *vrt);
__EXPORT void pdCZVrtDestroy(pdCZVrt *vrt);

/* methods to set parameters */
#define pdCZVrtSetRef(v,zd) (v)->_zd = zd
#define pdCZVrtSetQ1(v,q1) (v)->_q1 = q1
#define pdCZVrtSetQ2(v,q2) (v)->_q2 = q2
#define pdCZVrtSetMass(v,m) (v)->mass = m
#define pdCZVrtSetPrm(v,zd,q1,q2,m) do{\
  pdCZVrtSetRef( v, zd );\
  pdCZVrtSetQ1( v, q1 );\
  pdCZVrtSetQ2( v, q2 );\
  pdCZVrtSetMass( v, m );\
} while(0)

/* methods to get parameters */
#define pdCZVrtRef(v) (v)->_zd
#define pdCZVrtQ1(v) (v)->_q1
#define pdCZVrtQ2(v) (v)->_q2
#define pdCZVrtMass(v) (v)->mass

__EXPORT double pdCZVrtReactForce(pdCZVrt *vrt, double z, double vz);

void pdCZVrtUpdate(pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_VRT_H__ */
