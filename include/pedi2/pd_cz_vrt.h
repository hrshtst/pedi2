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
void pdCZVrtInit(pdCZVrt *vrt);
void pdCZVrtDestroy(pdCZVrt *vrt);

/* methods to set parameters */
#define pdCZVrtSetRef(v,zd) (v)->_zd = zd
#define pdCZVrtSetMass(v,m) (v)->mass = m
#define pdCZVrtSetQ1(v,q1) (v)->_q1 = q1
#define pdCZVrtSetQ2(v,q2) (v)->_q2 = q2

/* methods to get parameters */
#define pdCZVrtGetRef(v) (v)->_zd
#define pdCZVrtGetMass(v) (v)->mass
#define pdCZVrtGetQ1(v) (v)->_q1
#define pdCZVrtGetQ2(v) (v)->_q2


void pdCZVrtUpdate(pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_VRT_H__ */
