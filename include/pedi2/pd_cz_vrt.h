#ifndef __PD_CZ_VRT_H__
#define __PD_CZ_VRT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double zd;     /* desired COM position */
  double q1, q2; /* system poles */
  double mass;   /* mass of robot */

  double fz;     /* vertical reaction force */
  double acc;    /* COM acceleration */

  double zeta;

  double z;      /* deprecated */
} pdCZVrt;

/* c'tor and d'tor */
void pdCZVrtInit(pdCZVrt *vrt);
void pdCZVrtDestroy(pdCZVrt *vrt);

/* methods to set parameters */
#define pdCZVrtSetRef(v,z) (v)->zd = z
#define pdCZVrtSetMass(v,m) (v)->mass = m
#define pdCZVrtSetQ1(v,q) (v)->q1 = q
#define pdCZVrtSetQ2(v,q) (v)->q2 = q

/* methods to get parameters */
#define pdCZVrtGetRef(v) (v)->zd
#define pdCZVrtGetMass(v) (v)->mass
#define pdCZVrtGetQ1(v) (v)->q1
#define pdCZVrtGetQ2(v) (v)->q2


void pdCZVrtUpdate(pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_VRT_H__ */
