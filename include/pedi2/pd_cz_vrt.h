#ifndef __PD_CZ_VRT_H__
#define __PD_CZ_VRT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double zd;     /* desired COM position */
  double q1, q2; /* system poles */
  double m;      /* mass of robot */
  double fz;     /* vertical reaction force */

  double acc;    /* COM acceleration */
  double zeta;

  double z;      /* deprecated */
} pdCZVrt;

void pdCZVrtInit(pdCZVrt *vrt);
void pdCZVrtDestroy(pdCZVrt *vrt);

void pdCZVrtSetRef(pdCZVrt *vrt, double zd);
void pdCZVrtUpdate(pdCZVrt *vrt);

__END_DECLS

#endif /* __PD_CZ_VRT_H__ */
