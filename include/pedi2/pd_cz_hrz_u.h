#ifndef __PD_CZ_HRZ_U_H__
#define __PD_CZ_HRZ_U_H__

#include <cure/cure_defs.h>

__BEGIN_DECLS

typedef struct{
  double _vd;      /* referential COM velocity */
  double _q1, _q2; /* system poles */
  double *_kappa;  /* curvature of referential orbit */
  double *_zeta;   /* interference from vertical motion */

  double refzmp;   /* referential ZMP position */
  double refacc;   /* referential COM acceleration */
} pdCZHrzU;

/* c'tor and d'tor */
__EXPORT void pdCZHrzUInit(pdCZHrzU *u, double *kappa, double *zeta);
__EXPORT void pdCZHrzUDestroy(pdCZHrzU *u);

/* methods to set parameters */
#define pdCZHrzUSetRefVel(u,vd) (u)->_vd = vd
#define pdCZHrzUSetQ1(u,q1) (u)->_q1 = q1
#define pdCZHrzUSetQ2(u,q2) (u)->_q2 = q2
#define pdCZHrzUSetKappa(u,k) *((u)->_kappa) = k
#define pdCZHrzUSetPrm(u,vd,q1,q2) do{\
  pdCZHrzUSetRefVel( u, vd );\
  pdCZHrzUSetQ1( u, q1 );\
  pdCZHrzUSetQ2( u, q2 );\
} while(0)

/* methods to get parameters */
#define pdCZHrzURefVel(u) (u)->_vd
#define pdCZHrzUQ1(u) (u)->_q1
#define pdCZHrzUQ2(u) (u)->_q2
#define pdCZHrzUKappa(u) *((u)->_kappa)
#define pdCZHrzUZeta(u) *((u)->_zeta)

__END_DECLS

#endif /* __PD_CZ_HRZ_U_H__ */
