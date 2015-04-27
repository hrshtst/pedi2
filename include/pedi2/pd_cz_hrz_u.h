#ifndef __PD_CZ_HRZ_U_H__
#define __PD_CZ_HRZ_U_H__

#include <zm/zm.h>
#include <zeo/zeo_vec2d.h>
#include <pedi2/pd_misc.h>

__BEGIN_DECLS

typedef struct{
  double _vd;      /* referential COM velocity */
  double _q1, _q2; /* system poles */
  double *_kappa;  /* curvature of referential orbit */
  double *_zeta;   /* interference from vertical motion */
} pdCZHrzU;

/* c'tor and d'tor */
__EXPORT void pdCZHrzUInit(pdCZHrzU *u, double *kappa, double *zeta);
__EXPORT void pdCZHrzUDestroy(pdCZHrzU *u);

/* methods to get parameters */
#define pdCZHrzURefVel(u) (u)->_vd
#define pdCZHrzUQ1(u)     (u)->_q1
#define pdCZHrzUQ2(u)     (u)->_q2
#define pdCZHrzUKappa(u)  *((u)->_kappa)
#define pdCZHrzUZeta(u)   *((u)->_zeta)

/* methods to set parameters */
#define pdCZHrzUSetRefVel(u,vd) pdCZHrzURefVel(u) = vd
#define pdCZHrzUSetQ1(u,q1)     pdCZHrzUQ1(u) = q1
#define pdCZHrzUSetQ2(u,q2)     pdCZHrzUQ2(u) = q2
#define pdCZHrzUSetKappa(u,k)   pdCZHrzUKappa(u) = k
#define pdCZHrzUSetPrm(u,vd,q1,q2) do{\
  pdCZHrzUSetRefVel( u, vd );\
  pdCZHrzUSetQ1( u, q1 );\
  pdCZHrzUSetQ2( u, q2 );\
} while(0)

/* calculation methods */
__EXPORT double pdCZHrzUCalcSimZMP(pdCZHrzU *u, zVec2D delta, zVec2D vel);
__EXPORT double pdCZHrzUCalcRegZMP(pdCZHrzU *u, zVec2D delta, zVec2D vel);


__END_DECLS

#endif /* __PD_CZ_HRZ_U_H__ */
