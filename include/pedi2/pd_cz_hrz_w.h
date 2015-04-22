#ifndef __PD_CZ_HRZ_W_H__
#define __PD_CZ_HRZ_W_H__

#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double _vd;       /* desired COM velocity */
  double _q1, _q2;  /* system poles */
  double _rho;      /* degree of activation of nonlinear damping */
  double _kr;       /* degree of initial energy exertion */
  double _dist;     /* canonical distance of feet */
  double *_kappa;   /* curvature of referential orbit */
  double *_zeta;    /* interference from vertical motion */
} pdCZHrzW;

/* c'tor and d'tor */
__EXPORT void pdCZHrzWInit(pdCZHrzW *w, double *kappa, double *zeta);
__EXPORT void pdCZHrzWDestroy(pdCZHrzW *w);

/* methods to set parameters */
#define pdCZHrzWSetRefVel(w,vd) (w)->_vd = vd
#define pdCZHrzWSetQ1(w,q1) (w)->_q1 = q1
#define pdCZHrzWSetQ2(w,q2) (w)->_q2 = q2
#define pdCZHrzWSetRho(w,r) (w)->_rho = r
#define pdCZHrzWSetKr(w,k) (w)->_kr = k
#define pdCZHrzWSetDist(w,d) (w)->_dist = d
#define pdCZHrzWSetKappa(w,k) *((w)->_kappa) = k
#define pdCZHrzWSetPrm(w,vd,q1,q2,rho,kr,dist) do{\
  pdCZHrzWSetRefVel( w, vd );\
  pdCZHrzWSetQ1( w, q1 );\
  pdCZHrzWSetQ2( w, q2 );\
  pdCZHrzWSetRho( w, rho );\
  pdCZHrzWSetKr( w, kr );\
  pdCZHrzWSetDist( w, dist );\
} while(0)

/* methods to get parameters */
#define pdCZHrzWRefVel(w) (w)->_vd
#define pdCZHrzWQ1(w) (w)->_q1
#define pdCZHrzWQ2(w) (w)->_q2
#define pdCZHrzWRho(w) (w)->_rho
#define pdCZHrzWKr(w) (w)->_kr
#define pdCZHrzWDist(w) (w)->_dist
#define pdCZHrzWKappa(w) *((w)->_kappa)
#define pdCZHrzWZeta(w) *((w)->_zeta)

__END_DECLS

#endif /* __PD_CZ_HRZ_W_H__ */
