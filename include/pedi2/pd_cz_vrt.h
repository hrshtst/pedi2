#ifndef __PD_CZ_VRT_H__
#define __PD_CZ_VRT_H__

#include <roki/rk_g.h>
#include <zm/zm.h>

__BEGIN_DECLS

typedef struct{
  double _zd;      /* desired COM position */
  double _q1, _q2; /* system poles */

  double reffz;    /* referential vertical reaction force */
  double refacc;   /* referential COM acceleration */
  double refzmp;   /* referential ZMP position */
  double zeta;
} pdCZVrt;

/* c'tor and d'tor */
__EXPORT void pdCZVrtInit(pdCZVrt *vrt);
__EXPORT void pdCZVrtDestroy(pdCZVrt *vrt);

/* methods to set parameters */
#define pdCZVrtSetRef(v,zd) (v)->_zd = zd
#define pdCZVrtSetQ1(v,q1) (v)->_q1 = q1
#define pdCZVrtSetQ2(v,q2) (v)->_q2 = q2
#define pdCZVrtSetPrm(v,zd,q1,q2) do{\
  pdCZVrtSetRef( v, zd );\
  pdCZVrtSetQ1( v, q1 );\
  pdCZVrtSetQ2( v, q2 );\
} while(0)

/* methods to get parameters */
#define pdCZVrtRef(v) (v)->_zd
#define pdCZVrtQ1(v) (v)->_q1
#define pdCZVrtQ2(v) (v)->_q2
#define pdCZVrtRF(v) (v)->reffz
#define pdCZVrtAcc(v) (v)->refacc
#define pdCZVrtZMP(v) (v)->refzmp
#define pdCZVrtZeta(v) (v)->zeta

/* calculation methods */
__EXPORT double pdCZVrtCalcReactForce(pdCZVrt *vrt, double z, double vz);
__EXPORT double pdCZVrtCalcAcc(pdCZVrt *vrt, double z, double vz);
__EXPORT double pdCZVrtCalcZMP(pdCZVrt *vrt);
__EXPORT double pdCZVrtCalcZeta(pdCZVrt *vrt, double z, double az, double zz );

/* update method */
__EXPORT void pdCZVrtUpdate(pdCZVrt *vrt, double z, double vz, double az, double zz);

__END_DECLS

#endif /* __PD_CZ_VRT_H__ */
