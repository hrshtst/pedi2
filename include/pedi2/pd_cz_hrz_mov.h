#ifndef __PD_CZ_HRZ_MOV_H__
#define __PD_CZ_HRZ_MOV_H__

#include <zm/zm.h>
#include <zeo/zeo_bv.h>
#include <pedi2/pd_cz_vrt.h>
#include <pedi2/pd_cz_hrz_u.h>
#include <pedi2/pd_cz_hrz_w.h>

__BEGIN_DECLS

typedef struct{
  pdCZHrzU _u;     /* desired ZMP calculator for tangential dir. */
  pdCZHrzW _w;     /* desired ZMP calculator for radial dir. */
  double _kappa;   /* curvature of the referential orbit */
  pdCZVrt *_vrt;   /* controller for vertical motion */
  zVec3DList _sr;  /* supporting region */
  zVec3D *_sr_vert; /* vertices of supporting retion */
  int _vert_num;    /* number of vertices */

  double refuz;    /* referential ZMP pos. for tangential dir. */
  double refwz;    /* referential ZMP pos. for radial dir. */
  double refddu;   /* referential COM acc. for tangential dir. */
  double refddw;   /* referential COM acc. for radial dir. */
} pdCZHrzMov;

/* c'tor and d'tor */
__EXPORT void pdCZHrzMovInit(pdCZHrzMov *hrz, pdCZVrt *vrt);
__EXPORT void pdCZHrzMovDestroy(pdCZHrzMov *hrz);

/* methods to set parameters */
#define pdCZHrzMovSetRefVelU(h,vd) pdCZHrzUSetRefVel( &(h)->_u, vd )
#define pdCZHrzMovSetQ1U(h,q1) pdCZHrzUSetQ1( &(h)->_u, q1 )
#define pdCZHrzMovSetQ2U(h,q2) pdCZHrzUSetQ2( &(h)->_u, q2 )
#define pdCZHrzMovSetRefVelW(h,vd) pdCZHrzWSetRefVel( &(h)->_w, vd )
#define pdCZHrzMovSetQ1W(h,q1) pdCZHrzWSetQ1( &(h)->_w, q1 )
#define pdCZHrzMovSetQ2W(h,q2) pdCZHrzWSetQ2( &(h)->_w, q2 )
#define pdCZHrzMovSetRho(h,r) pdCZHrzWSetRho( &(h)->_w, r )
#define pdCZHrzMovSetKr(h,k) pdCZHrzWSetKr( &(h)->_w, k )
#define pdCZHrzMovSetDist(h,d) pdCZHrzWSetDist( &(h)->_w, d )
#define pdCZHrzMovSetKappa(h,k) (h)->_kappa = k
#define pdCZHrzMovSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) do{\
  pdCZHrzUSetPrm( &(h)->_u, vud, qu1, qu2 );\
  pdCZHrzWSetPrm( &(h)->_w, vwd, qw1, qw2, rho, kr, dist );\
  pdCZHrzMovSetKappa( h, kappa );\
} while(0)
/* #define pdCZHrzMovSetSR(h,sr) (h)->_sr = sr */
__EXPORT void pdCZHrzMovSetSR(pdCZHrzMov *hrz, zVec3D p[], int num);
#define pdCZHrzMovIsSRSet(h) ( zListNum( &(h)->_sr ) ? true : false )

/* methods to get parameters */
#define pdCZHrzMovRefVelU(h) pdCZHrzURefVel( &(h)->_u )
#define pdCZHrzMovQ1U(h) pdCZHrzUQ1( &(h)->_u )
#define pdCZHrzMovQ2U(h) pdCZHrzUQ2( &(h)->_u )
#define pdCZHrzMovRefVelW(h) pdCZHrzWRefVel( &(h)->_w )
#define pdCZHrzMovQ1W(h) pdCZHrzWQ1( &(h)->_w )
#define pdCZHrzMovQ2W(h) pdCZHrzWQ2( &(h)->_w )
#define pdCZHrzMovRho(h) pdCZHrzWRho( &(h)->_w )
#define pdCZHrzMovKr(h) pdCZHrzWKr( &(h)->_w )
#define pdCZHrzMovDist(h) pdCZHrzWDist( &(h)->_w )
#define pdCZHrzMovKappa(h) (h)->_kappa
#define pdCZHrzMovZeta(h) pdCZVrtZeta( (h)->_vrt )
#define pdCZHrzMovSR(h)     (h)->_sr
#define pdCZHrzMovSRVert(h) (h)->_sr_vert
#define pdCZHrzMovZMPU(h) (h)->refuz
#define pdCZHrzMovZMPW(h) (h)->refwz
#define pdCZHrzMovAccU(h) (h)->refddu
#define pdCZHrzMovAccW(h) (h)->refddw

/* calculation methods */
#define pdCZHrzMovCalcSimZMPU(h,du,vu,dw,wu) \
  pdCZHrzUCalcSimZMP( &(h)->_u, du, vu, dw, wu )
#define pdCZHrzMovCalcSimZMPW(h,du,vu,dw,wu) \
  pdCZHrzWCalcSimZMP( &(h)->_w, du, vu, dw, wu )
#define pdCZHrzMovCalcRegZMPU(h,du,vu,dw,wu) \
  pdCZHrzUCalcRegZMP( &(h)->_u, du, vu, dw, wu )
#define pdCZHrzMovCalcRegZMPW(h,du,vu,dw,wu) \
  pdCZHrzWCalcRegZMP( &(h)->_w, du, vu, dw, wu )
__EXPORT void pdCZHrzMovCalcZMP(pdCZHrzMov *hrz, double du, double vu, double dw, double vw, double *uz, double *wz);
__EXPORT void pdCZHrzMovCalcAcc(pdCZHrzMov *hrz, double uz, double wz, double *ddu, double *ddw);

/* update method */
__EXPORT void pdCZHrzMovUpdate(pdCZHrzMov *hrz, double du, double vu, double dw, double vw);


__END_DECLS

#endif /* __PD_CZ_HRZ_MOV_H__ */
