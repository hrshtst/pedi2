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

/* methods to get parameters */
#define pdCZHrzMovU(h)       (h)->_u
#define pdCZHrzMovW(h)       (h)->_w
#define pdCZHrzMovRefVelU(h) pdCZHrzURefVel( &pdCZHrzMovU(h) )
#define pdCZHrzMovQ1U(h)     pdCZHrzUQ1( &pdCZHrzMovU(h) )
#define pdCZHrzMovQ2U(h)     pdCZHrzUQ2( &pdCZHrzMovU(h) )
#define pdCZHrzMovRefVelW(h) pdCZHrzWRefVel( &pdCZHrzMovW(h) )
#define pdCZHrzMovQ1W(h)     pdCZHrzWQ1( &pdCZHrzMovW(h) )
#define pdCZHrzMovQ2W(h)     pdCZHrzWQ2( &pdCZHrzMovW(h) )
#define pdCZHrzMovRho(h)     pdCZHrzWRho( &pdCZHrzMovW(h) )
#define pdCZHrzMovKr(h)      pdCZHrzWKr( &pdCZHrzMovW(h) )
#define pdCZHrzMovDist(h)    pdCZHrzWDist( &pdCZHrzMovW(h) )
#define pdCZHrzMovKappa(h)   (h)->_kappa
#define pdCZHrzMovZeta(h)    pdCZVrtZeta( (h)->_vrt )
#define pdCZHrzMovSR(h)      (h)->_sr
#define pdCZHrzMovSRVert(h)  (h)->_sr_vert
#define pdCZHrzMovZMPU(h)    (h)->refuz
#define pdCZHrzMovZMPW(h)    (h)->refwz
#define pdCZHrzMovAccU(h)    (h)->refddu
#define pdCZHrzMovAccW(h)    (h)->refddw

/* methods to set parameters */
#define pdCZHrzMovSetRefVelU(h,vd) pdCZHrzUSetRefVel( &pdCZHrzMovU(h), vd )
#define pdCZHrzMovSetQ1U(h,q1)     pdCZHrzUSetQ1( &pdCZHrzMovU(h), q1 )
#define pdCZHrzMovSetQ2U(h,q2)     pdCZHrzUSetQ2( &pdCZHrzMovU(h), q2 )
#define pdCZHrzMovSetRefVelW(h,vd) pdCZHrzWSetRefVel( &pdCZHrzMovW(h), vd )
#define pdCZHrzMovSetQ1W(h,q1)     pdCZHrzWSetQ1( &pdCZHrzMovW(h), q1 )
#define pdCZHrzMovSetQ2W(h,q2)     pdCZHrzWSetQ2( &pdCZHrzMovW(h), q2 )
#define pdCZHrzMovSetRho(h,r)      pdCZHrzWSetRho( &pdCZHrzMovW(h), r )
#define pdCZHrzMovSetKr(h,k)       pdCZHrzWSetKr( &pdCZHrzMovW(h), k )
#define pdCZHrzMovSetDist(h,d)     pdCZHrzWSetDist( &pdCZHrzMovW(h), d )
#define pdCZHrzMovSetKappa(h,k)    pdCZHrzMovKappa(h) = k
#define pdCZHrzMovSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) do{\
  pdCZHrzUSetPrm( &pdCZHrzMovU(h), vud, qu1, qu2 );\
  pdCZHrzWSetPrm( &pdCZHrzMovW(h), vwd, qw1, qw2, rho, kr, dist );\
  pdCZHrzMovSetKappa( h, kappa );\
} while(0)
__EXPORT void pdCZHrzMovSetSR(pdCZHrzMov *hrz, zVec3D p[], int num);
#define pdCZHrzMovIsSRSet(h) ( zListNum( &(h)->_sr ) ? true : false )

/* calculation methods */
#define pdCZHrzMovCalcSimZMPU(h,du,vu,dw,wu) \
  pdCZHrzUCalcSimZMP( &pdCZHrzMovU(h), du, vu, dw, wu )
#define pdCZHrzMovCalcSimZMPW(h,du,vu,dw,wu) \
  pdCZHrzWCalcSimZMP( &pdCZHrzMovW(h), du, vu, dw, wu )
#define pdCZHrzMovCalcRegZMPU(h,du,vu,dw,wu) \
  pdCZHrzUCalcRegZMP( &pdCZHrzMovU(h), du, vu, dw, wu )
#define pdCZHrzMovCalcRegZMPW(h,du,vu,dw,wu) \
  pdCZHrzWCalcRegZMP( &pdCZHrzMovW(h), du, vu, dw, wu )
__EXPORT void pdCZHrzMovCalcZMP(pdCZHrzMov *hrz, double du, double vu, double dw, double vw, double *uz, double *wz);
__EXPORT void pdCZHrzMovCalcAcc(pdCZHrzMov *hrz, double uz, double wz, double *ddu, double *ddw);

/* update method */
__EXPORT void pdCZHrzMovUpdate(pdCZHrzMov *hrz, double du, double vu, double dw, double vw);

/* for debug */
__EXPORT void pdCZHrzMovFWrite(FILE *fp, pdCZHrzMov *hrz);
#define pdCZHrzMovWrite(h) pdCZHrzMovFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_MOV_H__ */
