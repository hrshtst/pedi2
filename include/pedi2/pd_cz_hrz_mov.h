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

  zVec2D zmp;   /* referential ZMP position */
  zVec2D acc;   /* referential COM acceleration */
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
#define pdCZHrzMovZMP(h)     (h)->zmp
#define pdCZHrzMovZMPU(h)    pdCZHrzMovZMP(h)[pdU]
#define pdCZHrzMovZMPW(h)    pdCZHrzMovZMP(h)[pdW]
#define pdCZHrzMovAcc(h)     (h)->acc
#define pdCZHrzMovAccU(h)    pdCZHrzMovAcc(h)[pdU]
#define pdCZHrzMovAccW(h)    pdCZHrzMovAcc(h)[pdW]

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
#define pdCZHrzMovCalcSimZMPU(h,d,v) \
  pdCZHrzUCalcSimZMP( &pdCZHrzMovU(h), d, v )
#define pdCZHrzMovCalcSimZMPW(h,d,v) \
  pdCZHrzWCalcSimZMP( &pdCZHrzMovW(h), d, v )
#define pdCZHrzMovCalcRegZMPU(h,d,v) \
  pdCZHrzUCalcRegZMP( &pdCZHrzMovU(h), d, v )
#define pdCZHrzMovCalcRegZMPW(h,d,v) \
  pdCZHrzWCalcRegZMP( &pdCZHrzMovW(h), d, v )
__EXPORT void pdCZHrzMovCalcZMP(pdCZHrzMov *hrz, zVec2D delta, zVec2D vel, zVec2D zmp);
__EXPORT void pdCZHrzMovCalcAcc(pdCZHrzMov *hrz, zVec2D zmp, zVec2D acc);

/* update method */
__EXPORT void pdCZHrzMovUpdate(pdCZHrzMov *hrz, zVec2D delta, zVec2D vel);

/* for debug */
__EXPORT void pdCZHrzMovFWrite(FILE *fp, pdCZHrzMov *hrz);
#define pdCZHrzMovWrite(h) pdCZHrzMovFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_MOV_H__ */
