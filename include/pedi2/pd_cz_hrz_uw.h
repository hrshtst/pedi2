#ifndef __PD_CZ_HRZ_UW_H__
#define __PD_CZ_HRZ_UW_H__

#include <zm/zm.h>
#include <zeo/zeo_vec3d.h>
#include <zeo/zeo_bv2d.h>
#include <zm/zm_complex.h>
#include <pedi2/pd_cz_vrt.h>
#include <pedi2/pd_cz_hrz_u.h>
#include <pedi2/pd_cz_hrz_w.h>

__BEGIN_DECLS

typedef struct{
  pdCZHrzU _u;         /* desired ZMP calculator for tangential direction */
  pdCZHrzW _w;         /* desired ZMP calculator for radial direction */
  double _kappa;       /* curvature of the referential orbit */
  pdCZVrt *_vrt;       /* controller for vertical motion */
  zLoop3D _sr;         /* supporting region */
  zVec3DData _sr_vert; /* vertices of supporting region */
  int _vert_num;       /* number of vertices */

  zVec2D zmp;   /* referential ZMP position */
  zVec2D acc;   /* referential COM acceleration */
} pdCZHrzUW;

/* c'tor and d'tor */
__EXPORT void pdCZHrzUWInit(pdCZHrzUW *hrz, pdCZVrt *vrt);
__EXPORT void pdCZHrzUWDestroy(pdCZHrzUW *hrz);

/* methods to get parameters */
#define pdCZHrzUPtr(h)      ( &(h)->_u )
#define pdCZHrzWPtr(h)      ( &(h)->_w )
#define pdCZHrzUWRefVelU(h) pdCZHrzURefVel( pdCZHrzUPtr(h) )
#define pdCZHrzUWQ1U(h)     pdCZHrzUQ1( pdCZHrzUPtr(h) )
#define pdCZHrzUWQ2U(h)     pdCZHrzUQ2( pdCZHrzUPtr(h) )
#define pdCZHrzUWRefVelW(h) pdCZHrzWRefVel( pdCZHrzWPtr(h) )
#define pdCZHrzUWQ1W(h)     pdCZHrzWQ1( pdCZHrzWPtr(h) )
#define pdCZHrzUWQ2W(h)     pdCZHrzWQ2( pdCZHrzWPtr(h) )
#define pdCZHrzUWRho(h)     pdCZHrzWRho( pdCZHrzWPtr(h) )
#define pdCZHrzUWKr(h)      pdCZHrzWKr( pdCZHrzWPtr(h) )
#define pdCZHrzUWDist(h)    pdCZHrzWDist( pdCZHrzWPtr(h) )
#define pdCZHrzUWKappa(h)   (h)->_kappa
#define pdCZHrzUWZeta(h)    pdCZVrtZeta( (h)->_vrt )
#define pdCZHrzUWSR(h)      ( &(h)->_sr )
#define pdCZHrzUWSRVert(h)  ( &(h)->_sr_vert )
#define pdCZHrzUWZMP(h)     ( &(h)->zmp )
#define pdCZHrzUWZMPU(h)    ( pdCZHrzUWZMP(h)->e[pdU] )
#define pdCZHrzUWZMPW(h)    ( pdCZHrzUWZMP(h)->e[pdW] )
#define pdCZHrzUWAcc(h)     ( &(h)->acc )
#define pdCZHrzUWAccU(h)    ( pdCZHrzUWAcc(h)->e[pdU] )
#define pdCZHrzUWAccW(h)    ( pdCZHrzUWAcc(h)->e[pdW] )

/* methods to set parameters */
#define pdCZHrzUWSetRefVelU(h,vd) pdCZHrzUSetRefVel( pdCZHrzUPtr(h), vd )
#define pdCZHrzUWSetQ1U(h,q1)     pdCZHrzUSetQ1( pdCZHrzUPtr(h), q1 )
#define pdCZHrzUWSetQ2U(h,q2)     pdCZHrzUSetQ2( pdCZHrzUPtr(h), q2 )
#define pdCZHrzUWSetRefVelW(h,vd) pdCZHrzWSetRefVel( pdCZHrzWPtr(h), vd )
#define pdCZHrzUWSetQ1W(h,q1)     pdCZHrzWSetQ1( pdCZHrzWPtr(h), q1 )
#define pdCZHrzUWSetQ2W(h,q2)     pdCZHrzWSetQ2( pdCZHrzWPtr(h), q2 )
#define pdCZHrzUWSetRho(h,r)      pdCZHrzWSetRho( pdCZHrzWPtr(h), r )
#define pdCZHrzUWSetKr(h,k)       pdCZHrzWSetKr( pdCZHrzWPtr(h), k )
#define pdCZHrzUWSetDist(h,d)     pdCZHrzWSetDist( pdCZHrzWPtr(h), d )
#define pdCZHrzUWSetKappa(h,k)    ( pdCZHrzUWKappa(h) = (k) )
#define pdCZHrzUWSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) do{\
  pdCZHrzUSetPrm( pdCZHrzUPtr(h), vud, qu1, qu2 );\
  pdCZHrzWSetPrm( pdCZHrzWPtr(h), vwd, qw1, qw2, rho, kr, dist );\
  pdCZHrzUWSetKappa( h, kappa );\
} while(0)
__EXPORT void pdCZHrzUWSetSR(pdCZHrzUW *hrz, zVec3D p[], int num);
#define pdCZHrzUWIsSRSet(h) ( zListSize( pdCZHrzUWSR(h) ) ? true : false )

/* calculation methods */
#define pdCZHrzUWCalcSimZMPU(h,d,v) \
  pdCZHrzUCalcSimZMP( pdCZHrzUPtr(h), d, v )
#define pdCZHrzUWCalcSimZMPW(h,d,v) \
  pdCZHrzWCalcSimZMP( pdCZHrzWPtr(h), d, v )
#define pdCZHrzUWCalcRegZMPU(h,d,v) \
  pdCZHrzUCalcRegZMP( pdCZHrzUPtr(h), d, v )
#define pdCZHrzUWCalcRegZMPW(h,d,v) \
  pdCZHrzWCalcRegZMP( pdCZHrzWPtr(h), d, v )
__EXPORT void pdCZHrzUWCalcZMP(pdCZHrzUW *hrz, zVec2D *delta, zVec2D *vel, zVec2D *zmp);
#define pdCZHrzUWCalcRegZMP(h,d,v,z) \
  zVec2DCreate( z, pdCZHrzUWCalcRegZMPU( h, d, v ), pdCZHrzUWCalcRegZMPW( h, d, v ) )
__EXPORT void pdCZHrzUWCalcAcc(pdCZHrzUW *hrz, zVec2D *zmp, zVec2D *acc);
__EXPORT void pdCZHrzUWCalcZMPPhase(pdCZHrzUW *hrz, zVec2D *delta, zVec2D *vel, zVec2D *zmp, zComplex *pz);

/* update method */
__EXPORT void pdCZHrzUWUpdate(pdCZHrzUW *hrz, zVec2D *delta, zVec2D *vel);

/* for debug */
__EXPORT void pdCZHrzUWFWrite(FILE *fp, pdCZHrzUW *hrz);
#define pdCZHrzUWWrite(h) pdCZHrzUWFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_UW_H__ */
