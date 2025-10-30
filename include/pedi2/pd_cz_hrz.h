#ifndef __PD_CZ_HRZ_H__
#define __PD_CZ_HRZ_H__

#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

typedef struct{
  zVec2D _xyd;     /* desired COM position */
  double _thetad;  /* desired rotational angle */
  zVec2D _xy;      /* COM position */
  zVec2D _vxy;     /* COM velocity */
  double _theta;   /* rotational angle */
  zLoop3D *_sr;    /* supporting region */

  pdCZHrzUW _uw;   /* horizontal controller w.r.t moving frame */
  zVec2D _uwd;     /* desired COM position w.r.t moving frame */
  zVec2D _delta;   /* difference from COM to desired position */
  zVec2D _vuw;     /* COM velocity w.r.t moving frame */

  zVec2D zmp;      /* referential ZMP position */
  zVec2D acc;      /* referential COM acceleration */
} pdCZHrz;

/* c'tor and d'tor */
__PEDI2_EXPORT void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt);
__PEDI2_EXPORT void pdCZHrzDestroy(pdCZHrz *hrz);

/* methods to get parameters */
#define pdCZHrzRefPos(h)   ( &(h)->_xyd )
#define pdCZHrzRefPosX(h)  ( pdCZHrzRefPos(h)->c.x )
#define pdCZHrzRefPosY(h)  ( pdCZHrzRefPos(h)->c.y )
#define pdCZHrzRefTheta(h) (h)->_thetad
#define pdCZHrzPos(h)      ( &(h)->_xy )
#define pdCZHrzPosX(h)     ( pdCZHrzPos(h)->c.x )
#define pdCZHrzPosY(h)     ( pdCZHrzPos(h)->c.y )
#define pdCZHrzVel(h)      ( &(h)->_vxy )
#define pdCZHrzVelX(h)     ( pdCZHrzVel(h)->c.x )
#define pdCZHrzVelY(h)     ( pdCZHrzVel(h)->c.y )
#define pdCZHrzTheta(h)    (h)->_theta
#define pdCZHrzUWPtr(h)    ( &(h)->_uw )
#define pdCZHrzRefVelU(h)  pdCZHrzUWRefVelU( pdCZHrzUWPtr(h) )
#define pdCZHrzQ1U(h)      pdCZHrzUWQ1U( pdCZHrzUWPtr(h) )
#define pdCZHrzQ2U(h)      pdCZHrzUWQ2U( pdCZHrzUWPtr(h) )
#define pdCZHrzRefVelW(h)  pdCZHrzUWRefVelW( pdCZHrzUWPtr(h) )
#define pdCZHrzQ1W(h)      pdCZHrzUWQ1W( pdCZHrzUWPtr(h) )
#define pdCZHrzQ2W(h)      pdCZHrzUWQ2W( pdCZHrzUWPtr(h) )
#define pdCZHrzRho(h)      pdCZHrzUWRho( pdCZHrzUWPtr(h) )
#define pdCZHrzKr(h)       pdCZHrzUWKr( pdCZHrzUWPtr(h) )
#define pdCZHrzDist(h)     pdCZHrzUWDist( pdCZHrzUWPtr(h) )
#define pdCZHrzKappa(h)    pdCZHrzUWKappa( pdCZHrzUWPtr(h) )
#define pdCZHrzSRUW(h)     pdCZHrzUWSR( pdCZHrzUWPtr(h) )
#define pdCZHrzSRVertUW(h) pdCZHrzUWSRVert( pdCZHrzUWPtr(h) )
#define pdCZHrzZMPUW(h)    pdCZHrzUWZMP( pdCZHrzUWPtr(h) )
#define pdCZHrzZMPU(h)     pdCZHrzUWZMPU( pdCZHrzUWPtr(h) )
#define pdCZHrzZMPW(h)     pdCZHrzUWZMPW( pdCZHrzUWPtr(h) )
#define pdCZHrzAccUW(h)    pdCZHrzUWAcc( pdCZHrzUWPtr(h) )
#define pdCZHrzAccU(h)     pdCZHrzUWAccU( pdCZHrzUWPtr(h) )
#define pdCZHrzAccW(h)     pdCZHrzUWAccW( pdCZHrzUWPtr(h) )
#define pdCZHrzSR(h)       (h)->_sr
#define pdCZHrzZMP(h)      ( &(h)->zmp )
#define pdCZHrzZMPX(h)     ( pdCZHrzZMP(h)->c.x )
#define pdCZHrzZMPY(h)     ( pdCZHrzZMP(h)->c.y )
#define pdCZHrzAcc(h)      ( &(h)->acc )
#define pdCZHrzAccX(h)     ( pdCZHrzAcc(h)->c.x )
#define pdCZHrzAccY(h)     ( pdCZHrzAcc(h)->c.y )
#define pdCZHrzRefPosUW(h) ( &(h)->_uwd )
#define pdCZHrzRefPosU(h)  ( pdCZHrzRefPosUW(h)->e[pdU] )
#define pdCZHrzRefPosW(h)  ( pdCZHrzRefPosUW(h)->e[pdW] )
#define pdCZHrzDelta(h)    ( &(h)->_delta )
#define pdCZHrzDeltaU(h)   ( pdCZHrzDelta(h)->e[pdU] )
#define pdCZHrzDeltaW(h)   ( pdCZHrzDelta(h)->e[pdW] )
#define pdCZHrzVelUW(h)    ( &(h)->_vuw )
#define pdCZHrzVelU(h)     ( pdCZHrzVelUW(h)->e[pdU] )
#define pdCZHrzVelW(h)     ( pdCZHrzVelUW(h)->e[pdW] )

/* methods to set parameters */
#define pdCZHrzSetRefPos(h,xd,yd)   zVec2DCreate( pdCZHrzRefPos(h), xd, yd )
#define pdCZHrzSetRefPosVec(h,pd)   zVec2DCopy( pd, pdCZHrzRefPos(h) )
#define pdCZHrzSetRefPosX(h,xd)     ( pdCZHrzRefPosX(h) = (xd) )
#define pdCZHrzSetRefPosY(h,yd)     ( pdCZHrzRefPosY(h) = (yd) )
#define pdCZHrzSetRefTheta(h,td)    ( pdCZHrzRefTheta(h) = (td) )
#define pdCZHrzSetPos(h,x,y)        zVec2DCreate( pdCZHrzPos(h), x, y )
#define pdCZHrzSetPosVec(h,p)       zVec2DCopy( p, pdCZHrzPos(h) )
#define pdCZHrzSetPosX(h,x)         ( pdCZHrzPosX(h) = (x) )
#define pdCZHrzSetPosY(h,y)         ( pdCZHrzPosY(h) = (y) )
#define pdCZHrzSetTheta(h,t)        ( pdCZHrzTheta(h) = (t) )
#define pdCZHrzSetVel(h,vx,vy)      zVec2DCreate( pdCZHrzVel(h), vx, vy )
#define pdCZHrzSetVelVec(h,v)       zVec2DCopy( v, pdCZHrzVel(h) )
#define pdCZHrzSetVelX(h,vx)        ( pdCZHrzVelX(h) = (vx) )
#define pdCZHrzSetVelY(h,vy)        ( pdCZHrzVelY(h) = (vy) )
#define pdCZHrzSetRefVelU(h,vd)     pdCZHrzUWSetRefVelU( pdCZHrzUWPtr(h), vd )
#define pdCZHrzSetQ1U(h,q1)         pdCZHrzUWSetQ1U( pdCZHrzUWPtr(h), q1 )
#define pdCZHrzSetQ2U(h,q2)         pdCZHrzUWSetQ2U( pdCZHrzUWPtr(h), q2 )
#define pdCZHrzSetRefVelW(h,vd)     pdCZHrzUWSetRefVelW( pdCZHrzUWPtr(h), vd )
#define pdCZHrzSetQ1W(h,q1)         pdCZHrzUWSetQ1W( pdCZHrzUWPtr(h), q1 )
#define pdCZHrzSetQ2W(h,q2)         pdCZHrzUWSetQ2W( pdCZHrzUWPtr(h), q2 )
#define pdCZHrzSetRho(h,r)          pdCZHrzUWSetRho( pdCZHrzUWPtr(h), r )
#define pdCZHrzSetKr(h,k)           pdCZHrzUWSetKr( pdCZHrzUWPtr(h), k )
#define pdCZHrzSetDist(h,d)         pdCZHrzUWSetDist( pdCZHrzUWPtr(h), d )
#define pdCZHrzSetKappa(h,k)        pdCZHrzUWSetKappa( pdCZHrzUWPtr(h), k )
#define pdCZHrzSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) \
  pdCZHrzUWSetPrm( pdCZHrzUWPtr(h), vud, qu1, qu2, \
                                    vwd, qw1, qw2, rho, kr, dist, kappa )
#define pdCZHrzSetSR(h,sr)          ( pdCZHrzSR(h) = (sr) )
#define pdCZHrzSetRefPosUW(h,ud,wd) zVec2DCreate( pdCZHrzRefPosUW(h), ud, wd )
#define pdCZHrzSetRefPosUWVec(h,pd) zVec2DCopy( pd, pdCZHrzRefPosUW(h) )
#define pdCZHrzSetRefPosU(h,ud)     ( pdCZHrzRefPosU(h) = (ud) )
#define pdCZHrzSetRefPosW(h,wd)     ( pdCZHrzRefPosW(h) = (wd) )
#define pdCZHrzSetDelta(h,du,dw)    zVec2DCreate( pdCZHrzDelta(h), du, dw )
#define pdCZHrzSetDeltaVec(h,d)     zVec2DCopy( d, pdCZHrzDelta(h) )
#define pdCZHrzSetDeltaU(h,du)      ( pdCZHrzDeltaU(h) = (du) )
#define pdCZHrzSetDeltaW(h,dw)      ( pdCZHrzDeltaW(h) = (dw) )
#define pdCZHrzSetVelUW(h,vu,vw)    zVec2DCreate( pdCZHrzVelUW(h), vu, vw )
#define pdCZHrzSetVelUWVec(h,v)     zVec2DCopy( v, pdCZHrzVelUW(h) )
#define pdCZHrzSetVelU(h,vu)        ( pdCZHrzVelU(h) = (vu) )
#define pdCZHrzSetVelW(h,vw)        ( pdCZHrzVelW(h) = (vw) )

/* calculation method */
__PEDI2_EXPORT zVec2D *pdCZHrzRotUWtoXY(pdCZHrz *hrz, zVec2D *vuw, zVec2D *vxy);
__PEDI2_EXPORT zVec2D *pdCZHrzRotXYtoUW(pdCZHrz *hrz, zVec2D *vxy, zVec2D *vuw);
__PEDI2_EXPORT zVec2D *pdCZHrzXformUWtoXY(pdCZHrz *hrz, zVec2D *uw, zVec2D *xy);
__PEDI2_EXPORT zVec2D *pdCZHrzXformXYtoUW(pdCZHrz *hrz, zVec2D *xy, zVec2D *uw);
__PEDI2_EXPORT void pdCZHrzXformSRXYtoUW(pdCZHrz *hrz, zLoop3D *sr);
__PEDI2_EXPORT void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, zVec2D *uwd, zVec2D *delta);

/* update method */
#define pdCZHrzUpdateUW(h,d,v) pdCZHrzUWUpdate( pdCZHrzUWPtr(h), d, v )
__PEDI2_EXPORT void pdCZHrzUpdateAcc(pdCZHrz *hrz, zVec2D *xy, double theta, zVec2D *vxy, zVec2D *xyd, double thetad, zLoop3D *sr);

/* for debug */
__PEDI2_EXPORT void pdCZHrzFWrite(FILE *fp, pdCZHrz *hrz);
#define pdCZHrzWrite(h) pdCZHrzFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_H__ */
