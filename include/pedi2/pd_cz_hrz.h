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
  zVec3DList *_sr; /* supporting region */

  pdCZHrzUW _uw;   /* horizontal controller w.r.t moving frame */
  zVec2D _uwd;     /* desired COM position w.r.t moving frame */
  zVec2D _delta;   /* difference from COM to desired position */
  zVec2D _vuw;     /* COM velocity w.r.t moving frame */

  zVec2D zmp;      /* referential ZMP position */
  zVec2D acc;      /* referential COM acceleration */
} pdCZHrz;

/* c'tor and d'tor */
__EXPORT void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt);
__EXPORT void pdCZHrzDestroy(pdCZHrz *hrz);

/* methods to get parameters */
#define pdCZHrzRefPos(h)   (h)->_xyd
#define pdCZHrzRefPosX(h)  pdCZHrzRefPos(h)[zX]
#define pdCZHrzRefPosY(h)  pdCZHrzRefPos(h)[zY]
#define pdCZHrzRefTheta(h) (h)->_thetad
#define pdCZHrzPos(h)      (h)->_xy
#define pdCZHrzPosX(h)     pdCZHrzPos(h)[zX]
#define pdCZHrzPosY(h)     pdCZHrzPos(h)[zY]
#define pdCZHrzVel(h)      (h)->_vxy
#define pdCZHrzVelX(h)     pdCZHrzVel(h)[zX]
#define pdCZHrzVelY(h)     pdCZHrzVel(h)[zY]
#define pdCZHrzTheta(h)    (h)->_theta
#define pdCZHrzUW(h)       (h)->_uw
#define pdCZHrzRefVelU(h)  pdCZHrzUWRefVelU( &pdCZHrzUW(h) )
#define pdCZHrzQ1U(h)      pdCZHrzUWQ1U( &pdCZHrzUW(h) )
#define pdCZHrzQ2U(h)      pdCZHrzUWQ2U( &pdCZHrzUW(h) )
#define pdCZHrzRefVelW(h)  pdCZHrzUWRefVelW( &pdCZHrzUW(h) )
#define pdCZHrzQ1W(h)      pdCZHrzUWQ1W( &pdCZHrzUW(h) )
#define pdCZHrzQ2W(h)      pdCZHrzUWQ2W( &pdCZHrzUW(h) )
#define pdCZHrzRho(h)      pdCZHrzUWRho( &pdCZHrzUW(h) )
#define pdCZHrzKr(h)       pdCZHrzUWKr( &pdCZHrzUW(h) )
#define pdCZHrzDist(h)     pdCZHrzUWDist( &pdCZHrzUW(h) )
#define pdCZHrzKappa(h)    pdCZHrzUWKappa( &pdCZHrzUW(h) )
#define pdCZHrzSRUW(h)     pdCZHrzUWSR( &pdCZHrzUW(h) )
#define pdCZHrzSRVertUW(h) pdCZHrzUWSRVert( &pdCZHrzUW(h) )
#define pdCZHrzZMPUW(h)    pdCZHrzUWZMP( &pdCZHrzUW(h) )
#define pdCZHrzZMPU(h)     pdCZHrzUWZMPU( &pdCZHrzUW(h) )
#define pdCZHrzZMPW(h)     pdCZHrzUWZMPW( &pdCZHrzUW(h) )
#define pdCZHrzAccUW(h)    pdCZHrzUWAcc( &pdCZHrzUW(h) )
#define pdCZHrzAccU(h)     pdCZHrzUWAccU( &pdCZHrzUW(h) )
#define pdCZHrzAccW(h)     pdCZHrzUWAccW( &pdCZHrzUW(h) )
#define pdCZHrzSR(h)       (h)->_sr
#define pdCZHrzZMP(h)      (h)->zmp
#define pdCZHrzZMPX(h)     pdCZHrzZMP(h)[zX]
#define pdCZHrzZMPY(h)     pdCZHrzZMP(h)[zY]
#define pdCZHrzAcc(h)      (h)->acc
#define pdCZHrzAccX(h)     pdCZHrzAcc(h)[zX]
#define pdCZHrzAccY(h)     pdCZHrzAcc(h)[zY]
#define pdCZHrzRefPosUW(h) (h)->_uwd
#define pdCZHrzRefPosU(h)  pdCZHrzRefPosUW(h)[pdU]
#define pdCZHrzRefPosW(h)  pdCZHrzRefPosUW(h)[pdW]
#define pdCZHrzDelta(h)    (h)->_delta
#define pdCZHrzDeltaU(h)   pdCZHrzDelta(h)[pdU]
#define pdCZHrzDeltaW(h)   pdCZHrzDelta(h)[pdW]
#define pdCZHrzVelUW(h)    (h)->_vuw
#define pdCZHrzVelU(h)     pdCZHrzVelUW(h)[pdU]
#define pdCZHrzVelW(h)     pdCZHrzVelUW(h)[pdW]

/* methods to set parameters */
#define pdCZHrzSetRefPos(h,xd,yd)   zVec2DCreate( pdCZHrzRefPos(h), xd, yd )
#define pdCZHrzSetRefPosVec(h,pd)   zVec2DCopy( pd, pdCZHrzRefPos(h) )
#define pdCZHrzSetRefPosX(h,xd)     pdCZHrzRefPosX(h) = xd
#define pdCZHrzSetRefPosY(h,yd)     pdCZHrzRefPosY(h) = yd
#define pdCZHrzSetRefTheta(h,td)    pdCZHrzRefTheta(h) = td
#define pdCZHrzSetPos(h,x,y)        zVec2DCreate( pdCZHrzPos(h), x, y )
#define pdCZHrzSetPosVec(h,p)       zVec2DCopy( p, pdCZHrzPos(h) )
#define pdCZHrzSetPosX(h,x)         pdCZHrzPosX(h) = x
#define pdCZHrzSetPosY(h,y)         pdCZHrzPosY(h) = y
#define pdCZHrzSetTheta(h,t)        pdCZHrzTheta(h) = t
#define pdCZHrzSetVel(h,vx,vy)      zVec2DCreate( pdCZHrzVel(h), vx, vy )
#define pdCZHrzSetVelVec(h,v)       zVec2DCopy( v, pdCZHrzVel(h) )
#define pdCZHrzSetVelX(h,vx)        pdCZHrzVelX(h) = vx
#define pdCZHrzSetVelY(h,vy)        pdCZHrzVelY(h) = vy
#define pdCZHrzSetRefVelU(h,vd)     pdCZHrzUWSetRefVelU( &pdCZHrzUW(h), vd )
#define pdCZHrzSetQ1U(h,q1)         pdCZHrzUWSetQ1U( &pdCZHrzUW(h), q1 )
#define pdCZHrzSetQ2U(h,q2)         pdCZHrzUWSetQ2U( &pdCZHrzUW(h), q2 )
#define pdCZHrzSetRefVelW(h,vd)     pdCZHrzUWSetRefVelW( &pdCZHrzUW(h), vd )
#define pdCZHrzSetQ1W(h,q1)         pdCZHrzUWSetQ1W( &pdCZHrzUW(h), q1 )
#define pdCZHrzSetQ2W(h,q2)         pdCZHrzUWSetQ2W( &pdCZHrzUW(h), q2 )
#define pdCZHrzSetRho(h,r)          pdCZHrzUWSetRho( &pdCZHrzUW(h), r )
#define pdCZHrzSetKr(h,k)           pdCZHrzUWSetKr( &pdCZHrzUW(h), k )
#define pdCZHrzSetDist(h,d)         pdCZHrzUWSetDist( &pdCZHrzUW(h), d )
#define pdCZHrzSetKappa(h,k)        pdCZHrzUWSetKappa( &pdCZHrzUW(h), k )
#define pdCZHrzSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) \
  pdCZHrzUWSetPrm( &pdCZHrzUW(h), vud, qu1, qu2, \
                                  vwd, qw1, qw2, rho, kr, dist, kappa )
#define pdCZHrzSetSR(h,sr)          pdCZHrzSR(h) = sr
#define pdCZHrzSetRefPosUW(h,ud,wd) zVec2DCreate( pdCZHrzRefPosUW(h), ud, wd )
#define pdCZHrzSetRefPosUWVec(h,pd) zVec2DCopy( pd, pdCZHrzRefPosUW(h) )
#define pdCZHrzSetRefPosU(h,ud)     pdCZHrzRefPosU(h) = ud
#define pdCZHrzSetRefPosW(h,wd)     pdCZHrzRefPosW(h) = wd
#define pdCZHrzSetDelta(h,du,dw)    zVec2DCreate( pdCZHrzDelta(h), du, dw )
#define pdCZHrzSetDeltaVec(h,d)     zVec2DCopy( d, pdCZHrzDelta(h) )
#define pdCZHrzSetDeltaU(h,du)      pdCZHrzDeltaU(h) = du
#define pdCZHrzSetDeltaW(h,dw)      pdCZHrzDeltaW(h) = dw
#define pdCZHrzSetVelUW(h,vu,vw)    zVec2DCreate( pdCZHrzVelUW(h), vu, vw )
#define pdCZHrzSetVelUWVec(h,v)     zVec2DCopy( v, pdCZHrzVelUW(h) )
#define pdCZHrzSetVelU(h,vu)        pdCZHrzVelU(h) = vu
#define pdCZHrzSetVelW(h,vw)        pdCZHrzVelW(h) = vw

/* calculation method */
__EXPORT double *pdCZHrzRotUWtoXY(pdCZHrz *hrz, zVec2D vuw, zVec2D vxy);
__EXPORT double *pdCZHrzRotXYtoUW(pdCZHrz *hrz, zVec2D vxy, zVec2D vuw);
__EXPORT double *pdCZHrzXformUWtoXY(pdCZHrz *hrz, zVec2D uw, zVec2D xy);
__EXPORT double *pdCZHrzXformXYtoUW(pdCZHrz *hrz, zVec2D xy, zVec2D uw);
__EXPORT void pdCZHrzXformSRXYtoUW(pdCZHrz *hrz, zVec3DList *sr);
__EXPORT void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, zVec2D uwd, zVec2D delta);

/* update method */
#define pdCZHrzUpdateUW(h,d,v) pdCZHrzUWUpdate( &pdCZHrzUW(h), d, v )
__EXPORT void pdCZHrzUpdate(pdCZHrz *hrz, zVec2D xy, double theta, zVec2D vxy, zVec2D xyd, double thetad, zVec3DList *sr);

/* for debug */
__EXPORT void pdCZHrzFWrite(FILE *fp, pdCZHrz *hrz);
#define pdCZHrzWrite(h) pdCZHrzFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_H__ */
