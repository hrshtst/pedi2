#ifndef __PD_CZ_HRZ_H__
#define __PD_CZ_HRZ_H__

#include <pedi2/pd_cz_hrz_mov.h>

__BEGIN_DECLS

typedef struct{
  zVec2D _pd;      /* desired COM position */
  double _thetad;  /* desired rotational angle */
  zVec2D _p, _v;   /* COM position and velocity */
  double _theta;   /* rotational angle */
  zVec3DList *_sr; /* supporting region */

  pdCZHrzMov _hrz_m; /* horizontal controller w.r.t moving frame */
  zVec2D _pd_m, _delta_m, _v_m;

  zVec2D zmp;       /* referential ZMP position */
  zVec2D acc;       /* referential COM acceleration */
} pdCZHrz;

/* c'tor and d'tor */
__EXPORT void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt);
__EXPORT void pdCZHrzDestroy(pdCZHrz *hrz);

/* methods to get parameters */
#define pdCZHrzRefPos(h)   (h)->_pd
#define pdCZHrzRefPosX(h)  pdCZHrzRefPos(h)[zX]
#define pdCZHrzRefPosY(h)  pdCZHrzRefPos(h)[zY]
#define pdCZHrzRefTheta(h) (h)->_thetad
#define pdCZHrzPos(h)      (h)->_p
#define pdCZHrzPosX(h)     pdCZHrzPos(h)[zX]
#define pdCZHrzPosY(h)     pdCZHrzPos(h)[zY]
#define pdCZHrzVel(h)      (h)->_v
#define pdCZHrzVelX(h)     pdCZHrzVel(h)[zX]
#define pdCZHrzVelY(h)     pdCZHrzVel(h)[zY]
#define pdCZHrzTheta(h)    (h)->_theta
#define pdCZHrzM(h)        (h)->_hrz_m
#define pdCZHrzRefVelU(h)  pdCZHrzMovRefVelU( &pdCZHrzM(h) )
#define pdCZHrzQ1U(h)      pdCZHrzMovQ1U( &pdCZHrzM(h) )
#define pdCZHrzQ2U(h)      pdCZHrzMovQ2U( &pdCZHrzM(h) )
#define pdCZHrzRefVelW(h)  pdCZHrzMovRefVelW( &pdCZHrzM(h) )
#define pdCZHrzQ1W(h)      pdCZHrzMovQ1W( &pdCZHrzM(h) )
#define pdCZHrzQ2W(h)      pdCZHrzMovQ2W( &pdCZHrzM(h) )
#define pdCZHrzRho(h)      pdCZHrzMovRho( &pdCZHrzM(h) )
#define pdCZHrzKr(h)       pdCZHrzMovKr( &pdCZHrzM(h) )
#define pdCZHrzDist(h)     pdCZHrzMovDist( &pdCZHrzM(h) )
#define pdCZHrzKappa(h)    pdCZHrzMovKappa( &pdCZHrzM(h) )
#define pdCZHrzSRM(h)      pdCZHrzMovSR( &pdCZHrzM(h) )
#define pdCZHrzSRVertM(h)  pdCZHrzMovSRVert( &pdCZHrzM(h) )
#define pdCZHrzZMPM(h)     pdCZHrzMovZMP( &pdCZHrzM(h) )
#define pdCZHrzZMPU(h)     pdCZHrzMovZMPU( &pdCZHrzM(h) )
#define pdCZHrzZMPW(h)     pdCZHrzMovZMPW( &pdCZHrzM(h) )
#define pdCZHrzAccM(h)     pdCZHrzMovAcc( &pdCZHrzM(h) )
#define pdCZHrzAccU(h)     pdCZHrzMovAccU( &pdCZHrzM(h) )
#define pdCZHrzAccW(h)     pdCZHrzMovAccW( &pdCZHrzM(h) )
#define pdCZHrzSR(h)       (h)->_sr
#define pdCZHrzZMP(h)      (h)->zmp
#define pdCZHrzZMPX(h)     pdCZHrzZMP(h)[zX]
#define pdCZHrzZMPY(h)     pdCZHrzZMP(h)[zY]
#define pdCZHrzAcc(h)      (h)->acc
#define pdCZHrzAccX(h)     pdCZHrzAcc(h)[zX]
#define pdCZHrzAccY(h)     pdCZHrzAcc(h)[zY]
#define pdCZHrzRefPosM(h)  (h)->_pd_m
#define pdCZHrzRefPosU(h)  pdCZHrzRefPosM(h)[pdU]
#define pdCZHrzRefPosW(h)  pdCZHrzRefPosM(h)[pdW]
#define pdCZHrzDeltaM(h)   (h)->_delta_m
#define pdCZHrzDeltaU(h)   pdCZHrzDeltaM(h)[pdU]
#define pdCZHrzDeltaW(h)   pdCZHrzDeltaM(h)[pdW]
#define pdCZHrzVelM(h)     (h)->_v_m
#define pdCZHrzVelU(h)     pdCZHrzVelM(h)[pdU]
#define pdCZHrzVelW(h)     pdCZHrzVelM(h)[pdW]

/* methods to set parameters */
#define pdCZHrzSetRefPos(h,xd,yd) zVec2DCreate( pdCZHrzRefPos(h), xd, yd )
#define pdCZHrzSetRefPosVec(h,pd) zVec2DCopy( pd, pdCZHrzRefPos(h) )
#define pdCZHrzSetRefPosX(h,xd)   pdCZHrzRefPosX(h) = xd
#define pdCZHrzSetRefPosY(h,yd)   pdCZHrzRefPosY(h) = yd
#define pdCZHrzSetRefTheta(h,td)  pdCZHrzRefTheta(h) = td
#define pdCZHrzSetPos(h,x,y)      zVec2DCreate( pdCZHrzPos(h), x, y )
#define pdCZHrzSetPosVec(h,p)     zVec2DCopy( p, pdCZHrzPos(h) )
#define pdCZHrzSetPosX(h,x)       pdCZHrzPosX(h) = x
#define pdCZHrzSetPosY(h,y)       pdCZHrzPosY(h) = y
#define pdCZHrzSetTheta(h,t)      pdCZHrzTheta(h) = t
#define pdCZHrzSetVel(h,vx,vy)    zVec2DCreate( pdCZHrzVel(h), vx, vy )
#define pdCZHrzSetVelVec(h,v)     zVec2DCopy( v, pdCZHrzVel(h) )
#define pdCZHrzSetVelX(h,vx)      pdCZHrzVelX(h) = vx
#define pdCZHrzSetVelY(h,vy)      pdCZHrzVelY(h) = vy
#define pdCZHrzSetRefVelU(h,vd)   pdCZHrzMovSetRefVelU( &pdCZHrzM(h), vd )
#define pdCZHrzSetQ1U(h,q1)       pdCZHrzMovSetQ1U( &pdCZHrzM(h), q1 )
#define pdCZHrzSetQ2U(h,q2)       pdCZHrzMovSetQ2U( &pdCZHrzM(h), q2 )
#define pdCZHrzSetRefVelW(h,vd)   pdCZHrzMovSetRefVelW( &pdCZHrzM(h), vd )
#define pdCZHrzSetQ1W(h,q1)       pdCZHrzMovSetQ1W( &pdCZHrzM(h), q1 )
#define pdCZHrzSetQ2W(h,q2)       pdCZHrzMovSetQ2W( &pdCZHrzM(h), q2 )
#define pdCZHrzSetRho(h,r)        pdCZHrzMovSetRho( &pdCZHrzM(h), r )
#define pdCZHrzSetKr(h,k)         pdCZHrzMovSetKr( &pdCZHrzM(h), k )
#define pdCZHrzSetDist(h,d)       pdCZHrzMovSetDist( &pdCZHrzM(h), d )
#define pdCZHrzSetKappa(h,k)      pdCZHrzMovSetKappa( &pdCZHrzM(h), k )
#define pdCZHrzSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) \
  pdCZHrzMovSetPrm( &pdCZHrzM(h), vud, qu1, qu2, \
                                  vwd, qw1, qw2, rho, kr, dist, kappa )
#define pdCZHrzSetSR(h,sr)          pdCZHrzSR(h) = sr
#define pdCZHrzSetRefPosM(h,ud,wd) zVec2DCreate( pdCZHrzRefPosM(h), ud, wd )
#define pdCZHrzSetRefPosMVec(h,pd) zVec2DCopy( pd, pdCZHrzRefPosM(h) )
#define pdCZHrzSetRefPosU(h,ud)    pdCZHrzRefPosU(h) = ud
#define pdCZHrzSetRefPosW(h,wd)    pdCZHrzRefPosW(h) = wd
#define pdCZHrzSetDeltaM(h,du,dw)  zVec2DCreate( pdCZHrzDeltaM(h), du, dw )
#define pdCZHrzSetDeltaMVec(h,d)   zVec2DCopy( d, pdCZHrzDeltaM(h) )
#define pdCZHrzSetDeltaU(h,du)     pdCZHrzDeltaU(h) = du
#define pdCZHrzSetDeltaW(h,dw)     pdCZHrzDeltaW(h) = dw
#define pdCZHrzSetVelM(h,vu,vw)    zVec2DCreate( pdCZHrzVelM(h), vu, vw )
#define pdCZHrzSetVelMVec(h,v)     zVec2DCopy( v, pdCZHrzVelM(h) )
#define pdCZHrzSetVelU(h,vu)       pdCZHrzVelU(h) = vu
#define pdCZHrzSetVelW(h,vw)       pdCZHrzVelW(h) = vw

/* calculation method */
__EXPORT double *pdCZHrzRotMtoW(pdCZHrz *hrz, zVec2D vm, zVec2D vw);
__EXPORT double *pdCZHrzRotWtoM(pdCZHrz *hrz, zVec2D vw, zVec2D vm);
__EXPORT double *pdCZHrzXformMtoW(pdCZHrz *hrz, zVec2D pm, zVec2D pw);
__EXPORT double *pdCZHrzXformWtoM(pdCZHrz *hrz, zVec2D pw, zVec2D pm);
__EXPORT void pdCZHrzXformSRWtoM(pdCZHrz *hrz, zVec3DList *sr);
__EXPORT void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, zVec2D pdm, zVec2D deltam);

/* update method */
#define pdCZHrzUpdateM(h,d,v) pdCZHrzMovUpdate( &pdCZHrzM(h), d, v )
__EXPORT void pdCZHrzUpdate(pdCZHrz *hrz, zVec2D p, double theta, zVec2D v, zVec2D pd, double thetad, zVec3DList *sr);

/* for debug */
__EXPORT void pdCZHrzFWrite(FILE *fp, pdCZHrz *hrz);
#define pdCZHrzWrite(h) pdCZHrzFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_H__ */
