#ifndef __PD_CZ_HRZ_H__
#define __PD_CZ_HRZ_H__

#include <pedi2/pd_cz_hrz_mov.h>

__BEGIN_DECLS

typedef struct{
  double _xd, _yd; /* desired COM position */
  double _thetad;  /* desired rotational angle */
  double _x, _y;   /* COM position */
  double _vx, _vy; /* COM velocity */
  double _theta;   /* rotational angle */
  zVec3DList *_sr; /* supporting region */

  pdCZHrzMov _hrzm; /* horizontal controller w.r.t moving frame */
  double _ud, _deltau, _vu;
  double _wd, _deltaw, _vw;

  double refxz;     /* referential ZMP pos. */
  double refyz;     /* referential ZMP pos. */
  double refddx;    /* referential COM acc. */
  double refddy;    /* referential COM acc. */
} pdCZHrz;

/* c'tor and d'tor */
__EXPORT void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt);
__EXPORT void pdCZHrzDestroy(pdCZHrz *hrz);

/* methods to get parameters */
#define pdCZHrzRefPosX(h)  (h)->_xd
#define pdCZHrzRefPosY(h)  (h)->_yd
#define pdCZHrzRefTheta(h) (h)->_thetad
#define pdCZHrzPosX(h)     (h)->_x
#define pdCZHrzPosY(h)     (h)->_y
#define pdCZHrzVelX(h)     (h)->_vx
#define pdCZHrzVelY(h)     (h)->_vy
#define pdCZHrzTheta(h)    (h)->_theta
#define pdCZHrzM(h)        (h)->_hrzm
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
#define pdCZHrzZMPU(h)     pdCZHrzMovZMPU( &pdCZHrzM(h) )
#define pdCZHrzZMPW(h)     pdCZHrzMovZMPW( &pdCZHrzM(h) )
#define pdCZHrzAccU(h)     pdCZHrzMovAccU( &pdCZHrzM(h) )
#define pdCZHrzAccW(h)     pdCZHrzMovAccW( &pdCZHrzM(h) )
#define pdCZHrzSR(h)       (h)->_sr
#define pdCZHrzZMPX(h)     (h)->refxz
#define pdCZHrzZMPY(h)     (h)->refyz
#define pdCZHrzAccX(h)     (h)->refddx
#define pdCZHrzAccY(h)     (h)->refddy
#define pdCZHrzRefU(h)     (h)->_ud
#define pdCZHrzDeltaU(h)   (h)->_deltau
#define pdCZHrzVelU(h)     (h)->_vu
#define pdCZHrzRefW(h)     (h)->_wd
#define pdCZHrzDeltaW(h)   (h)->_deltaw
#define pdCZHrzVelW(h)     (h)->_vw

/* methods to set parameters */
#define pdCZHrzSetRefPosX(h,xd)  pdCZHrzRefPosX(h) = xd
#define pdCZHrzSetRefPosY(h,yd)  pdCZHrzRefPosY(h) = yd
#define pdCZHrzSetRefTheta(h,td) pdCZHrzRefTheta(h) = td
#define pdCZHrzSetPosX(h,x)      pdCZHrzPosX(h) = x
#define pdCZHrzSetPosY(h,y)      pdCZHrzPosY(h) = y
#define pdCZHrzSetVelX(h,vx)     pdCZHrzVelX(h) = vx
#define pdCZHrzSetVelY(h,vy)     pdCZHrzVelY(h) = vy
#define pdCZHrzSetTheta(h,t)     pdCZHrzTheta(h) = t
#define pdCZHrzSetRefVelU(h,vd)  pdCZHrzMovSetRefVelU( &pdCZHrzM(h), vd )
#define pdCZHrzSetQ1U(h,q1)      pdCZHrzMovSetQ1U( &pdCZHrzM(h), q1 )
#define pdCZHrzSetQ2U(h,q2)      pdCZHrzMovSetQ2U( &pdCZHrzM(h), q2 )
#define pdCZHrzSetRefVelW(h,vd)  pdCZHrzMovSetRefVelW( &pdCZHrzM(h), vd )
#define pdCZHrzSetQ1W(h,q1)      pdCZHrzMovSetQ1W( &pdCZHrzM(h), q1 )
#define pdCZHrzSetQ2W(h,q2)      pdCZHrzMovSetQ2W( &pdCZHrzM(h), q2 )
#define pdCZHrzSetRho(h,r)       pdCZHrzMovSetRho( &pdCZHrzM(h), r )
#define pdCZHrzSetKr(h,k)        pdCZHrzMovSetKr( &pdCZHrzM(h), k )
#define pdCZHrzSetDist(h,d)      pdCZHrzMovSetDist( &pdCZHrzM(h), d )
#define pdCZHrzSetKappa(h,k)     pdCZHrzMovSetKappa( &pdCZHrzM(h), k )
#define pdCZHrzSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) \
  pdCZHrzMovSetPrm( &pdCZHrzM(h), vud, qu1, qu2, \
                                  vwd, qw1, qw2, rho, kr, dist, kappa )
#define pdCZHrzSetSR(h,sr)     pdCZHrzSR(h) = sr
#define pdCZHrzSetRefU(h,ud)   pdCZHrzRefU(h) = ud
#define pdCZHrzSetDeltaU(h,du) pdCZHrzDeltaU(h) = du
#define pdCZHrzSetVelU(h,vu)   pdCZHrzVelU(h) = vu
#define pdCZHrzSetRefW(h,wd)   pdCZHrzRefW(h) = wd
#define pdCZHrzSetDeltaW(h,dw) pdCZHrzDeltaW(h) = dw
#define pdCZHrzSetVelW(h,vw)   pdCZHrzVelW(h) = vw

/* calculation method */
__EXPORT void pdCZHrzXformMtoW(pdCZHrz *hrz, double u, double w, double *X, double *Y);
__EXPORT void pdCZHrzXformWtoM(pdCZHrz *hrz, double X, double Y, double *u, double *w);
__EXPORT void pdCZHrzRotMtoW(pdCZHrz *hrz, double vu, double vw, double *vx, double *vy);
__EXPORT void pdCZHrzRotWtoM(pdCZHrz *hrz, double vx, double vy, double *vu, double *vw);
__EXPORT void pdCZHrzXformSRWtoM(pdCZHrz *hrz, zVec3DList *sr);
__EXPORT void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, double ud, double wd, double *deltau, double *deltaw);

/* update method */
#define pdCZHrzUpdateM(h,du,vu,dw,vw) pdCZHrzMovUpdate( &pdCZHrzM(h), du, vu, dw, vw )
__EXPORT void pdCZHrzUpdate(pdCZHrz *hrz, double x, double y, double theta, double vx, double vy, double xd, double yd, double thetad, zVec3DList *sr);

/* for debug */
__EXPORT void pdCZHrzFWrite(FILE *fp, pdCZHrz *hrz);
#define pdCZHrzWrite(h) pdCZHrzFWrite( stdout, h )

__END_DECLS

#endif /* __PD_CZ_HRZ_H__ */
