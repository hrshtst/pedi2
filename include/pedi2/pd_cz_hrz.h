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

  double refxz;     /* referential ZMP pos. */
  double refyz;     /* referential ZMP pos. */
  double refddx;    /* referential COM acc. */
  double refddy;    /* referential COM acc. */
} pdCZHrz;

/* c'tor and d'tor */
__EXPORT void pdCZHrzInit(pdCZHrz *hrz, pdCZVrt *vrt);
__EXPORT void pdCZHrzDestroy(pdCZHrz *hrz);

/* methods to set parameters */
#define pdCZHrzSetRefPosX(h,xd)  (h)->_xd = xd
#define pdCZHrzSetRefPosY(h,yd)  (h)->_yd = yd
#define pdCZHrzSetRefTheta(h,td) (h)->_thetad = td
#define pdCZHrzSetPosX(h,x)      (h)->_x = x
#define pdCZHrzSetPosY(h,y)      (h)->_y = y
#define pdCZHrzSetVelX(h,vx)     (h)->_vx = vx
#define pdCZHrzSetVelY(h,vy)     (h)->_vy = vy
#define pdCZHrzSetTheta(h,t)     (h)->_theta = t
#define pdCZHrzSetRefVelU(h,vd)  pdCZHrzMovSetRefVelU(&(h)->_hrzm,vd)
#define pdCZHrzSetQ1U(h,q1)      pdCZHrzMovSetQ1U(&(h)->_hrzm,q1)
#define pdCZHrzSetQ2U(h,q2)      pdCZHrzMovSetQ2U(&(h)->_hrzm,q2)
#define pdCZHrzSetRefVelW(h,vd)  pdCZHrzMovSetRefVelW(&(h)->_hrzm,vd)
#define pdCZHrzSetQ1W(h,q1)      pdCZHrzMovSetQ1W(&(h)->_hrzm,q1)
#define pdCZHrzSetQ2W(h,q2)      pdCZHrzMovSetQ2W(&(h)->_hrzm,q2)
#define pdCZHrzSetRho(h,r)       pdCZHrzMovSetRho(&(h)->_hrzm,r)
#define pdCZHrzSetKr(h,k)        pdCZHrzMovSetKr(&(h)->_hrzm,k)
#define pdCZHrzSetDist(h,d)      pdCZHrzMovSetDist(&(h)->_hrzm,d)
#define pdCZHrzSetKappa(h,k)     pdCZHrzMovSetKappa(&(h)->_hrzm,k)
#define pdCZHrzSetPrm(h,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa) \
  pdCZHrzMovSetPrm(&(h)->_hrzm,vud,qu1,qu2,vwd,qw1,qw2,rho,kr,dist,kappa)
#define pdCZHrzSetSR(h,sr) (h)->_sr = sr

/* methods to get parameters */
#define pdCZHrzRefPosX(h)  (h)->_xd
#define pdCZHrzRefPosY(h)  (h)->_yd
#define pdCZHrzRefTheta(h) (h)->_thetad
#define pdCZHrzPosX(h)     (h)->_x
#define pdCZHrzPosY(h)     (h)->_y
#define pdCZHrzVelX(h)     (h)->_vx
#define pdCZHrzVelY(h)     (h)->_vy
#define pdCZHrzTheta(h)    (h)->_theta
#define pdCZHrzRefVelU(h)  pdCZHrzMovRefVelU(&(h)->_hrzm)
#define pdCZHrzQ1U(h)      pdCZHrzMovQ1U(&(h)->_hrzm)
#define pdCZHrzQ2U(h)      pdCZHrzMovQ2U(&(h)->_hrzm)
#define pdCZHrzRefVelW(h)  pdCZHrzMovRefVelW(&(h)->_hrzm)
#define pdCZHrzQ1W(h)      pdCZHrzMovQ1W(&(h)->_hrzm)
#define pdCZHrzQ2W(h)      pdCZHrzMovQ2W(&(h)->_hrzm)
#define pdCZHrzRho(h)      pdCZHrzMovRho(&(h)->_hrzm)
#define pdCZHrzKr(h)       pdCZHrzMovKr(&(h)->_hrzm)
#define pdCZHrzDist(h)     pdCZHrzMovDist(&(h)->_hrzm)
#define pdCZHrzKappa(h)    pdCZHrzMovKappa(&(h)->_hrzm)
#define pdCZHrzSRM(h)      pdCZHrzMovSR(&(h)->_hrzm)
#define pdCZHrzSRVertM(h)  pdCZHrzMovSRVert(&(h)->_hrzm)
#define pdCZHrzZMPU(h)     pdCZHrzMovZMPU(&(h)->_hrzm)
#define pdCZHrzZMPW(h)     pdCZHrzMovZMPW(&(h)->_hrzm)
#define pdCZHrzAccU(h)     pdCZHrzMovAccU(&(h)->_hrzm)
#define pdCZHrzAccW(h)     pdCZHrzMovAccW(&(h)->_hrzm)
#define pdCZHrzSRW(h)      (h)->_sr
#define pdCZHrzZMPX(h)     (h)->refxz
#define pdCZHrzZMPY(h)     (h)->refyz
#define pdCZHrzAccX(h)     (h)->refddx
#define pdCZHrzAccY(h)     (h)->refddy

/* calculation method */
__EXPORT void pdCZHrzXformMtoW(pdCZHrz *hrz, double u, double w, double *X, double *Y);
__EXPORT void pdCZHrzXformWtoM(pdCZHrz *hrz, double X, double Y, double *u, double *w);
__EXPORT void pdCZHrzRotMtoW(pdCZHrz *hrz, double vu, double vw, double *vx, double *vy);
__EXPORT void pdCZHrzRotWtoM(pdCZHrz *hrz, double vx, double vy, double *vu, double *vw);
__EXPORT void pdCZHrzXformSRWtoM(pdCZHrz *hrz, zVec3DList *sr);
__EXPORT void pdCZHrzCalcDiffToRefPos(pdCZHrz *hrz, double ud, double wd, double *deltau, double *deltaw);

/* update method */
#define pdCZHrzUpdateM(h,du,vu,dw,vw) pdCZHrzMovUpdate( &(h)->_hrzm, du, vu, dw, vw )
__EXPORT void pdCZHrzUpdate(pdCZHrz *hrz, double x, double y, double theta, double vx, double vy, double xd, double yd, double thetad, zVec3DList *sr);

__END_DECLS

#endif /* __PD_CZ_HRZ_H__ */
