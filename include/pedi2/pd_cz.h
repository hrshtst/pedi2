#ifndef __PD_CZ_H__
#define __PD_CZ_H__

#include <zm/zm_ode.h>
#include <zeo/zeo_vec3d.h>
#include <pedi2/pd_cz_vrt.h>
#include <pedi2/pd_cz_hrz.h>

__BEGIN_DECLS

typedef struct{
  zVec3D _comd;     /* commanded COM position */
  double _thetad;   /* commanded rotational angle */
  zVec3D _com;      /* COM position */
  zVec3D _vel;      /* COM velocity */
  zVec3D _acc;      /* COM acceleration */
  zVec3D _zmp;      /* ZMP position */
  double _fz;       /* vertial reaction force */
  zVec3D _ef;       /* external force */
  double _theta;    /* rotational angle */
  zLoop3D *_sr;     /* supporting region */

  pdCZVrt _vrt;     /* vertical motion controller */
  pdCZHrz _hrz;     /* horizontal motion controller */

  struct{
    double _alpha[3]; /* estimated ZMP error */
    double _beta[3];  /* estimated COM error */
    double _k[3];     /* relaxation coefficient for ZMP error */
    double _b[3];     /* relaxation coefficient for COM error */
  } _errcomp;         /* error compensator */

  struct{
    double _t;      /* time */
    double _dt;     /* time step */
    zVec pos, vel;  /* state vector: pos = [ x y z ]^T */
    zODE2 solver;   /* ODE solver */
  } _ode;

  zVec3D refcom;    /* referential COM position */
  zVec3D refvel;    /* referential COM velocity */
  zVec3D refacc;    /* referential COM acceleration */
  zVec3D refzmp;    /* referential ZMP position */
  double reffz;     /* referential vertical reaction force */
} pdCZ;

/* c'tor and d'tor */
__PEDI2_EXPORT void pdCZInit(pdCZ *cz, double dt);
__PEDI2_EXPORT void pdCZDestroy(pdCZ *cz);

/* methods to get parameters */
#define pdCZTime(cz)      (cz)->_ode._t
#define pdCZTimeStep(cz)  (cz)->_ode._dt
#define pdCZCmdCOM(cz)    ( &(cz)->_comd )
#define pdCZCmdCOMX(cz)   pdCZCmdCOM(cz)->c.x
#define pdCZCmdCOMY(cz)   pdCZCmdCOM(cz)->c.y
#define pdCZCmdCOMZ(cz)   pdCZCmdCOM(cz)->c.z
#define pdCZCmdTheta(cz)  (cz)->_thetad
#define pdCZCOM(cz)       ( &(cz)->_com )
#define pdCZCOMX(cz)      pdCZCOM(cz)->c.x
#define pdCZCOMY(cz)      pdCZCOM(cz)->c.y
#define pdCZCOMZ(cz)      pdCZCOM(cz)->c.z
#define pdCZVel(cz)       ( &(cz)->_vel )
#define pdCZVelX(cz)      pdCZVel(cz)->c.x
#define pdCZVelY(cz)      pdCZVel(cz)->c.y
#define pdCZVelZ(cz)      pdCZVel(cz)->c.z
#define pdCZAcc(cz)       ( &(cz)->_acc )
#define pdCZAccX(cz)      pdCZAcc(cz)->c.x
#define pdCZAccY(cz)      pdCZAcc(cz)->c.y
#define pdCZAccZ(cz)      pdCZAcc(cz)->c.z
#define pdCZZMP(cz)       ( &(cz)->_zmp )
#define pdCZZMPX(cz)      pdCZZMP(cz)->c.x
#define pdCZZMPY(cz)      pdCZZMP(cz)->c.y
#define pdCZZMPZ(cz)      pdCZZMP(cz)->c.z
#define pdCZFZ(cz)        (cz)->_fz
#define pdCZExtF(cz)      ( &(cz)->_ef )
#define pdCZExtFX(cz)     pdCZExtF(cz)->c.x
#define pdCZExtFY(cz)     pdCZExtF(cz)->c.y
#define pdCZExtFZ(cz)     pdCZExtF(cz)->c.z
#define pdCZTheta(cz)     (cz)->_theta
#define pdCZSR(cz)        (cz)->_sr
#define pdCZVrtPtr(cz)    ( &(cz)->_vrt )
#define pdCZQ1Z(cz)       pdCZVrtQ1( pdCZVrtPtr(cz) )
#define pdCZQ2Z(cz)       pdCZVrtQ2( pdCZVrtPtr(cz) )
#define pdCZZeta(cz)      pdCZVrtZeta( pdCZVrtPtr(cz) )
#define pdCZHrzPtr(cz)    ( &(cz)->_hrz )
#define pdCZRefVelU(cz)   pdCZHrzRefVelU( pdCZHrzPtr(cz) )
#define pdCZQ1U(cz)       pdCZHrzQ1U( pdCZHrzPtr(cz) )
#define pdCZQ2U(cz)       pdCZHrzQ2U( pdCZHrzPtr(cz) )
#define pdCZRefVelW(cz)   pdCZHrzRefVelW( pdCZHrzPtr(cz) )
#define pdCZQ1W(cz)       pdCZHrzQ1W( pdCZHrzPtr(cz) )
#define pdCZQ2W(cz)       pdCZHrzQ2W( pdCZHrzPtr(cz) )
#define pdCZRho(cz)       pdCZHrzRho( pdCZHrzPtr(cz) )
#define pdCZKr(cz)        pdCZHrzKr( pdCZHrzPtr(cz) )
#define pdCZDist(cz)      pdCZHrzDist( pdCZHrzPtr(cz) )
#define pdCZKappa(cz)     pdCZHrzKappa( pdCZHrzPtr(cz) )
#define pdCZRefPosUW(cz)  pdCZHrzRefPosUW( pdCZHrzPtr(cz) )
#define pdCZRefPosU(cz)   pdCZHrzRefPosU( pdCZHrzPtr(cz) )
#define pdCZRefPosW(cz)   pdCZHrzRefPosW( pdCZHrzPtr(cz) )
#define pdCZDelta(cz)     pdCZHrzDelta( pdCZHrzPtr(cz) )
#define pdCZDeltaU(cz)    pdCZHrzDeltaU( pdCZHrzPtr(cz) )
#define pdCZDeltaW(cz)    pdCZHrzDeltaW( pdCZHrzPtr(cz) )
#define pdCZVelUW(cz)     pdCZHrzVelUW( pdCZHrzPtr(cz) )
#define pdCZVelU(cz)      pdCZHrzVelU( pdCZHrzPtr(cz) )
#define pdCZVelW(cz)      pdCZHrzVelW( pdCZHrzPtr(cz) )
#define pdCZAlphaX(cz)    ( (cz)->_errcomp._alpha[0] )
#define pdCZAlphaY(cz)    ( (cz)->_errcomp._alpha[1] )
#define pdCZAlphaZ(cz)    ( (cz)->_errcomp._alpha[2] )
#define pdCZBetaX(cz)     ( (cz)->_errcomp._beta[0] )
#define pdCZBetaY(cz)     ( (cz)->_errcomp._beta[1] )
#define pdCZBetaZ(cz)     ( (cz)->_errcomp._beta[2] )
#define pdCZErrCompKX(cz) ( (cz)->_errcomp._k[0] )
#define pdCZErrCompKY(cz) ( (cz)->_errcomp._k[1] )
#define pdCZErrCompKZ(cz) ( (cz)->_errcomp._k[2] )
#define pdCZErrCompBX(cz) ( (cz)->_errcomp._b[0] )
#define pdCZErrCompBY(cz) ( (cz)->_errcomp._b[1] )
#define pdCZErrCompBZ(cz) ( (cz)->_errcomp._b[2] )
#define pdCZRefCOM(cz)    ( &(cz)->refcom )
#define pdCZRefCOMX(cz)   pdCZRefCOM(cz)->c.x
#define pdCZRefCOMY(cz)   pdCZRefCOM(cz)->c.y
#define pdCZRefCOMZ(cz)   pdCZRefCOM(cz)->c.z
#define pdCZRefVel(cz)    ( &(cz)->refvel )
#define pdCZRefVelX(cz)   pdCZRefVel(cz)->c.x
#define pdCZRefVelY(cz)   pdCZRefVel(cz)->c.y
#define pdCZRefVelZ(cz)   pdCZRefVel(cz)->c.z
#define pdCZRefAcc(cz)    ( &(cz)->refacc )
#define pdCZRefAccX(cz)   pdCZRefAcc(cz)->c.x
#define pdCZRefAccY(cz)   pdCZRefAcc(cz)->c.y
#define pdCZRefAccZ(cz)   pdCZRefAcc(cz)->c.z
#define pdCZRefZMP(cz)    ( &(cz)->refzmp )
#define pdCZRefZMPX(cz)   pdCZRefZMP(cz)->c.x
#define pdCZRefZMPY(cz)   pdCZRefZMP(cz)->c.y
#define pdCZRefZMPZ(cz)   pdCZRefZMP(cz)->c.z
#define pdCZRefFZ(cz)     ( (cz)->reffz )

/* methods to set parameters */
#define pdCZSetTime(cz,t)          ( (cz)->_ode._t = (t) )
#define pdCZResetTime(cz)          pdCZSetTime( cz, 0 )
#define pdCZSetTimeStep(cz,dt)     ( (cz)->_ode._dt = (dt) )
#define pdCZIncrTime(cz)           ( pdCZTime( cz ) += pdCZTimeStep( cz ) )
#define pdCZSetCmdCOM(cz,xd,yd,zd) zVec3DCreate( pdCZCmdCOM(cz), xd, yd, zd )
#define pdCZSetCmdCOMVec(cz,pd)    zVec3DCopy( pd, pdCZCmdCOM(cz) )
#define pdCZSetCmdCOMX(cz,xd)      ( pdCZCmdCOMX(cz) = (xd) )
#define pdCZSetCmdCOMY(cz,yd)      ( pdCZCmdCOMY(cz) = (yd) )
#define pdCZSetCmdCOMZ(cz,zd)      ( pdCZCmdCOMZ(cz) = (zd) )
#define pdCZSetCmdTheta(cz,td)     ( pdCZCmdTheta(cz) = (td) )
#define pdCZSetCOM(cz,x,y,z)       zVec3DCreate( pdCZCOM(cz), x, y, z )
#define pdCZSetCOMVec(cz,p)        zVec3DCopy( p, pdCZCOM(cz) )
#define pdCZSetCOMX(cz,x)          ( pdCZCOMX(cz) = (x) )
#define pdCZSetCOMY(cz,y)          ( pdCZCOMY(cz) = (y) )
#define pdCZSetCOMZ(cz,z)          ( pdCZCOMZ(cz) = (z) )
#define pdCZSetVel(cz,vx,vy,vz)    zVec3DCreate( pdCZVel(cz), vx, vy, vz )
#define pdCZSetVelVec(cz,v)        zVec3DCopy( v, pdCZVel(cz) )
#define pdCZSetVelX(cz,vx)         ( pdCZVelX(cz) = (vx) )
#define pdCZSetVelY(cz,vy)         ( pdCZVelY(cz) = (vy) )
#define pdCZSetVelZ(cz,vz)         ( pdCZVelZ(cz) = (vz) )
#define pdCZSetAcc(cz,ax,ay,az)    zVec3DCreate( pdCZAcc(cz), ax, ay, az )
#define pdCZSetAccVec(cz,a)        zVec3DCopy( a, pdCZAcc(cz) )
#define pdCZSetAccX(cz,ax)         ( pdCZAccX(cz) = (ax) )
#define pdCZSetAccY(cz,ay)         ( pdCZAccY(cz) = (ay) )
#define pdCZSetAccZ(cz,az)         ( pdCZAccZ(cz) = (az) )
#define pdCZSetZMP(cz,xz,yz,zz)    zVec3DCreate( pdCZZMP(cz), xz, yz, zz )
#define pdCZSetZMPVec(cz,pz)       zVec3DCopy( pz, pdCZZMP(cz) )
#define pdCZSetZMPX(cz,xz)         ( pdCZZMPX(cz) = (xz) )
#define pdCZSetZMPY(cz,yz)         ( pdCZZMPY(cz) = (yz) )
#define pdCZSetZMPZ(cz,zz)         ( pdCZZMPZ(cz) = (zz) )
#define pdCZSetFZ(cz,fz)           ( (cz)->_fz = (fz) )
#define pdCZSetExtF(cz,x,y,z)      zVec3DCreate( pdCZExtF(cz), x, y, z )
#define pdCZSetExtFVec(cz,ef)      zVec3DCopy( ef, pdCZExtF(cz) )
#define pdCZSetExtFX(cz,x)         ( pdCZExtFX(cz) = (x) )
#define pdCZSetExtFY(cz,y)         ( pdCZExtFY(cz) = (y) )
#define pdCZSetExtFZ(cz,z)         ( pdCZExtFZ(cz) = (z) )
#define pdCZSetTheta(cz,t)         ( (cz)->_theta = (t) )
#define pdCZSetSR(cz,sr)           ( (cz)->_sr = (sr) )
#define pdCZSetQ1Z(cz,q1)          pdCZVrtSetQ1( pdCZVrtPtr(cz), q1 )
#define pdCZSetQ2Z(cz,q2)          pdCZVrtSetQ2( pdCZVrtPtr(cz), q2 )
#define pdCZSetRefVelU(cz,vd)      pdCZHrzSetRefVelU( pdCZHrzPtr(cz), vd )
#define pdCZSetQ1U(cz,q1)          pdCZHrzSetQ1U( pdCZHrzPtr(cz), q1 )
#define pdCZSetQ2U(cz,q2)          pdCZHrzSetQ2U( pdCZHrzPtr(cz), q2 )
#define pdCZSetRefVelW(cz,vd)      pdCZHrzSetRefVelW( pdCZHrzPtr(cz), vd )
#define pdCZSetQ1W(cz,q1)          pdCZHrzSetQ1W( pdCZHrzPtr(cz), q1 )
#define pdCZSetQ2W(cz,q2)          pdCZHrzSetQ2W( pdCZHrzPtr(cz), q2 )
#define pdCZSetRho(cz,r)           pdCZHrzSetRho( pdCZHrzPtr(cz), r )
#define pdCZSetKr(cz,k)            pdCZHrzSetKr( pdCZHrzPtr(cz), k )
#define pdCZSetDist(cz,d)          pdCZHrzSetDist( pdCZHrzPtr(cz), d )
#define pdCZSetKappa(cz,k)         pdCZHrzSetKappa( pdCZHrzPtr(cz), k )
#define pdCZSetRefPosUW(cz,ud,wd) pdCZHrzSetRefPosUW( pdCZHrzPtr(cz), ud, wd )
#define pdCZSetRefPosUWVec(cz,pd) pdCZHrzSetRefPosUWVec( pdCZHrzPtr(cz), pd )
#define pdCZSetRefPosU(cz,ud)     pdCZHrzSetRefPosU( pdCZHrzPtr(cz), ud )
#define pdCZSetRefPosW(cz,wd)     pdCZHrzSetRefPosW( pdCZHrzPtr(cz), wd )
#define pdCZSetDelta(cz,du,dw)    pdCZHrzSetDelta( pdCZHrzPtr(cz), du, dw )
#define pdCZSetDeltaVec(cz,d)     pdCZHrzSetDeltaVec( pdCZHrzPtr(cz), d )
#define pdCZSetDeltaU(cz,du)      pdCZHrzSetDeltaU( pdCZHrzPtr(cz), du )
#define pdCZSetDeltaW(cz,dw)      pdCZHrzSetDeltaW( pdCZHrzPtr(cz), dw )
#define pdCZSetVelUW(cz,vu,vw)    pdCZHrzSetVelUW( pdCZHrzPtr(cz), vu, vw )
#define pdCZSetVelUWVec(cz,v)     pdCZHrzSetVelUW( pdCZHrzPtr(cz), v )
#define pdCZSetVelU(cz,vu)        pdCZHrzSetVelU( pdCZHrzPtr(cz), vu )
#define pdCZSetVelW(cz,vw)        pdCZHrzSetVelW( pdCZHrzPtr(cz), vw )
#define pdCZSetErrCompKX(cz,kx)   ( (cz)->_errcomp._k[0] = (kx) )
#define pdCZSetErrCompKY(cz,ky)   ( (cz)->_errcomp._k[1] = (ky) )
#define pdCZSetErrCompKZ(cz,kz)   ( (cz)->_errcomp._k[2] = (kz) )
#define pdCZSetErrCompBX(cz,dx)   ( (cz)->_errcomp._b[0] = (dx) )
#define pdCZSetErrCompBY(cz,dy)   ( (cz)->_errcomp._b[1] = (dy) )
#define pdCZSetErrCompBZ(cz,dz)   ( (cz)->_errcomp._b[2] = (dz) )

/* calculation method */
__PEDI2_EXPORT double pdCZCalcDeltaTheta(pdCZ *cz, zVec2D *refuw);
__PEDI2_EXPORT double pdCZCalcDeltaW(pdCZ *cz, zVec2D *refuw, double delta_theta);
__PEDI2_EXPORT void pdCZCalcNextUW(pdCZ *cz, zVec2D *refuw, zVec2D *nextuwd);

/* update method */
__PEDI2_EXPORT void pdCZUpdate(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, zVec3D *ef, double theta, zLoop3D *sr);
__PEDI2_EXPORT void pdCZAutoUpdateRef_old(pdCZ *cz, zVec3D *comd, double *thetad);
__PEDI2_EXPORT void pdCZAutoUpdateRef(pdCZ *cz, zVec3D *lfpos, zVec3D *rfpos, zVec3D *comd, double *thetad);

/* output method */
__PEDI2_EXPORT void pdCZFWrite(FILE *fp, pdCZ *cz);
#define pdCZWrite(cz) pdCZFWrite( stdout, cz )
__PEDI2_EXPORT void pdCZDataFWrite(FILE *fp, pdCZ *cz);
#define pdCZDataWrite(cz) pdCZDataFWrite( stdout, cz )

__END_DECLS

#endif /* __PD_CZ_H__ */
