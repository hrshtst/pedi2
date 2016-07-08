#ifndef __PD_CZ_H__
#define __PD_CZ_H__

#include <zm/zm_ode.h>
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
  double _theta;    /* rotational angle */
  zVec3DList *_sr;  /* supporting region */

  pdCZVrt _vrt;     /* vertical motion controller */
  pdCZHrz _hrz;     /* horizontal motion controller */

  struct{
    double _alpha[3]; /* estimated ZMP error */
    double _beta[3];  /* estimated COM error */
    double _k[3];     /* relaxation coefficient for ZMP error */
    double _b[3];     /* relaxation coefficient for COM error */
  } _errcomp;         /* error compensator */

  struct{
    double _t;       /* time */
    double _dt;      /* time step */
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
__EXPORT void pdCZInit(pdCZ *cz, double dt);
__EXPORT void pdCZDestroy(pdCZ *cz);

/* methods to get parameters */
#define pdCZTime(c)     (c)->_ode._t
#define pdCZTimeStep(c) (c)->_ode._dt
#define pdCZCmdCOM(c)   ( &(c)->_comd )
#define pdCZCmdCOMX(c)  zVec3DElem( pdCZCmdCOM(c), zX )
#define pdCZCmdCOMY(c)  zVec3DElem( pdCZCmdCOM(c), zY )
#define pdCZCmdCOMZ(c)  zVec3DElem( pdCZCmdCOM(c), zZ )
#define pdCZCmdTheta(c) (c)->_thetad
#define pdCZCOM(c)      ( &(c)->_com )
#define pdCZCOMX(c)     zVec3DElem( pdCZCOM(c), zX )
#define pdCZCOMY(c)     zVec3DElem( pdCZCOM(c), zY )
#define pdCZCOMZ(c)     zVec3DElem( pdCZCOM(c), zZ )
#define pdCZVel(c)      ( &(c)->_vel )
#define pdCZVelX(c)     zVec3DElem( pdCZVel(c), zX )
#define pdCZVelY(c)     zVec3DElem( pdCZVel(c), zY )
#define pdCZVelZ(c)     zVec3DElem( pdCZVel(c), zZ )
#define pdCZAcc(c)      ( &(c)->_acc )
#define pdCZAccX(c)     zVec3DElem( pdCZAcc(c), zX )
#define pdCZAccY(c)     zVec3DElem( pdCZAcc(c), zY )
#define pdCZAccZ(c)     zVec3DElem( pdCZAcc(c), zZ )
#define pdCZZMP(c)      ( &(c)->_zmp )
#define pdCZZMPX(c)     zVec3DElem( pdCZZMP(c), zX )
#define pdCZZMPY(c)     zVec3DElem( pdCZZMP(c), zY )
#define pdCZZMPZ(c)     zVec3DElem( pdCZZMP(c), zZ )
#define pdCZFZ(c)      (c)->_fz
#define pdCZTheta(c)   (c)->_theta
#define pdCZSR(c)      (c)->_sr
#define pdCZVrtPtr(c)  ( &(c)->_vrt )
#define pdCZQ1Z(c)     pdCZVrtQ1( pdCZVrtPtr(c) )
#define pdCZQ2Z(c)     pdCZVrtQ2( pdCZVrtPtr(c) )
#define pdCZZeta(c)    pdCZVrtZeta( pdCZVrtPtr(c) )
#define pdCZHrzPtr(c)  ( &(c)->_hrz )
#define pdCZRefVelU(c) pdCZHrzRefVelU( pdCZHrzPtr(c) )
#define pdCZQ1U(c)     pdCZHrzQ1U( pdCZHrzPtr(c) )
#define pdCZQ2U(c)     pdCZHrzQ2U( pdCZHrzPtr(c) )
#define pdCZRefVelW(c) pdCZHrzRefVelW( pdCZHrzPtr(c) )
#define pdCZQ1W(c)     pdCZHrzQ1W( pdCZHrzPtr(c) )
#define pdCZQ2W(c)     pdCZHrzQ2W( pdCZHrzPtr(c) )
#define pdCZRho(c)     pdCZHrzRho( pdCZHrzPtr(c) )
#define pdCZKr(c)      pdCZHrzKr( pdCZHrzPtr(c) )
#define pdCZDist(c)    pdCZHrzDist( pdCZHrzPtr(c) )
#define pdCZKappa(c)   pdCZHrzKappa( pdCZHrzPtr(c) )
#define pdCZRefPosUW(c) pdCZHrzRefPosUW( pdCZHrzPtr(c) )
#define pdCZRefPosU(c)  pdCZHrzRefPosU( pdCZHrzPtr(c) )
#define pdCZRefPosW(c)  pdCZHrzRefPosW( pdCZHrzPtr(c) )
#define pdCZDelta(c)    pdCZHrzDelta( pdCZHrzPtr(c) )
#define pdCZDeltaU(c)   pdCZHrzDeltaU( pdCZHrzPtr(c) )
#define pdCZDeltaW(c)   pdCZHrzDeltaW( pdCZHrzPtr(c) )
#define pdCZVelUW(c)    pdCZHrzVelUW( pdCZHrzPtr(c) )
#define pdCZVelU(c)     pdCZHrzVelU( pdCZHrzPtr(c) )
#define pdCZVelW(c)     pdCZHrzVelW( pdCZHrzPtr(c) )
#define pdCZAlphaX(c)   ( (c)->_errcomp._alpha[0] )
#define pdCZAlphaY(c)   ( (c)->_errcomp._alpha[1] )
#define pdCZAlphaZ(c)   ( (c)->_errcomp._alpha[2] )
#define pdCZBetaX(c)    ( (c)->_errcomp._beta[0] )
#define pdCZBetaY(c)    ( (c)->_errcomp._beta[1] )
#define pdCZBetaZ(c)    ( (c)->_errcomp._beta[2] )
#define pdCZErrCompKX(c) ( (c)->_errcomp._k[0] )
#define pdCZErrCompKY(c) ( (c)->_errcomp._k[1] )
#define pdCZErrCompKZ(c) ( (c)->_errcomp._k[2] )
#define pdCZErrCompBX(c) ( (c)->_errcomp._b[0] )
#define pdCZErrCompBY(c) ( (c)->_errcomp._b[1] )
#define pdCZErrCompBZ(c) ( (c)->_errcomp._b[2] )
#define pdCZRefCOM(c)  ( &(c)->refcom )
#define pdCZRefCOMX(c) zVec3DElem( pdCZRefCOM(c), zX )
#define pdCZRefCOMY(c) zVec3DElem( pdCZRefCOM(c), zY )
#define pdCZRefCOMZ(c) zVec3DElem( pdCZRefCOM(c), zZ )
#define pdCZRefVel(c)  ( &(c)->refvel )
#define pdCZRefVelX(c) zVec3DElem( pdCZRefVel(c), zX )
#define pdCZRefVelY(c) zVec3DElem( pdCZRefVel(c), zY )
#define pdCZRefVelZ(c) zVec3DElem( pdCZRefVel(c), zZ )
#define pdCZRefAcc(c)  ( &(c)->refacc )
#define pdCZRefAccX(c) zVec3DElem( pdCZRefAcc(c), zX )
#define pdCZRefAccY(c) zVec3DElem( pdCZRefAcc(c), zY )
#define pdCZRefAccZ(c) zVec3DElem( pdCZRefAcc(c), zZ )
#define pdCZRefZMP(c)  ( &(c)->refzmp )
#define pdCZRefZMPX(c) zVec3DElem( pdCZRefZMP(c), zX )
#define pdCZRefZMPY(c) zVec3DElem( pdCZRefZMP(c), zY )
#define pdCZRefZMPZ(c) zVec3DElem( pdCZRefZMP(c), zZ )
#define pdCZRefFZ(c)   ( (c)->reffz )

/* methods to set parameters */
#define pdCZSetTime(c,t)          ( (c)->_ode._t = (t) )
#define pdCZResetTime(c)          pdCZSetTime( c, 0 )
#define pdCZSetTimeStep(c,dt)     ( (c)->_ode._dt = (dt) )
#define pdCZIncrTime(c)           ( pdCZTime( c ) += pdCZTimeStep( c ) )
#define pdCZSetCmdCOM(c,xd,yd,zd) zVec3DCreate( pdCZCmdCOM(c), xd, yd, zd )
#define pdCZSetCmdCOMVec(c,pd)    zVec3DCopy( pd, pdCZCmdCOM(c) )
#define pdCZSetCmdCOMX(c,xd)      zVec3DSetElem( pdCZCmdCOM(c), zX, xd )
#define pdCZSetCmdCOMY(c,yd)      zVec3DSetElem( pdCZCmdCOM(c), zY, yd )
#define pdCZSetCmdCOMZ(c,zd)      zVec3DSetElem( pdCZCmdCOM(c), zZ, zd )
#define pdCZSetCmdTheta(c,td)     ( pdCZCmdTheta(c) = (td) )
#define pdCZSetCOM(c,x,y,z)       zVec3DCreate( pdCZCOM(c), x, y, z )
#define pdCZSetCOMVec(c,p)        zVec3DCopy( p, pdCZCOM(c) )
#define pdCZSetCOMX(c,x)          zVec3DSetElem( pdCZCOM(c), zX, x )
#define pdCZSetCOMY(c,y)          zVec3DSetElem( pdCZCOM(c), zY, y )
#define pdCZSetCOMZ(c,z)          zVec3DSetElem( pdCZCOM(c), zZ, z )
#define pdCZSetVel(c,vx,vy,vz)    zVec3DCreate( pdCZVel(c), vx, vy, vz )
#define pdCZSetVelVec(c,v)        zVec3DCopy( v, pdCZVel(c) )
#define pdCZSetVelX(c,vx)         zVec3DSetElem( pdCZVel(c), zX, vx )
#define pdCZSetVelY(c,vy)         zVec3DSetElem( pdCZVel(c), zY, vy )
#define pdCZSetVelZ(c,vz)         zVec3DSetElem( pdCZVel(c), zZ, vz )
#define pdCZSetAcc(c,ax,ay,az)    zVec3DCreate( pdCZAcc(c), ax, ay, az )
#define pdCZSetAccVec(c,a)        zVec3DCopy( a, pdCZAcc(c) )
#define pdCZSetAccX(c,ax)         zVec3DSetElem( pdCZAcc(c), zX, ax )
#define pdCZSetAccY(c,ay)         zVec3DSetElem( pdCZAcc(c), zY, ay )
#define pdCZSetAccZ(c,az)         zVec3DSetElem( pdCZAcc(c), zZ, az )
#define pdCZSetZMP(c,xz,yz,zz)    zVec3DCreate( pdCZZMP(c), xz, yz, zz )
#define pdCZSetZMPVec(c,pz)       zVec3DCopy( pz, pdCZZMP(c) )
#define pdCZSetZMPX(c,xz)         zVec3DSetElem( pdCZZMP(c), zX, xz )
#define pdCZSetZMPY(c,yz)         zVec3DSetElem( pdCZZMP(c), zY, yz )
#define pdCZSetZMPZ(c,zz)         zVec3DSetElem( pdCZZMP(c), zZ, zz )
#define pdCZSetFZ(c,fz)           ( (c)->_fz = (fz) )
#define pdCZSetTheta(c,t)         ( (c)->_theta = (t) )
#define pdCZSetSR(c,sr)           ( (c)->_sr = (sr) )
#define pdCZSetQ1Z(c,q1)          pdCZVrtSetQ1( pdCZVrtPtr(c), q1 )
#define pdCZSetQ2Z(c,q2)          pdCZVrtSetQ2( pdCZVrtPtr(c), q2 )
#define pdCZSetRefVelU(c,vd)      pdCZHrzSetRefVelU( pdCZHrzPtr(c), vd )
#define pdCZSetQ1U(c,q1)          pdCZHrzSetQ1U( pdCZHrzPtr(c), q1 )
#define pdCZSetQ2U(c,q2)          pdCZHrzSetQ2U( pdCZHrzPtr(c), q2 )
#define pdCZSetRefVelW(c,vd)      pdCZHrzSetRefVelW( pdCZHrzPtr(c), vd )
#define pdCZSetQ1W(c,q1)          pdCZHrzSetQ1W( pdCZHrzPtr(c), q1 )
#define pdCZSetQ2W(c,q2)          pdCZHrzSetQ2W( pdCZHrzPtr(c), q2 )
#define pdCZSetRho(c,r)           pdCZHrzSetRho( pdCZHrzPtr(c), r )
#define pdCZSetKr(c,k)            pdCZHrzSetKr( pdCZHrzPtr(c), k )
#define pdCZSetDist(c,d)          pdCZHrzSetDist( pdCZHrzPtr(c), d )
#define pdCZSetKappa(c,k)         pdCZHrzSetKappa( pdCZHrzPtr(c), k )
#define pdCZSetRefPosUW(c,ud,wd) pdCZHrzSetRefPosUW( pdCZHrzPtr(c), ud, wd )
#define pdCZSetRefPosUWVec(c,pd) pdCZHrzSetRefPosUWVec( pdCZHrzPtr(c), pd )
#define pdCZSetRefPosU(c,ud)     pdCZHrzSetRefPosU( pdCZHrzPtr(c), ud )
#define pdCZSetRefPosW(c,wd)     pdCZHrzSetRefPosW( pdCZHrzPtr(c), wd )
#define pdCZSetDelta(c,du,dw)    pdCZHrzSetDelta( pdCZHrzPtr(c), du, dw )
#define pdCZSetDeltaVec(c,d)     pdCZHrzSetDeltaVec( pdCZHrzPtr(c), d )
#define pdCZSetDeltaU(c,du)      pdCZHrzSetDeltaU( pdCZHrzPtr(c), du )
#define pdCZSetDeltaW(c,dw)      pdCZHrzSetDeltaW( pdCZHrzPtr(c), dw )
#define pdCZSetVelUW(c,vu,vw)    pdCZHrzSetVelUW( pdCZHrzPtr(c), vu, vw )
#define pdCZSetVelUWVec(c,v)     pdCZHrzSetVelUW( pdCZHrzPtr(c), v )
#define pdCZSetVelU(c,vu)        pdCZHrzSetVelU( pdCZHrzPtr(c), vu )
#define pdCZSetVelW(c,vw)        pdCZHrzSetVelW( pdCZHrzPtr(c), vw )
#define pdCZSetErrCompKX(c,kx)   ( (c)->_errcomp._k[0] = (kx) )
#define pdCZSetErrCompKY(c,ky)   ( (c)->_errcomp._k[1] = (ky) )
#define pdCZSetErrCompKZ(c,kz)   ( (c)->_errcomp._k[2] = (kz) )
#define pdCZSetErrCompBX(c,dx)   ( (c)->_errcomp._b[0] = (dx) )
#define pdCZSetErrCompBY(c,dy)   ( (c)->_errcomp._b[1] = (dy) )
#define pdCZSetErrCompBZ(c,dz)   ( (c)->_errcomp._b[2] = (dz) )

/* calculation method */
__EXPORT double pdCZCalcDeltaTheta(pdCZ *cz, zVec2D refuw);
__EXPORT double pdCZCalcDeltaW(pdCZ *cz, zVec2D refuw, double delta_theta);
__EXPORT void pdCZCalcNextUW(pdCZ *cz, zVec2D refuw, zVec2D nextuwd);

/* update method */
__EXPORT void pdCZUpdate(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, double theta, zVec3DList *sr);
__EXPORT void pdCZAutoUpdateRef(pdCZ *cz, zVec3D *lfpos, zVec3D *rfpos, zVec3D *comd, double *thetad);

/* output method */
__EXPORT void pdCZFWrite(FILE *fp, pdCZ *cz);
#define pdCZWrite(c) pdCZFWrite( stdout, c )
__EXPORT void pdCZDataFWrite(FILE *fp, pdCZ *cz);
#define pdCZDataWrite(c) pdCZDataFWrite( stdout, c )

__END_DECLS

#endif /* __PD_CZ_H__ */
