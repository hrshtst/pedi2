#ifndef __PD_FOOT_H__
#define __PD_FOOT_H__

#include <pedi2/pd_cz_hrz.h>
#include <pedi2/pd_foot_uw.h>
#include <pedi2/pd_foot_z.h>

__BEGIN_DECLS

enum{
  PD_FOOT_INVALID=-1,
  PD_FOOT_LEFT=0,
  PD_FOOT_RIGHT
};

typedef struct{
  zVec3D _p;          /* current foot position */
  zVec3D _pd;         /* desired foot position */
  zVec3D _a;          /* current foot attitude (ZYX Euler angle) */
  zVec3D _ad;         /* desired foot attitude (ZYX Euler angle) */
  zVec3DList *_sr;    /* supporting region */

  pdFootUW _uw;       /* horizontal motion  */
  pdFootZ  _z;        /* lifting motion */
  pdCZHrz *_cz;       /* COM-ZMP motion controller */

  struct{
    double _t;        /* time */
    double _dt;       /* time step */
    double _k[3];
    double _c[3];
    double _old[3];
  } _sol;             /* for second-order lag system */

  zVec3D refp;        /* referential foot position */
  zVec3D refa;        /* referential foot attitude (ZYX Euler angle) */
} pdFoot;

/* c'tor and d'tor */
__EXPORT void pdFootInit(pdFoot *f, pdCZHrz *cz, byte dir, double dt);
__EXPORT void pdFootDestroy(pdFoot *f);

/* methods to get parameters */
#define pdFootTime(f)      (f)->_sol._t
#define pdFootTimeStep(f)  (f)->_sol._dt
#define pdFootPos(f)       ( &(f)->_p )
#define pdFootPosX(f)      zVec3DElem( pdFootPos(f), zX )
#define pdFootPosY(f)      zVec3DElem( pdFootPos(f), zY )
#define pdFootPosZ(f)      zVec3DElem( pdFootPos(f), zZ )
#define pdFootDesPos(f)    ( &(f)->_pd )
#define pdFootDesPosX(f)   zVec3DElem( pdFootDesPos(f), zX )
#define pdFootDesPosY(f)   zVec3DElem( pdFootDesPos(f), zY )
#define pdFootDesPosZ(f)   zVec3DElem( pdFootDesPos(f), zZ )
#define pdFootAtt(f)       ( &(f)->_a )
#define pdFootAttX(f)      zVec3DElem( pdFootAtt(f), zX )
#define pdFootAttY(f)      zVec3DElem( pdFootAtt(f), zY )
#define pdFootAttZ(f)      zVec3DElem( pdFootAtt(f), zZ )
#define pdFootDesAtt(f)    ( &(f)->_ad )
#define pdFootDesAttX(f)   zVec3DElem( pdFootDesAtt(f), zX )
#define pdFootDesAttY(f)   zVec3DElem( pdFootDesAtt(f), zY )
#define pdFootDesAttZ(f)   zVec3DElem( pdFootDesAtt(f), zZ )
#define pdFootSR(f)        (f)->_sr
#define pdFootUWPtr(f)     ( &(f)->_uw )
#define pdFootZPtr(f)      ( &(f)->_z )
#define pdFootCZPtr(f)     ( (f)->_cz )
#define pdFootMaxHeight(f) pdFootZMaxHeight( pdFootZPtr(f) )
#define pdFootRefPos(f)    ( &(f)->refp )
#define pdFootRefPosX(f)   zVec3DElem( pdFootRefPos(f), zX )
#define pdFootRefPosY(f)   zVec3DElem( pdFootRefPos(f), zY )
#define pdFootRefPosZ(f)   zVec3DElem( pdFootRefPos(f), zZ )
#define pdFootRefAtt(f)    ( &(f)->refa )
#define pdFootRefAttX(f)   zVec3DElem( pdFootRefAtt(f), zX )
#define pdFootRefAttY(f)   zVec3DElem( pdFootRefAtt(f), zY )
#define pdFootRefAttZ(f)   zVec3DElem( pdFootRefAtt(f), zZ )

/* methods to set parameters */
#define pdFootSetTime(f,t)        ( (f)->_sol._t = (t) )
#define pdFootResetTime(f)        pdFootSetTime( f, 0 )
#define pdFootSetTimeStep(f,dt)   ( (f)->_sol._dt = (dt) )
#define pdFootIncrTime(f)         ( pdFootTime( f ) += pdFootTimeStep( f ) )
#define pdFootSetPos(f,x,y,z)     zVec3DCreate( pdFootPos(f), x, y, z )
#define pdFootSetPosVec(f,v)      zVec3DCopy( v, pdFootPos(f) )
#define pdFootSetPosX(f,x)        zVec3DSetElem( pdFootPos(f), zX, x )
#define pdFootSetPosY(f,y)        zVec3DSetElem( pdFootPos(f), zY, y )
#define pdFootSetPosZ(f,z)        zVec3DSetElem( pdFootPos(f), zZ, z )
#define pdFootSetDesPos(f,x,y,z)  zVec3DCreate( pdFootDesPos(f), x, y, z )
#define pdFootSetDesPosVec(f,v)   zVec3DCopy( v, pdFootDesPos(f) )
#define pdFootSetDesPosX(f,x)     zVec3DSetElem( pdFootDesPos(f), zX, x )
#define pdFootSetDesPosY(f,y)     zVec3DSetElem( pdFootDesPos(f), zY, y )
#define pdFootSetDesPosZ(f,z)     zVec3DSetElem( pdFootDesPos(f), zZ, z )
#define pdFootSetAtt(f,x,y,z)     zVec3DCreate( pdFootAtt(f), x, y, z )
#define pdFootSetAttVec(f,v)      zVec3DCopy( v, pdFootAtt(f) )
#define pdFootSetAttX(f,x)        zVec3DSetElem( pdFootAtt(f), zX, x )
#define pdFootSetAttY(f,y)        zVec3DSetElem( pdFootAtt(f), zY, y )
#define pdFootSetAttZ(f,z)        zVec3DSetElem( pdFootAtt(f), zZ, z )
#define pdFootSetDesAtt(f,x,y,z)  zVec3DCreate( pdFootDesAtt(f), x, y, z )
#define pdFootSetDesAttVec(f,v)   zVec3DCopy( v, pdFootDesAtt(f) )
#define pdFootSetDesAttX(f,x)     zVec3DSetElem( pdFootDesAtt(f), zX, x )
#define pdFootSetDesAttY(f,y)     zVec3DSetElem( pdFootDesAtt(f), zY, y )
#define pdFootSetDesAttZ(f,z)     zVec3DSetElem( pdFootDesAtt(f), zZ, z )
#define pdFootSetSR(f,sr)         ( (f)->_sr = (sr) )
#define pdFootSetTrXK(f,k)        ( (f)->_sol._k[0] = (k) )
#define pdFootSetTrXC(f,c)        ( (f)->_sol._c[0] = (c) )
#define pdFootSetTrXOld(f,old)    ( (f)->_sol._old[0] = (old) )
#define pdFootSetTrYK(f,k)        ( (f)->_sol._k[1] = (k) )
#define pdFootSetTrYC(f,c)        ( (f)->_sol._c[1] = (c) )
#define pdFootSetTrYOld(f,old)    ( (f)->_sol._old[1] = (old) )
#define pdFootSetTrZK(f,k)        ( (f)->_sol._k[2] = (k) )
#define pdFootSetTrZC(f,c)        ( (f)->_sol._c[2] = (c) )
#define pdFootSetTrZOld(f,old)    ( (f)->_sol._old[2] = (old) )
#define pdFootSetMaxHeight(f,h)   pdFootZSetMaxHeight( pdFootZPtr(f), h )

/* calculation method */
#define pdFootRotUWtoXY(f,vuw,vxy) pdCZHrzRotUWtoXY( pdFootCZPtr(f), vuw, vxy )
#define pdFootRotXYtoUW(f,vxy,vuw) pdCZHrzRotXYtoUW( pdFootCZPtr(f), vxy, vuw )
#define pdFootXformUWtoXY(f,uw,xy) pdCZHrzXformUWtoXY( pdFootCZPtr(f), uw, xy )
#define pdFootXformXYtoUW(f,xy,uw) pdCZHrzXformXYtoUW( pdFootCZPtr(f), xy, uw )
__EXPORT void pdFootXformSRXYtoUW(pdFoot *f, zVec3DList *sr);
__EXPORT void pdFootCalcRefPos(pdFoot *f, zVec3D *p, zVec3D *pd, zVec3D *refp);
__EXPORT void pdFootCalcRefAtt(pdFoot *f, zVec3D *pd, zVec3D *refa);

/* update */
__EXPORT void pdFootUpdate(pdFoot *lf, pdFoot *rf, zVec2D delta, zVec2D vel, zVec2D zmp, zVec3D *lfp, zVec3D *rfp, zVec3DList *lfsr, zVec3DList *rfsr);

__END_DECLS

#endif /* __PD_FOOT_H__ */
