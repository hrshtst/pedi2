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
  zVec3D _pp;         /* foot position when pivoting */
  zVec3D _a;          /* current foot attitude (ZYX Euler angle) */
  zVec3D _ad;         /* desired foot attitude (ZYX Euler angle) */
  zVec3D _ap;         /* foot attitude when pivoting */
  zLoop3D *_sr;       /* supporting region */

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
__PEDI2_EXPORT void pdFootInit(pdFoot *f, pdCZHrz *cz, byte dir, double dt);
__PEDI2_EXPORT void pdFootDestroy(pdFoot *f);

/* methods to get parameters */
#define pdFootTime(f)      (f)->_sol._t
#define pdFootTimeStep(f)  (f)->_sol._dt
#define pdFootPos(f)       ( &(f)->_p )
#define pdFootPosX(f)      pdFootPos(f)->c.x
#define pdFootPosY(f)      pdFootPos(f)->c.y
#define pdFootPosZ(f)      pdFootPos(f)->c.z
#define pdFootDesPos(f)    ( &(f)->_pd )
#define pdFootDesPosX(f)   pdFootDesPos(f)->c.x
#define pdFootDesPosY(f)   pdFootDesPos(f)->c.y
#define pdFootDesPosZ(f)   pdFootDesPos(f)->c.z
#define pdFootPivotPos(f)  ( &(f)->_pp )
#define pdFootPivotPosX(f) pdFootPivotPos(f)->c.x
#define pdFootPivotPosY(f) pdFootPivotPos(f)->c.y
#define pdFootPivotPosZ(f) pdFootPivotPos(f)->c.z
#define pdFootAtt(f)       ( &(f)->_a )
#define pdFootAttX(f)      pdFootAtt(f)->c.x
#define pdFootAttY(f)      pdFootAtt(f)->c.y
#define pdFootAttZ(f)      pdFootAtt(f)->c.z
#define pdFootDesAtt(f)    ( &(f)->_ad )
#define pdFootDesAttX(f)   pdFootDesAtt(f)->c.x
#define pdFootDesAttY(f)   pdFootDesAtt(f)->c.y
#define pdFootDesAttZ(f)   pdFootDesAtt(f)->c.z
#define pdFootPivotAtt(f)  ( &(f)->_ap )
#define pdFootPivotAttX(f) pdFootPivotAtt(f)->c.x
#define pdFootPivotAttY(f) pdFootPivotAtt(f)->c.y
#define pdFootPivotAttZ(f) pdFootPivotAtt(f)->c.z
#define pdFootSR(f)        (f)->_sr
#define pdFootUWPtr(f)     ( &(f)->_uw )
#define pdFootZPtr(f)      ( &(f)->_z )
#define pdFootCZPtr(f)     ( (f)->_cz )
#define pdFootMaxHeight(f) pdFootZMaxHeight( pdFootZPtr(f) )
#define pdFootRefPos(f)    ( &(f)->refp )
#define pdFootRefPosX(f)   pdFootRefPos(f)->c.x
#define pdFootRefPosY(f)   pdFootRefPos(f)->c.y
#define pdFootRefPosZ(f)   pdFootRefPos(f)->c.z
#define pdFootRefAtt(f)    ( &(f)->refa )
#define pdFootRefAttX(f)   pdFootRefAtt(f)->c.x
#define pdFootRefAttY(f)   pdFootRefAtt(f)->c.y
#define pdFootRefAttZ(f)   pdFootRefAtt(f)->c.z
#define pdFootZMPPhase(f)  pdFootZZMPPhase( pdFootZPtr(f) )
#define pdFootPhase(f)     pdFootZFootPhase( pdFootZPtr(f) )
#define pdFootPhi(f)       pdFootUWPhi( pdFootUWPtr(f) )
#define pdFootRegZMP(f)    pdFootUWRegZMP( pdFootUWPtr(f) )
#define pdFootRegZMPU(f)   pdFootUWRegZMPU( pdFootUWPtr(f) )
#define pdFootRegZMPW(f)   pdFootUWRegZMPW( pdFootUWPtr(f) )

/* methods to set parameters */
#define pdFootSetTime(f,t)        ( (f)->_sol._t = (t) )
#define pdFootResetTime(f)        pdFootSetTime( f, 0 )
#define pdFootSetTimeStep(f,dt)   ( (f)->_sol._dt = (dt) )
#define pdFootIncrTime(f)         ( pdFootTime( f ) += pdFootTimeStep( f ) )
#define pdFootSetPos(f,x,y,z)     zVec3DCreate( pdFootPos(f), x, y, z )
#define pdFootSetPosVec(f,v)      zVec3DCopy( v, pdFootPos(f) )
#define pdFootSetPosX(f,x)        ( pdFootPosX(f) = (x) )
#define pdFootSetPosY(f,y)        ( pdFootPosY(f) = (y) )
#define pdFootSetPosZ(f,z)        ( pdFootPosZ(f) = (z) )
#define pdFootSetDesPos(f,x,y,z)  zVec3DCreate( pdFootDesPos(f), x, y, z )
#define pdFootSetDesPosVec(f,v)   zVec3DCopy( v, pdFootDesPos(f) )
#define pdFootSetDesPosX(f,x)     ( pdFootDesPosX(f) = (x) )
#define pdFootSetDesPosY(f,y)     ( pdFootDesPosY(f) = (y) )
#define pdFootSetDesPosZ(f,z)     ( pdFootDesPosZ(f) = (z) )
#define pdFootSetAtt(f,x,y,z)     zVec3DCreate( pdFootAtt(f), x, y, z )
#define pdFootSetAttVec(f,v)      zVec3DCopy( v, pdFootAtt(f) )
#define pdFootSetAttX(f,x)        ( pdFootAttX(f) = (x) )
#define pdFootSetAttY(f,y)        ( pdFootAttY(f) = (y) )
#define pdFootSetAttZ(f,z)        ( pdFootAttZ(f) = (z) )
#define pdFootSetDesAtt(f,x,y,z)  zVec3DCreate( pdFootDesAtt(f), x, y, z )
#define pdFootSetDesAttVec(f,v)   zVec3DCopy( v, pdFootDesAtt(f) )
#define pdFootSetDesAttX(f,x)     ( pdFootDesAttX(f) = (x) )
#define pdFootSetDesAttY(f,y)     ( pdFootDesAttY(f) = (y) )
#define pdFootSetDesAttZ(f,z)     ( pdFootDesAttZ(f) = (z) )
#define pdFootSetSR(f,sr)         ( (f)->_sr = (sr) )
#define pdFootSetMaxHeight(f,h)   pdFootZSetMaxHeight( pdFootZPtr(f), h )
#define pdFootSetTrXK(f,k)        ( (f)->_sol._k[0] = (k) )
#define pdFootSetTrXC(f,c)        ( (f)->_sol._c[0] = (c) )
#define pdFootSetTrXOld(f,old)    ( (f)->_sol._old[0] = (old) )
#define pdFootSetTrYK(f,k)        ( (f)->_sol._k[1] = (k) )
#define pdFootSetTrYC(f,c)        ( (f)->_sol._c[1] = (c) )
#define pdFootSetTrYOld(f,old)    ( (f)->_sol._old[1] = (old) )
#define pdFootSetTrZK(f,k)        ( (f)->_sol._k[2] = (k) )
#define pdFootSetTrZC(f,c)        ( (f)->_sol._c[2] = (c) )
#define pdFootSetTrZOld(f,old)    ( (f)->_sol._old[2] = (old) )
#define pdFootSetTrK(f,x,y,z) do{ \
  pdFootSetTrXK( f, x ); \
  pdFootSetTrYK( f, y ); \
  pdFootSetTrZK( f, z ); \
} while(0)
#define pdFootSetTrC(f,x,y,z) do{ \
  pdFootSetTrXC( f, x ); \
  pdFootSetTrYC( f, y ); \
  pdFootSetTrZC( f, z ); \
} while(0)
#define pdFootSetTrOld(f,x,y,z) do{ \
  pdFootSetTrXOld( f, x ); \
  pdFootSetTrYOld( f, y ); \
  pdFootSetTrZOld( f, z ); \
} while(0)
#define pdFootSetTrOldVec(f,v) do{ \
  pdFootSetTrXOld( f, (v)->c.x ); \
  pdFootSetTrYOld( f, (v)->c.y ); \
  pdFootSetTrZOld( f, (v)->c.z ); \
} while(0)

/* calculation method */
__PEDI2_EXPORT bool pdFootIsOn(pdFoot *f);
__PEDI2_EXPORT bool pdFootIsOnNext(pdFoot *f);
__PEDI2_EXPORT bool pdFootIsOnAttempt(pdFoot *f);
#define pdFootIsOff(f) !pdFootIsOn( f )
#define pdFootIsOffNext(f) !pdFootIsOnNext( f )
#define pdFootIsOffAttempt(f) !pdFootIsOnAttempt( f )

#define pdFootIsBothOn(lf,rf) ( pdFootIsOn( lf ) && pdFootIsOn( rf ) )
#define pdFootIsEitherOn(lf,rf) ( pdFootIsOn( lf ) || pdFootIsOn( rf ) )
#define pdFootIsEitherOff(lf,rf) ( pdFootIsOff( lf ) || pdFootIsOff( rf ) )
#define pdFootIsBothOnNext(lf,rf) ( pdFootIsOnNext( lf ) && pdFootIsOnNext( rf ) )
#define pdFootIsEitherOnNext(lf,rf) ( pdFootIsOnNext( lf ) || pdFootIsOnNext( rf ) )
#define pdFootIsEitherOffNext(lf,rf) ( pdFootIsOffNext( lf ) || pdFootIsOffNext( rf ) )
#define pdFootIsBothOnAttempt(lf,rf) ( pdFootIsOnAttempt( lf ) && pdFootIsOnAttempt( rf ) )
#define pdFootIsEitherOnAttempt(lf,rf) ( pdFootIsOnAttempt( lf ) || pdFootIsOnAttempt( rf ) )
#define pdFootIsEitherOffAttempt(lf,rf) ( pdFootIsOffAttempt( lf ) || pdFootIsOffAttempt( rf ) )

__PEDI2_EXPORT pdFoot *pdFootKFPtr(pdFoot *lf, pdFoot *rf);
__PEDI2_EXPORT pdFoot *pdFootFFPtr(pdFoot *lf, pdFoot *rf, double vwd);
__PEDI2_EXPORT pdFoot *pdFootBFPtr(pdFoot *lf, pdFoot *rf, double vwd);

#define pdFootRotUWtoXY(f,vuw,vxy) pdCZHrzRotUWtoXY( pdFootCZPtr(f), vuw, vxy )
#define pdFootRotXYtoUW(f,vxy,vuw) pdCZHrzRotXYtoUW( pdFootCZPtr(f), vxy, vuw )
#define pdFootXformUWtoXY(f,uw,xy) pdCZHrzXformUWtoXY( pdFootCZPtr(f), uw, xy )
#define pdFootXformXYtoUW(f,xy,uw) pdCZHrzXformXYtoUW( pdFootCZPtr(f), xy, uw )
__PEDI2_EXPORT void pdFootXformSRXYtoUW(pdFoot *f, zLoop3D *sr);
__PEDI2_EXPORT void pdFootCalcRefPos(pdFoot *f, zVec3D *p, zVec3D *pd, zVec3D *refp);
__PEDI2_EXPORT void pdFootCalcRefAtt(pdFoot *f, zVec3D *pd, zVec3D *refa);
__PEDI2_EXPORT void pdFootCalcCOMRefPos(pdFoot *lf, pdFoot *rf, zVec3D *lf_pos, zVec3D *rf_pos, zVec3D *ref_pos);

/* update */
__PEDI2_EXPORT void pdFootUpdateState(pdFoot *f, zVec3D *pos, zVec3D *att, zLoop3D *sr);
__PEDI2_EXPORT void pdFootUpdate(pdFoot *lf, pdFoot *rf, zVec2D *delta, zVec2D *vel, zVec3D *zmp, zVec3D *lfp, zVec3D *rfp, zVec3D *lfa, zVec3D *rfa, zLoop3D *lfsr, zLoop3D *rfsr, double refdist);

/* output */
__PEDI2_EXPORT void pdFootFWrite(FILE *fp, pdFoot *lf, pdFoot *rf);
#define pdFootWrite(lf,rf) pdFootFWrite( stdout, lf, rf )
__PEDI2_EXPORT void pdFootDataFWrite(FILE *fp, pdFoot *lf, pdFoot *rf);
#define pdFootDataWrite(lf,rf) pdFootDataFWrite( stdout, lf, rf )

__END_DECLS

#endif /* __PD_FOOT_H__ */
