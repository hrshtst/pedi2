#ifndef __PD_FOOT_UW_H__
#define __PD_FOOT_UW_H__

#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

typedef struct{
  pdCZHrzUW *_czuw;

  double _sign;   /* for calculation (left: +1, right: -1) */

  double phi;     /* rotational angle */
  zVec2D regzmp;  /* regulated ZMP */
  zVec2D refpos;  /* referential foot position */
} pdFootUW;

/* c'tor and d'tor */
__EXPORT void pdFootUWInit(pdFootUW *fuw, pdCZHrzUW *czuw);
__EXPORT void pdFootUWDestroy(pdFootUW *fuw);

/* methods to get parameters */
#define pdFootUWCZPtr(f)   (f)->_czuw
#define pdFootUWSign(f)    (f)->_sign
#define pdFootUWPhi(f)     (f)->phi
#define pdFootUWRegZMP(f)  ( &(f)->regzmp )
#define pdFootUWRegZMPU(f) pdFootUWRegZMP(f)->e[pdU]
#define pdFootUWRegZMPW(f) pdFootUWRegZMP(f)->e[pdW]
#define pdFootUWRefPos(f)  ( &(f)->refpos )
#define pdFootUWRefPosU(f) pdFootUWRefPos(f)->e[pdU]
#define pdFootUWRefPosW(f) pdFootUWRefPos(f)->e[pdW]
#define pdFootUWKappa(f)   pdCZHrzUWKappa( pdFootUWCZPtr(f) )
#define pdFootUWDist(f)    pdCZHrzUWDist( pdFootUWCZPtr(f) )

/* calculation method */
#define pdFootUWCalcRegZMP(f,d,v,z) pdCZHrzUWCalcRegZMP( pdFootUWCZPtr(f), d, v, z )
#define pdFootUWCalcRegZMPU(f,d,v)  pdCZHrzUWCalcRegZMPU( pdFootUWCZPtr(f), d, v )
#define pdFootUWCalcRegZMPW(f,d,v)  pdCZHrzUWCalcRegZMPW( pdFootUWCZPtr(f), d, v )
__EXPORT double pdFootUWCalcPhi(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp);
__EXPORT void pdFootUWCalcRefPos(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos);
__EXPORT void pdFootUWCalcCOMRefPos(zVec2D *lf_pos, zVec2D *rf_pos, zVec2D *ref_pos);

/* update method */
__EXPORT void pdFootUWUpdate(pdFootUW *kf, zVec2D *delta, zVec2D *vel);

/* output method */
__EXPORT void pdFootUWFWrite(FILE *fp, pdFootUW *fuw);
#define pdFootUWWrite(f) pdFootUWFWrite( stdout, f )

__END_DECLS

#endif /* __PD_FOOT_UW_H__ */
