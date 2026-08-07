#ifndef __PD_FOOT_UW_H__
#define __PD_FOOT_UW_H__

#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

typedef struct _pdFootUW pdFootUW;

/* landing-target override: called at the end of pdFootUWUpdate with the
 * default target already stored in refpos; it may replace or modify the
 * target, which is expressed in the COM-centered UW frame.  Since the
 * override runs outside of pdFootUWCalcRefPos, it is free to call that
 * function with its own anchor point in place of the regulated ZMP to
 * reuse the default placement rule, including the guard that keeps the
 * foot outboard of its nominal station (half the canonical foot
 * distance beside the desired COM) so that the feet never cross */
typedef void (*pdFootUWLandingOverride)(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos, void *util);

struct _pdFootUW{
  pdCZHrzUW *_czuw;

  double _sign;   /* for calculation (left: +1, right: -1) */

  double phi;     /* rotational angle */
  zVec2D regzmp;  /* regulated ZMP */
  zVec2D refpos;  /* referential foot position */

  pdFootUWLandingOverride _landing_fn; /* landing-target override (NULL for the default rule) */
  void *_landing_util;                 /* utility workspace for the override */
};

/* c'tor and d'tor */
__PEDI2_EXPORT void pdFootUWInit(pdFootUW *fuw, pdCZHrzUW *czuw);
__PEDI2_EXPORT void pdFootUWDestroy(pdFootUW *fuw);

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
#define pdFootUWLandingFn(f)   (f)->_landing_fn
#define pdFootUWLandingUtil(f) (f)->_landing_util
#define pdFootUWKappa(f)   pdCZHrzUWKappa( pdFootUWCZPtr(f) )
#define pdFootUWDist(f)    pdCZHrzUWDist( pdFootUWCZPtr(f) )
#define pdFootUWZeta(f)    pdCZHrzUWZeta( pdFootUWCZPtr(f) )

/* calculation method */
#define pdFootUWCalcRegZMP(f,d,v,z) pdCZHrzUWCalcRegZMP( pdFootUWCZPtr(f), d, v, z )
#define pdFootUWCalcRegZMPU(f,d,v)  pdCZHrzUWCalcRegZMPU( pdFootUWCZPtr(f), d, v )
#define pdFootUWCalcRegZMPW(f,d,v)  pdCZHrzUWCalcRegZMPW( pdFootUWCZPtr(f), d, v )
__PEDI2_EXPORT double pdFootUWCalcPhi(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp);
__PEDI2_EXPORT void pdFootUWCalcRefPos(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos);
__PEDI2_EXPORT void pdFootUWCalcCOMRefPos(zVec2D *lf_pos, zVec2D *rf_pos, zVec2D *ref_pos);

/* update method */
__PEDI2_EXPORT void pdFootUWUpdate(pdFootUW *kf, zVec2D *delta, zVec2D *vel);

/* output method */
__PEDI2_EXPORT void pdFootUWFWrite(FILE *fp, pdFootUW *fuw);
#define pdFootUWWrite(f) pdFootUWFWrite( stdout, f )

__END_DECLS

#endif /* __PD_FOOT_UW_H__ */
