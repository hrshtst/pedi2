#ifndef __PD_FOOT_UW_H__
#define __PD_FOOT_UW_H__

#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

enum{
  PD_FOOT_INVALID=-1,
  PD_FOOT_LEFT=0,
  PD_FOOT_RIGHT
};

typedef struct{
  pdCZHrzUW *_czuw;

  double _sign;   /* for calculation (left: +1, right: -1) */

  double phi;     /* rotational angle */
  zVec2D regzmp;  /* regulated ZMP */
  zVec2D refpos;  /* referential foot position */
} pdFootUW;

/* c'tor and d'tor */
__EXPORT void pdFootUWInit(pdFootUW *fuw, pdCZHrzUW *czuw, byte dir);
__EXPORT void pdFootUWDestroy(pdFootUW *fuw);

/* methods to get parameters */
#define pdFootUWCZPtr(f)   (f)->_czuw
#define pdFootUWSign(f)    (f)->_sign
#define pdFootUWPhi(f)     (f)->phi
#define pdFootUWRegZMP(f)  (f)->regzmp
#define pdFootUWRegZMPU(f) pdFootUWRegZMP(f)[pdU]
#define pdFootUWRegZMPW(f) pdFootUWRegZMP(f)[pdW]
#define pdFootUWRefPos(f)  (f)->refpos
#define pdFootUWRefPosU(f) pdFootUWRefPos(f)[pdU]
#define pdFootUWRefPosW(f) pdFootUWRefPos(f)[pdW]
#define pdFootUWKappa(f)   pdCZHrzUWKappa( pdFootUWCZPtr(f) )

__END_DECLS

#endif /* __PD_FOOT_UW_H__ */
