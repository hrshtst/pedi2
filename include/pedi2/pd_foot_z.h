#ifndef __PD_FOOT_Z_H__
#define __PD_FOOT_Z_H__

#include <zm/zm_complex.h>
#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

typedef struct{
  pdCZHrzUW *_czuw;

  double _h;         /* maximum lifting height */
  zVec3DList *_sr;   /* supporting region */
  double _sign;      /* for calculation (left: +1, right: -1) */

  zComplex pz;       /* ZMP phase */
  double phase;      /* lifting phase */
  double refz;       /* referential lifting height */
} pdFootZ;

/* c'tor and d'tor */
__EXPORT void pdFootZInit(pdFootZ *fz, pdCZHrzUW *czuw);
__EXPORT void pdFootZDestroy(pdFootZ *fz);

/* methods to get parameters */
#define pdFootZCZPtr(f)      (f)->_czuw
#define pdFootZMaxHeight(f) (f)->_h
#define pdFootZSR(f)         (f)->_sr
#define pdFootZSign(f)       (f)->_sign
#define pdFootZZMPPhase(f)  ( &(f)->pz )
#define pdFootZFootPhase(f) (f)->phase
#define pdFootZRefHeight(f) (f)->refz

/* methods to set parameters */
#define pdFootZSetMaxHeight(f,h) ( pdFootZMaxHeight(f) = (h) )
#define pdFootZSetSR(f,sr)        ( pdFootZSR(f) = (sr) )

__END_DECLS

#endif /* __PD_FOOT_Z_H__ */
