#ifndef __PD_FOOT_Z_H__
#define __PD_FOOT_Z_H__

#include <pedi2/pd_cz_hrz_uw.h>

__BEGIN_DECLS

typedef struct{
  pdCZHrzUW *_czuw;

  double _h;           /* maximum lifting height */
  zLoop3D _sr;         /* supporting region */
  zVec3DData _sr_vert; /* vertices of supporting region */
  int _vert_num;       /* number of vertices */
  double _sign;        /* for calculation (left: +1, right: -1) */

  zComplex pz;         /* ZMP phase */
  double phase;        /* lifting phase */
  double refz;         /* referential lifting height */
} pdFootZ;

/* c'tor and d'tor */
__EXPORT void pdFootZInit(pdFootZ *fz, pdCZHrzUW *czuw);
__EXPORT void pdFootZDestroy(pdFootZ *fz);

/* methods to get parameters */
#define pdFootZCZPtr(f)     (f)->_czuw
#define pdFootZMaxHeight(f) (f)->_h
#define pdFootZSR(f)        ( &(f)->_sr )
#define pdFootZSRVert(f)    ( &(f)->_sr_vert )
#define pdFootZSign(f)      (f)->_sign
#define pdFootZZMPPhase(f)  ( &(f)->pz )
#define pdFootZFootPhase(f) (f)->phase
#define pdFootZRefZ(f)      (f)->refz
#define pdFootZRho(f)       pdCZHrzUWRho( pdFootZCZPtr(f) )
#define pdFootZDist(f)      pdCZHrzUWDist( pdFootZCZPtr(f) )

/* methods to set parameters */
#define pdFootZSetMaxHeight(f,h) ( pdFootZMaxHeight(f) = (h) )
__EXPORT void pdFootZSetSR(pdFootZ *fz, zVec3D p[], int num);
#define pdFootZIsSRSet(f) ( zListSize( pdFootZSR(f) ) ? true : false )

/* calculation method */
#define pdFootZCalcZMPPhase(f,d,v,z,pz) pdCZHrzUWCalcZMPPhase( pdFootZCZPtr(f), d, v, z, pz )
__EXPORT int pdFootZFindIntersection(pdFootZ *f, zVec2D *zmp, zVec3D ip[]);
__EXPORT int pdFootZFindInnerPoint(pdFootZ *f, zVec2D *zmp, zVec3D *p);
__EXPORT double pdFootZCalcFootPhase(pdFootZ *pf, zVec2D *delta, zVec2D *vel, zVec2D *zmp);
__EXPORT double pdFootZCalcRefZ(pdFootZ *pf, double phase, zComplex *pz);

/* update method */
__EXPORT void pdFootZUpdate(pdFootZ *pf, pdFootZ *kf, zVec2D *delta, zVec2D *vel, zVec2D *zmp);

/* output method */
__EXPORT void pdFootZFWrite(FILE *fp, pdFootZ *fz);
#define pdFootZWrite(f) pdFootZFWrite( stdout, f )

__END_DECLS

#endif /* __PD_FOOT_Z_H__ */
