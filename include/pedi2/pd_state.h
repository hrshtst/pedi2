#ifndef __PD_STATE_H__
#define __PD_STATE_H__

#include <zeo/zeo_vec3d.h>
#include <zeo/zeo_bv3d.h>
#include <zeo/zeo_mat3d.h>

__BEGIN_DECLS

typedef struct{
  zVec3D com_pos;
  zVec3D com_vel;
  zVec3D com_acc;
  zVec3D lf_pos, rf_pos;
  zVec3D lh_pos, rh_pos;
  /* zMat3D base_att; */
  /* zMat3D lf_att, rf_att; */
  /* zMat3D lh_att, rh_att; */
  zVec3D base_att;
  zVec3D lf_att, rf_att;
  zVec3D lh_att, rh_att;
  zVec3D zmp;       /* actual ZMP position */
  zVec3D deszmp;    /* desired ZMP position */
  double fz;
  zVec3D ef;        /* external force */
  zLoop3D sr_lf;
  zLoop3D sr_rf;
  zLoop3D sr;
} pdState;

/* c'tor and d'tor */
__EXPORT void pdStateInit(pdState *state);
__EXPORT void pdStateDestroy(pdState *state);

/* methods to get parameters */
__EXPORT bool pdStateFootIsOn(pdState *state, zVec3D *p, zLoop3D *sr);
#define pdStateFootIsOff(s,p,sr) !pdStateFootIsOn( s, p, sr )
#define pdStateLFIsOn(s)  pdStateFootIsOn( s, &(s)->lf_pos, &(s)->sr_lf )
#define pdStateLFIsOff(s) pdStateFootIsOff( s, &(s)->lf_pos, &(s)->sr_lf )
#define pdStateRFIsOn(s)  pdStateFootIsOn( s, &(s)->rf_pos, &(s)->sr_rf )
#define pdStateRFIsOff(s) pdStateFootIsOff( s, &(s)->rf_pos, &(s)->sr_rf )
#define pdStateBothFeetOn(s) ( pdStateLFIsOn( s ) && pdStateRFIsOn( s ) )
#define pdStateEitherFootOn(s) ( pdStateLFIsOn( s ) || pdStateRFIsOn( s ) )
#define pdStateEitherFootOff(s) ( pdStateLFIsOff( s ) || pdStateRFIsOff( s ) )
__EXPORT bool pdStateFFOn(pdState *state, double vwd);
#define pdStateFFOff(s,vwd) !pdStateFFOn( s, vwd )
__EXPORT bool pdStateBFOn(pdState *state, double vwd);
#define pdStateBFOff(s,vwd) !pdStateBFOn( s, vwd )
__EXPORT double pdStateFootDist(pdState *state);

/* output method  */
__EXPORT void pdStateSRDataFWrite(FILE *fp, pdState *state);
#define pdStateSRDataWrite(s) pdStateSRDataFWrite( stdout, s )

__END_DECLS

#endif /* __PD_STATE_H__ */
