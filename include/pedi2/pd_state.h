#ifndef __PD_STATE_H__
#define __PD_STATE_H__

#include <zeo/zeo_vec3d.h>
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
  zVec3D zmp;
  double fz;
  zVec3DList *sr_lf;
  zVec3DList *sr_rf;
  zVec3DList *sr;
} pdState;

/* c'tor and d'tor */
__EXPORT void pdStateInit(pdState *state);
__EXPORT void pdStateDestroy(pdState *state);

__END_DECLS

#endif /* __PD_STATE_H__ */
