#ifndef __PD_FOOT_H__
#define __PD_FOOT_H__

#include <zeo/zeo_vec3d.h>
#include <pedi2/pd_cz.h>

__BEGIN_DECLS

/* foot property (sideway properties only) */
typedef struct{
  double h;           /* maximum lifting height */
  zVec3D p;           /* current position */
  zVec3D ps;          /* desired position for IK seeds */
  zVec3D pd;          /* desired position */
  zVec3D a;           /* current attitude */
  zVec3D as;          /* desired attitude for IK seeds */
  double stride_x, track_kx, track_cx, track_xold;
  double stride_y, track_ky, track_cy, track_yold;
  double track_kz, track_cz, track_zold;
  double dy;          /* direction from inner side to outer side */
  double sole_w;      /* sole width */
  double phase;       /* lifting phase */
} pdFoot;

void pdFootPhase(pdFoot *pf, pdFoot *kf, double xd, double yd, double theta, zComplex *pz);

__END_DECLS

#endif /* __PD_FOOT_H__ */
