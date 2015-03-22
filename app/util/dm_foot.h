#ifndef __DM_FOOT_H__
#define __DM_FOOT_H__

#include <zeo/zeo_vec3d.h>
#include <pedi2/pd_cz.h>

void dmCtrlZMPPhase(pdCZ *c, double vw, zComplex *pz);

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
  double yin, yout;   /* supporting region */
  double xsfor, xsaft; /* stepping range */
  double ysin, ysout; /* stepping range */
  double xfc, yfc;    /* canonical distance from COM to each foot */
  double phase;       /* lifting phase */
  double xk0, yk0;    /* initial position of swing foot */
  bool is_ff, is_bf;  /* boolean if the foot is following or braking */
} dmFoot;

void dmFootPhase(dmFoot *pf, dmFoot *kf, double xd, double yd, double theta, zComplex *pz);
void dmFootLift(pdCZ *ctrl, dmFoot *lf, dmFoot *rf, double xd, double yd, double theta, zComplex *pz);
bool dmFootIsCurrentFootFloating(dmFoot *f);
bool dmFootIsDesiredFootFloating(dmFoot *f);
bool dmFootIsDesiredFootForSeedFloating(dmFoot *f);
void dmFootStep(pdCZ *ctrl, dmFoot *pf, dmFoot *kf, double xd, double yd, double theta);
void dmFootMove(pdCZ *ctrl, dmFoot *lf, dmFoot *rf, double xd, double yd, double theta);
void dmFootUpdatePos(dmFoot *lf, dmFoot *rf);

#endif /* __DM_FOOT_H__ */
