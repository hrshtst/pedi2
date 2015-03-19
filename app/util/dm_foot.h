#ifndef __DM_FOOT_H__
#define __DM_FOOT_H__

#include <zeo/zeo_vec3d.h>
#include <pedi2/pd_ctrl.h>

/* foot property (sideway properties only) */
typedef struct{
  double h;           /* maximum lifting height */
  zVec3D p;           /* current position */
  zVec3D pds;         /* desired position for IK seeds */
  zVec3D pd;          /* desired position */
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

void dmCtrlFootPhase(pdCtrl *c, dmFoot *lf, dmFoot *rf, zComplex *pz);
void dmCtrlFootLift(pdCtrl *c, dmFoot *lf, dmFoot *rf, zComplex *pz);
bool dmCtrlIsCurrentFootFloating(dmFoot *f);
bool dmCtrlIsDesiredFootFloating(dmFoot *f);
bool dmCtrlIsDesiredFootForSeedFloating(dmFoot *f);
void dmCtrlFootStep(pdCtrl *cx, pdCtrl *cy, dmFoot *pf, dmFoot *kf, double sxz, double syz, double y, double vy);
void dmCtrlFootStepMidpoint(pdCtrl *cx, pdCtrl *cy, dmFoot *pf, dmFoot *kf, double sxz, double syz);
void dmCtrlFootStepCanonical(pdCtrl *cx, pdCtrl *cy, dmFoot *pf, dmFoot *kf, double sxz, double syz);
void dmCtrlFootStepAsymptotic(pdCtrl *cx, pdCtrl *cy, dmFoot *pf, dmFoot *kf, double sxz, double syz, double y, double vy);
void dmCtrlFootMove(pdCtrl *cx, pdCtrl *cy, dmFoot *lf, dmFoot *rf, double sxz, double syz, double y, double vy);
void dmCtrlFootUpdatePos(dmFoot *lf, dmFoot *rf);

#define DT 0.01

#endif /* __DM_FOOT_H__ */
