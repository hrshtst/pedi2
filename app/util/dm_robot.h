#ifndef __DM_ROBOT_H__
#define __DM_ROBOT_H__

#include <roki/rk_ik.h>
#include <roki/glrk_chain.h>

/* robot kinematics/dynamics model */
extern rkChain dm_robot;

/* IK seeds */
extern zVec3D dm_d_com, dm_d_lf, dm_d_rf;

void dmRobotInit(void);
void dmRobotExit(void);

void dmRobotSolveIK(void);

void dmSupportRegion(void);
void dmSupportRegionBorder(double *xmin, double *xmax, double *ymin, double *ymax);

/* dummy --- should handle 2D phase estimation */
void dmRobotFootPos(zVec3D *lf, zVec3D *rf);
void dmRobotFootRegion(double *ylout, double *ylin, double *dyl, double *yrout, double *yrin, double *dyr);

/* scene */

void dmGLGauge(void);
void dmGLRobot(void);
void dmGLSupportRegion(void);

#endif /* __DM_ROBOT_H__ */
