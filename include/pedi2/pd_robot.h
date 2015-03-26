#ifndef __PD_ROBOT_H__
#define __PD_ROBOT_H__

#include <roki/rk_ik.h>

__BEGIN_DECLS

typedef struct{
  /* robot kinematics/dynamics model */
  rkChain chain;

  /* IK solver */
  rkIK ik;
  rkIKCell *cell[6]; /* IK cell */
  zVec dis;          /* displacement vector */
  int body_id, lf_id, rf_id;
  zVec3D d_com_pos, d_lf_pos, d_rf_pos;  /* position constraint */
  zVec3D d_body_att, d_lf_att, d_rf_att; /* attitude constraint */

  /* supporting region */
  zVec3DList sr_lf; /* supporting region on the left sole */
  zVec3DList sr_rf; /* supporting region on the right sole */
  zVec3DList sr;    /* supporting region */
  /* contact points */
  zVec3D sr_lf_vert[4], sr_rf_vert[4], sr_vert[8];
} pdRobot;

void pdRobotInit(pdRobot *robot);
void pdRobotLoad(pdRobot *robot, const char* model_file, const char* conf_file);
void pdRobotExit(pdRobot *robot);

void pdRobotSolveIK(pdRobot *robot);
#define pdRobotGetJointSize(r) rkChainJointSize(&(r)->chain)
#define pdRobotGetJointDis(r,v) zVecCopy((r)->dis,v)
void pdRobotSupportRegion(pdRobot *robot);

void pdRobotFootPos(pdRobot *robot, zVec3D *lf, zVec3D *rf);
void pdRobotFootAtt(pdRobot *robot, zVec3D *lf, zVec3D *rf);

__END_DECLS

#endif /* __PD_ROBOT_H__ */
