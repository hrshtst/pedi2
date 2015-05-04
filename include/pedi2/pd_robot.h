#ifndef __PD_ROBOT_H__
#define __PD_ROBOT_H__

#include <roki/rk_ik.h>

__BEGIN_DECLS

typedef struct{
  rkChain _chain;    /* robot kinematics/dynamics model */
  rkIK _ik;          /* IK solver */
  rkIKCell **_cell;  /* IK cell */
  int _num_cell;     /* number of IK cell */
  zVec _dis;         /* displacement vector */
} pdRobot;

/* c'tor and d'tor */
void pdRobotInit(pdRobot *robot);
void pdRobotLoad(pdRobot *robot, const char model[]);
void pdRobotDestroy(pdRobot *robot);

/* methods to get parameters */
#define pdRobotChainPtr(r) ( &(r)->_chain )
#define pdRobotIKPtr(r)    ( &(r)->_ik )

/* methods to solve IK */
__EXPORT void pdRobotSolveIK(pdRobot *robot);

#define pdRobotJointSize(r) rkChainJointSize( pdRobotChainPtr(r) )
#define pdRobotJointDis(r)  (r)->_dis

__END_DECLS

#endif /* __PD_ROBOT_H__ */
