#ifndef __PD_ROBOT_H__
#define __PD_ROBOT_H__

#include <roki/rk_ik.h>

__BEGIN_DECLS

typedef struct{
  rkChain _chain;    /* robot kinematics/dynamics model */
  rkIK _ik;          /* IK solver */
} pdRobot;

/* c'tor and d'tor */
void pdRobotInit(pdRobot *robot);
void pdRobotDestroy(pdRobot *robot);

/* methods to get parameters */
#define pdRobotChainPtr(r) ( &(r)->_chain )
#define pdRobotIKPtr(r)    ( &(r)->_ik )

__END_DECLS

#endif /* __PD_ROBOT_H__ */
