#ifndef __PD_ROBOT_H__
#define __PD_ROBOT_H__

#include <roki/rk_ik.h>

__BEGIN_DECLS

typedef enum{
  PD_ROBOT_IKCELL_ID_INVALID=-1,
  PD_ROBOT_IKCELL_ID_COM=0,
  PD_ROBOT_IKCELL_ID_BASE_ATT,
  PD_ROBOT_IKCELL_ID_LF_POS,
  PD_ROBOT_IKCELL_ID_LF_ATT,
  PD_ROBOT_IKCELL_ID_RF_POS,
  PD_ROBOT_IKCELL_ID_RF_ATT,
  PD_ROBOT_IKCELL_ID_LH_POS,
  PD_ROBOT_IKCELL_ID_LH_ATT,
  PD_ROBOT_IKCELL_ID_RH_POS,
  PD_ROBOT_IKCELL_ID_RH_ATT
} pdRobotIKCellID;

#define PD_ROBOT_REQUIRED_CONST_NUM 10

typedef struct{
  rkChain _chain;      /* robot kinematics/dynamics model */
  rkIK _ik;            /* IK solver */
  rkIKCell **_cell;    /* IK cell */
  int _cell_num;       /* number of IK cell */
  int _base_id;        /* identifier of base link */
  int _lf_id, _rf_id;  /* identifier of foot link */
  int _lh_id, _rh_id;  /* identifief of hand link */
  zVec3D *_ref_vec;    /* reference vector to set as constraint */
  bool *_ref_set_flag; /* flag to set IKCell reference */
  zVec _dis;           /* displacement vector */
} pdRobot;

/* c'tor and d'tor */
void pdRobotInit(pdRobot *robot);
bool pdRobotLoad(pdRobot *robot, const char model[]);
void pdRobotDestroy(pdRobot *robot);

/* methods to get parameters */
#define pdRobotChainPtr(r) ( &(r)->_chain )
#define pdRobotIKPtr(r)    ( &(r)->_ik )
#define pdRobotCellNum(r)  (r)->_cell_num

/* methods to solve IK */
__EXPORT void pdRobotUnsetAllFlags(pdRobot *robot);
__EXPORT void pdRobotSetRefVec(pdRobot *robot, zVec3D *ref, int id);
#define pdRobotSetRefCOM(r,v)     pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_COM )
#define pdRobotSetRefBaseAtt(r,v) pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_BASE_ATT )
#define pdRobotSetRefLFPos(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_LF_POS )
#define pdRobotSetRefLFAtt(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_LF_ATT )
#define pdRobotSetRefRFPos(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_RF_POS )
#define pdRobotSetRefRFAtt(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_RF_ATT )
#define pdRobotSetRefLHPos(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_LH_POS )
#define pdRobotSetRefLHAtt(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_LH_ATT )
#define pdRobotSetRefRHPos(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_RH_POS )
#define pdRobotSetRefRHAtt(r,v)   pdRobotSetRefVec( r, v, PD_ROBOT_IKCELL_ID_RH_ATT )
__EXPORT void pdRobotSolveIK(pdRobot *robot);

#define pdRobotJointSize(r) rkChainJointSize( pdRobotChainPtr(r) )
#define pdRobotJointDis(r)  (r)->_dis

__END_DECLS

#endif /* __PD_ROBOT_H__ */
