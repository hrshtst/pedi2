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
  zVec3D *_ref_vec;    /* reference vector to set as constraint */
  bool *_ref_set_flag; /* flag to set IKCell reference */

  int _base_id;        /* identifier of base link */
  int _lf_id, _rf_id;  /* identifier of foot link */
  int _lh_id, _rh_id;  /* identifief of hand link */

  zVec dis;           /* displacement vector */
} pdRobot;

/* c'tor and d'tor */
void pdRobotInit(pdRobot *robot);
bool pdRobotLoad(pdRobot *robot, const char model[]);
void pdRobotDestroy(pdRobot *robot);

/* methods to get parameters */
#define pdRobotChainPtr(r) ( &(r)->_chain )
#define pdRobotIKPtr(r)    ( &(r)->_ik )
#define pdRobotCellNum(r)  (r)->_cell_num

#define pdRobotFlagIsOn(r,id) ( (r)->_ref_set_flag[id] )
#define pdRobotCOMFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_COM )
#define pdRobotBaseAttFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_BASE_ATT )
#define pdRobotLFPosFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_LF_POS )
#define pdRobotLFAttFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_LF_ATT )
#define pdRobotRFPosFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_RF_POS )
#define pdRobotRFAttFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_RF_ATT )
#define pdRobotLHPosFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_LH_POS )
#define pdRobotLHAttFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_LH_ATT )
#define pdRobotRHPosFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_RH_POS )
#define pdRobotRHAttFlagIsOn(r) pdRobotFlagIsOn( r, PD_ROBOT_IKCELL_ID_RH_ATT )

#define pdRobotBaseID(r)   (r)->_base_id
#define pdRobotLFID(r)     (r)->_lf_id
#define pdRobotRFID(r)     (r)->_rf_id
#define pdRobotLHID(r)     (r)->_lh_id
#define pdRobotRHID(r)     (r)->_rh_id

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

/* methods to get parameters */
#define pdRobotJointSize(r) rkChainJointSize( pdRobotChainPtr(r) )
#define pdRobotJointDis(r)  (r)->dis
__EXPORT void pdRobotCOMPos(pdRobot *robot, zVec3D *com);
__EXPORT void pdRobotBaseAtt(pdRobot *robot, zVec3D *att);
__EXPORT void pdRobotFootPos(pdRobot *robot, zVec3D *lf, zVec3D *rf);
__EXPORT void pdRobotFootAtt(pdRobot *robot, zVec3D *lf, zVec3D *rf);
__EXPORT void pdRobotHandPos(pdRobot *robot, zVec3D *lh, zVec3D *rh);
__EXPORT void pdRobotHandAtt(pdRobot *robot, zVec3D *lh, zVec3D *rh);

__END_DECLS

#endif /* __PD_ROBOT_H__ */
