#ifndef __PD_ROBOT_H__
#define __PD_ROBOT_H__

#include <roki/rk_chain.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_state.h>

__BEGIN_DECLS

typedef struct{
  rkChain _chain;         /* robot kinematics/dynamics model */

  int _torso_id;          /* identifier of torso link */
  int _lf_id, _rf_id;     /* identifier of foot link */
  int _lh_id, _rh_id;     /* identifier of hand link */

  zVec3DData _sr_lf_vert; /* set of vertices of left foot */
  zVec3DData _sr_rf_vert; /* set of vertices of right foot */
  zVec3DData _sr_vert;    /* set of vertices of both feet */

  zVec dis;               /* displacement vector */
  zVec disold;            /* old displacement vector */
} pdRobot;

#define PD_ROBOT_DEFAULT_PRIORITY_COM       3
#define PD_ROBOT_DEFAULT_PRIORITY_TORSO_ATT 2
#define PD_ROBOT_DEFAULT_PRIORITY_LF_POS    5
#define PD_ROBOT_DEFAULT_PRIORITY_LF_ATT    4
#define PD_ROBOT_DEFAULT_PRIORITY_RF_POS    5
#define PD_ROBOT_DEFAULT_PRIORITY_RF_ATT    4
#define PD_ROBOT_DEFAULT_PRIORITY_LH_POS    1
#define PD_ROBOT_DEFAULT_PRIORITY_LH_ATT    0
#define PD_ROBOT_DEFAULT_PRIORITY_RH_POS    1
#define PD_ROBOT_DEFAULT_PRIORITY_RH_ATT    0

#define PD_ROBOT_IKCELL_NAME_COM            "com"
#define PD_ROBOT_IKCELL_NAME_TORSO_ATT      "torso_att"
#define PD_ROBOT_IKCELL_NAME_LF_POS         "left_foot_pos"
#define PD_ROBOT_IKCELL_NAME_LF_ATT         "left_foot_att"
#define PD_ROBOT_IKCELL_NAME_RF_POS         "right_foot_pos"
#define PD_ROBOT_IKCELL_NAME_RF_ATT         "right_foot_att"
#define PD_ROBOT_IKCELL_NAME_LH_POS         "left_hand_pos"
#define PD_ROBOT_IKCELL_NAME_LH_ATT         "left_hand_att"
#define PD_ROBOT_IKCELL_NAME_RH_POS         "right_hand_pos"
#define PD_ROBOT_IKCELL_NAME_RH_ATT         "right_hand_att"

/* c'tor and d'tor */
__EXPORT void pdRobotInit(pdRobot *robot);
__EXPORT void pdRobotDestroy(pdRobot *robot);

/* model loader */
__EXPORT bool pdRobotLoad(pdRobot *robot, const char model[]);
__EXPORT bool pdRobotBindTorso(pdRobot *robot, const char torso[]);
__EXPORT bool pdRobotBindFeet(pdRobot *robot, const char left_foot[], const char right_foot[]);
__EXPORT bool pdRobotBindHands(pdRobot *robot, const char left_hand[], const char right_hand[]);
#define pdRobotBindTorsoLimb(robot,torso,lf,rf,lh,rh) do{\
  pdRobotBindTorso( robot, torso );\
  pdRobotBindFeet(  robot, lf, rf );\
  pdRobotBindHands( robot, lh, rh );\
} while(0)

/* methods to get parameters */
#define pdRobotChain(r)          ( &(r)->_chain )
#define pdRobotIKSolver(r)       pdRobotChain(r)->_ik
#define pdRobotIKCellList(r)     ( &pdRobotIKSolver(r)->cell_list )
#define pdRobotIKCellListSize(r) zListSize( pdRobotIKCellList(r) )

#define pdRobotTorsoID(r)  (r)->_torso_id
#define pdRobotLFID(r)     (r)->_lf_id
#define pdRobotRFID(r)     (r)->_rf_id
#define pdRobotLHID(r)     (r)->_lh_id
#define pdRobotRHID(r)     (r)->_rh_id

#define pdRobotSRLFVert(r) ( &(r)->_sr_lf_vert )
#define pdRobotSRRFVert(r) ( &(r)->_sr_rf_vert )
#define pdRobotSRVert(r)   ( &(r)->_sr_vert )

/* methods to find cells */
__EXPORT rkIKCell *pdRobotFindIKCellByName(pdRobot *robot, const char *name);
__EXPORT int pdRobotFindLinkIDByIKCellName(pdRobot *robot, const char *ikcell_names[]);

/* methods to get parameters */
#define pdRobotJointSize(r)  rkChainJointSize( pdRobotChain(r) )
#define pdRobotJointDis(r)   (r)->dis
#define pdRobotLinkNum(r)    rkChainLinkNum( pdRobotChain(r) )

/* methods to solve FK */
__EXPORT void pdRobotLinkJointSetDis(pdRobot *robot, int id, double *dis);
__EXPORT void pdRobotSetJointDis(pdRobot *robot, zIndex index, zVec dis);
#define pdRobotGetJointDisAll(r,v) zVecCopy( pdRobotJointDis(r), v )
__EXPORT void pdRobotGetJointDiffAll(pdRobot *robot, zVec v);
__EXPORT void pdRobotGetJointVelAll(pdRobot *robot, double dt, zVec v);

__EXPORT void pdRobotFK(pdRobot *robot, zVec dis);
__EXPORT void pdRobotFKIndex(pdRobot *robot, zIndex index, zVec dis);

__EXPORT void pdRobotResetJointDis(pdRobot *robot);

/* methods to solve IK */
__EXPORT bool pdRobotSetRef(pdRobot *robot, const char *ikcell_name, double v1, double v2, double v3);
__EXPORT bool pdRobotSetRefVec(pdRobot *robot, const char *ikcell_name, zVec3D *vec);
__EXPORT bool pdRobotSetRefAtt(pdRobot *robot, const char *ikcell_name, zMat3D *att);
#define pdRobotSetRefCOM(r,pos)      pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_COM, pos )
#define pdRobotSetRefTorsoZYX(r,zyx) pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_TORSO_ATT, zyx )
#define pdRobotSetRefTorsoAtt(r,att) pdRobotSetRefAtt( r, PD_ROBOT_IKCELL_NAME_TORSO_ATT, att )
#define pdRobotSetRefLFPos(r,pos)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_LF_POS, pos )
#define pdRobotSetRefLFZYX(r,zyx)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_LF_ATT, zyx )
#define pdRobotSetRefLFAtt(r,att)    pdRobotSetRefAtt( r, PD_ROBOT_IKCELL_NAME_LF_ATT, att )
#define pdRobotSetRefRFPos(r,pos)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_RF_POS, pos )
#define pdRobotSetRefRFZYX(r,zyx)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_RF_ATT, zyx )
#define pdRobotSetRefRFAtt(r,att)    pdRobotSetRefAtt( r, PD_ROBOT_IKCELL_NAME_RF_ATT, att )
#define pdRobotSetRefLHPos(r,pos)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_LH_POS, pos )
#define pdRobotSetRefLHZYX(r,zyx)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_LH_ATT, zyx )
#define pdRobotSetRefLHAtt(r,att)    pdRobotSetRefAtt( r, PD_ROBOT_IKCELL_NAME_LH_ATT, att )
#define pdRobotSetRefRHPos(r,pos)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_RH_POS, pos )
#define pdRobotSetRefRHZYX(r,zyx)    pdRobotSetRefVec( r, PD_ROBOT_IKCELL_NAME_RH_ATT, zyx )
#define pdRobotSetRefRHAtt(r,att)    pdRobotSetRefAtt( r, PD_ROBOT_IKCELL_NAME_RH_ATT, att )

#define pdRobotRegisterIKJointID(r,i,w)    rkChainRegisterIKJointID( pdRobotChain(r), i, w )
#define pdRobotRegisterIKJointAll(r,w)     rkChainRegisterIKJointAll( pdRobotChain(r), w )
#define pdRobotUnregisterIKJointID(r,i)    rkChainUnregisterIKJointID( pdRobotChain(r), i )
#define pdRobotRegisterIKJoint(r,name,w)   rkChainRegisterIKJoint( pdRobotChain(r), name, w )
#define pdRobotUnregisterIKJoint(r,name)   rkChainUnregisterIKJoint( pdRobotChain(r), name )

__EXPORT void pdRobotSolveIK(pdRobot *robot, int iter);

__EXPORT zVec3D *pdRobotGetRefPos(pdRobot *robot, const char *ikcell_name, zVec3D *pos);
__EXPORT zVec3D *pdRobotGetRefZYX(pdRobot *robot, const char *ikcell_name, zVec3D *zyx);
__EXPORT zMat3D *pdRobotGetRefAtt(pdRobot *robot, const char *ikcell_name, zMat3D *att);
#define pdRobotGetRefCOM(r,pos)      pdRobotGetRefPos( r, PD_ROBOT_IKCELL_NAME_COM, pos )
#define pdRobotGetRefTorsoZYX(r,zyx) pdRobotGetRefZYX( r, PD_ROBOT_IKCELL_NAME_TORSO_ATT, zyx )
#define pdRobotGetRefTorsoAtt(r,att) pdRobotGetRefAtt( r, PD_ROBOT_IKCELL_NAME_TORSO_ATT, att )
#define pdRobotGetRefLFPos(r,pos)    pdRobotGetRefPos( r, PD_ROBOT_IKCELL_NAME_LF_POS, pos )
#define pdRobotGetRefLFZYX(r,zyx)    pdRobotGetRefZYX( r, PD_ROBOT_IKCELL_NAME_LF_ATT, zyx )
#define pdRobotGetRefLFAtt(r,att)    pdRobotGetRefAtt( r, PD_ROBOT_IKCELL_NAME_LF_ATT, att )
#define pdRobotGetRefRFPos(r,pos)    pdRobotGetRefPos( r, PD_ROBOT_IKCELL_NAME_RF_POS, pos )
#define pdRobotGetRefRFZYX(r,zyx)    pdRobotGetRefZYX( r, PD_ROBOT_IKCELL_NAME_RF_ATT, zyx )
#define pdRobotGetRefRFAtt(r,att)    pdRobotGetRefAtt( r, PD_ROBOT_IKCELL_NAME_RF_ATT, att )
#define pdRobotGetRefLHPos(r,pos)    pdRobotGetRefPos( r, PD_ROBOT_IKCELL_NAME_LH_POS, pos )
#define pdRobotGetRefLHZYX(r,zyx)    pdRobotGetRefZYX( r, PD_ROBOT_IKCELL_NAME_LH_ATT, zyx )
#define pdRobotGetRefLHAtt(r,att)    pdRobotGetRefAtt( r, PD_ROBOT_IKCELL_NAME_LH_ATT, att )
#define pdRobotGetRefRHPos(r,pos)    pdRobotGetRefPos( r, PD_ROBOT_IKCELL_NAME_RH_POS, pos )
#define pdRobotGetRefRHZYX(r,zyx)    pdRobotGetRefZYX( r, PD_ROBOT_IKCELL_NAME_RH_ATT, zyx )
#define pdRobotGetRefRHAtt(r,att)    pdRobotGetRefAtt( r, PD_ROBOT_IKCELL_NAME_RH_ATT, att )
__EXPORT void pdRobotCOMPos(pdRobot *robot, zVec3D *com);
__EXPORT void pdRobotTorsoZYX(pdRobot *robot, zVec3D *zyx);
__EXPORT void pdRobotTorsoAtt(pdRobot *robot, zMat3D *att);
__EXPORT void pdRobotFootPos(pdRobot *robot, zVec3D *lf, zVec3D *rf);
__EXPORT void pdRobotFootZYX(pdRobot *robot, zVec3D *lf, zVec3D *rf);
__EXPORT void pdRobotFootAtt(pdRobot *robot, zMat3D *lf, zMat3D *rf);
__EXPORT void pdRobotHandPos(pdRobot *robot, zVec3D *lh, zVec3D *rh);
__EXPORT void pdRobotHandZYX(pdRobot *robot, zVec3D *lh, zVec3D *rh);
__EXPORT void pdRobotHandAtt(pdRobot *robot, zMat3D *lh, zMat3D *rh);

/* methods regarding biped robot */
__EXPORT void pdRobotBipedDefaultInit(pdRobot *robot, pdBiped *biped, pdState *state);
__EXPORT void pdRobotBipedResetPose(pdRobot *robot, pdBiped *biped, pdState *state, zVec dis);
__EXPORT void pdRobotBipedSetRefVec(pdRobot *robot, pdBiped *biped);
__EXPORT bool pdRobotSupportRegion(pdRobot *robot, zLoop3D *sr_lf, zLoop3D *sr_rf, zLoop3D *sr);
__EXPORT void pdRobotUpdateState(pdRobot *robot, pdState *state);

/* output method */
__EXPORT void pdRobotFPrint(FILE *fp, pdRobot *robot);
#define pdRobotPrint(r) pdRobotFPrint( stdout, r )

__END_DECLS

#endif /* __PD_ROBOT_H__ */
