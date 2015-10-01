#ifndef __PD_JOINT_PD_TRQ_H__
#define __PD_JOINT_PD_TRQ_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT bool pdJointCreatePDTrq(pdJoint *joint, double pgain, double dgain);

extern pdJointMethod pd_joint_pd_trq_met;

__END_DECLS

#endif /* __PD_JOINT_PD_TRQ_H__ */
