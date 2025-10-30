#ifndef __PD_JOINT_PD_TRQ_H__
#define __PD_JOINT_PD_TRQ_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__PEDI2_EXPORT pdJoint *pdJointPDTrqCreate(pdJoint *joint, double pgain, double dgain);

__PEDI2_EXPORT pdJoint *pdJointPDTrqSetLim(pdJoint *joint, double min, double max);
__PEDI2_EXPORT pdJoint *pdJointPDTrqSetPgain(pdJoint *joint, double pgain);
__PEDI2_EXPORT pdJoint *pdJointPDTrqSetDgain(pdJoint *joint, double dgain);

extern pdJointCom pd_joint_pd_trq_com;

__END_DECLS

#endif /* __PD_JOINT_PD_TRQ_H__ */
