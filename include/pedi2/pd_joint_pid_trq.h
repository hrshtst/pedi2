#ifndef __PD_JOINT_PID_TRQ_H__
#define __PD_JOINT_PID_TRQ_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT pdJoint *pdJointPIDTrqCreate(pdJoint *joint, double pgain, double igain, double dgain);

__EXPORT pdJoint *pdJointPIDTrqSetLim(pdJoint *joint, double min, double max);
__EXPORT pdJoint *pdJointPIDTrqSetPgain(pdJoint *joint, double pgain);
__EXPORT pdJoint *pdJointPIDTrqSetIgain(pdJoint *joint, double igain);
__EXPORT pdJoint *pdJointPIDTrqSetDgain(pdJoint *joint, double dgain);

extern pdJointCom pd_joint_pid_trq_com;

__END_DECLS

#endif /* __PD_JOINT_PID_TRQ_H__ */
