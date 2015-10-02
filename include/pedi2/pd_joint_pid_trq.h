#ifndef __PD_JOINT_PID_TRQ_H__
#define __PD_JOINT_PID_TRQ_H__

/* NONE: never include this header file in user programs. */

__BEGIN_DECLS

__EXPORT bool pdJointCreatePIDTrq(pdJoint *joint, double pgain, double igain, double dgain);

__EXPORT void pdJointPIDTrqSetLim(pdJoint *joint, double min, double max);
__EXPORT void pdJointPIDTrqSetPgain(pdJoint *joint, double pgain);
__EXPORT void pdJointPIDTrqSetIgain(pdJoint *joint, double igain);
__EXPORT void pdJointPIDTrqSetDgain(pdJoint *joint, double dgain);

extern pdJointMethod pd_joint_pid_trq_met;

__END_DECLS

#endif /* __PD_JOINT_PID_TRQ_H__ */
