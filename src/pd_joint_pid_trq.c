#include <pedi2/pd_joint.h>

typedef struct{
  double pgain, igain, dgain;
  double trqmin, trqmax;
  double err;
} _pdJointPIDTrq;

void pdJointDestroyPIDTrq(pdJoint *joint)
{
  zFree( joint->_prm );
  pdJointDestroyDefault( joint );
}

void pdJointUpdatePIDTrq(pdJoint *joint, double dt)
{
  double x, v;
  _pdJointPIDTrq *pid;

  x = pdJointDis(joint) - pdJointRefDis(joint);
  v = ( pdJointDis(joint) - pdJointDisOld(joint) ) / dt;
  pid = joint->_prm;
  pid->err += x * dt;
  joint->output = -pid->pgain * x - pid->dgain * v - pid->igain * pid->err;
  joint->output = zLimit( joint->output, pid->trqmin, pid->trqmax );
  pdJointSetVel( joint, v );
}

pdJointMethod pd_joint_pid_trq_met = {
  type: "PIDtrq",
  setdis: pdJointSetDisDefault,
  setvel: pdJointSetVelDefault,
  setref: pdJointSetRefDefault,
  refresh: pdJointRefreshDefault,
  update: pdJointUpdatePIDTrq,
  destroy: pdJointDestroyPIDTrq,
};

bool pdJointCreatePIDTrq(pdJoint *joint, double pgain, double igain, double dgain)
{
  _pdJointPIDTrq *pid;

  if( !( pid = zAlloc( _pdJointPIDTrq, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  pid->pgain = pgain;
  pid->igain = igain;
  pid->dgain = dgain;
  pid->trqmin = -HUGE_VAL;
  pid->trqmax = HUGE_VAL;
  pid->err = 0;
  pdJointInit( joint );
  joint->_prm = pid;
  joint->_met = &pd_joint_pid_trq_met;
  return true;
}

void pdJointPIDTrqSetLim(pdJoint *joint, double min, double max)
{
  _pdJointPIDTrq *pid;

  pid = joint->_prm;
  pid->trqmin = zMin( min, max );
  pid->trqmax = zMax( min, max );
}

void pdJointPIDTrqSetPgain(pdJoint *joint, double pgain)
{
  _pdJointPIDTrq *pid;

  pid = joint->_prm;
  pid->pgain = pgain;
}

void pdJointPIDTrqSetIgain(pdJoint *joint, double igain)
{
  _pdJointPIDTrq *pid;

  pid = joint->_prm;
  pid->igain = igain;
}

void pdJointPIDTrqSetDgain(pdJoint *joint, double dgain)
{
  _pdJointPIDTrq *pid;

  pid = joint->_prm;
  pid->dgain = dgain;
}
