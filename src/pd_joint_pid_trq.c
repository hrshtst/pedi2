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

  pdJointUpdateDefault( joint, dt );
  x = pdJointRefDis(joint) - pdJointDis(joint);
  v = pdJointRefVel(joint) - pdJointVel(joint);
  pid = joint->_prm;
  pid->err += x * dt;
  joint->output = pid->pgain * x + pid->dgain * v + pid->igain * pid->err;
  joint->output = zLimit( joint->output, pid->trqmin, pid->trqmax );
}

typedef struct{
  double pgain, igain, dgain;
  double trqmin, trqmax;
} _pdJointPIDTrqParam;

static bool _pdJointFReadPIDTrq(FILE *fp, void *prm, char *buf, bool *success);

bool _pdJointFReadPIDTrq(FILE *fp, void *prm, char *buf, bool *success)
{
  if( strcmp( buf, "pgain" ) == 0 ){
    ((_pdJointPIDTrqParam *)prm)->pgain = zFDouble( fp );
  } else
  if( strcmp( buf, "igain" ) == 0 ){
    ((_pdJointPIDTrqParam *)prm)->igain = zFDouble( fp );
  } else
  if( strcmp( buf, "dgain" ) == 0 ){
    ((_pdJointPIDTrqParam *)prm)->dgain = zFDouble( fp );
  } else
  if( strcmp( buf, "min" ) == 0 ){
    ((_pdJointPIDTrqParam *)prm)->trqmin = zFDouble( fp );
  } else
  if( strcmp( buf, "max" ) == 0 ){
    ((_pdJointPIDTrqParam *)prm)->trqmax = zFDouble( fp );
  } else
    return false;
  return true;
}

pdJoint *pdJointFReadPIDTrq(FILE *fp, pdJoint *joint)
{
  _pdJointPIDTrq prm = { 0, 0, 0, -HUGE_VAL, HUGE_VAL, 0 };

  zFieldFRead( fp, _pdJointFReadPIDTrq, &prm );
  if( !pdJointCreatePIDTrq( joint, prm.pgain, prm.igain, prm.dgain ) )
    return NULL;
  pdJointPIDTrqSetLim( joint, prm.trqmin, prm.trqmax );
  return joint;
}

pdJointMethod pd_joint_pid_trq_met = {
  type: "PIDtrq",
  setdis: pdJointSetDisDefault,
  setvel: pdJointSetVelDefault,
  setrefdis: pdJointSetRefDisDefault,
  setrefvel: pdJointSetRefVelDefault,
  refresh: pdJointRefreshDefault,
  update: pdJointUpdatePIDTrq,
  destroy: pdJointDestroyPIDTrq,
  fread: pdJointFReadPIDTrq,
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
