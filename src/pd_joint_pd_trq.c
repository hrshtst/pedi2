#include <pedi2/pd_joint.h>

typedef struct{
  double pgain, dgain;
  double trqmin, trqmax;
} _pdJointPDTrq;

void pdJointDestroyPDTrq(pdJoint *joint)
{
  zFree( joint->_prm );
  pdJointDestroyDefault( joint );
}

void pdJointUpdatePDTrq(pdJoint *joint, double dt)
{
  double x, v;
  _pdJointPDTrq *pd;

  x = pdJointDis(joint) - pdJointRefDis(joint);
  v = ( pdJointDis(joint) - pdJointDisOld(joint) ) / dt;
  pd = joint->_prm;
  joint->output = -pd->pgain * x - pd->dgain * v;
  pdJointSetVel( joint, v );
}

pdJointMethod pd_joint_pd_trq_met = {
  type: "PDtrq",
  setdis: pdJointSetDisDefault,
  setvel: pdJointSetVelDefault,
  setref: pdJointSetRefDefault,
  refresh: pdJointRefreshDefault,
  update: pdJointUpdatePDTrq,
  destroy: pdJointDestroyPDTrq,
};

bool pdJointCreatePDTrq(pdJoint *joint, double pgain, double dgain)
{
  _pdJointPDTrq *pd;

  if( !( pd = zAlloc( _pdJointPDTrq, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  pd->pgain = pgain;
  pd->dgain = dgain;
  pd->trqmin = -HUGE_VAL;
  pd->trqmax = HUGE_VAL;
  pdJointInit( joint );
  joint->_prm = pd;
  joint->_met = &pd_joint_pd_trq_met;
  return true;
}
