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

  pdJointUpdateDefault( joint, dt );
  x = pdJointRefDis(joint) - pdJointDis(joint);
  v = pdJointRefVel(joint) - pdJointVel(joint);
  pd = joint->_prm;
  joint->output = pd->pgain * x + pd->dgain * v;
  joint->output = zLimit( joint->output, pd->trqmin, pd->trqmax );
}

typedef struct{
  double pgain, dgain;
  double trqmin, trqmax;
} _pdJointPDTrqParam;

static bool _pdJointFReadPDTrq(FILE *fp, void *prm, char *buf, bool *success);

bool _pdJointFReadPDTrq(FILE *fp, void *prm, char *buf, bool *success)
{
  if( strcmp( buf, "pgain" ) == 0 ){
    ((_pdJointPDTrqParam *)prm)->pgain = zFDouble( fp );
  } else
  if( strcmp( buf, "dgain" ) == 0 ){
    ((_pdJointPDTrqParam *)prm)->dgain = zFDouble( fp );
  } else
  if( strcmp( buf, "min" ) == 0 ){
    ((_pdJointPDTrqParam *)prm)->trqmin = zFDouble( fp );
  } else
  if( strcmp( buf, "max" ) == 0 ){
    ((_pdJointPDTrqParam *)prm)->trqmax = zFDouble( fp );
  } else
    return false;
  return true;
}

pdJoint *pdJointFReadPDTrq(FILE *fp, pdJoint *joint)
{
  _pdJointPDTrq prm = { 0, 0, -HUGE_VAL, HUGE_VAL };

  zFieldFRead( fp, _pdJointFReadPDTrq, &prm );
  if( !pdJointCreatePDTrq( joint, prm.pgain, prm.dgain ) )
    return NULL;
  pdJointPDTrqSetLim( joint, prm.trqmin, prm.trqmax );
  return joint;
}

pdJointMethod pd_joint_pd_trq_met = {
  "PDtrq",
  pdJointSetDisDefault,
  pdJointSetVelDefault,
  pdJointSetRefDisDefault,
  pdJointSetRefVelDefault,
  pdJointRefreshDefault,
  pdJointUpdatePDTrq,
  pdJointDestroyPDTrq,
  pdJointFReadPDTrq,
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

void pdJointPDTrqSetLim(pdJoint *joint, double min, double max)
{
  _pdJointPDTrq *pd;

  pd = joint->_prm;
  pd->trqmin = zMin( min, max );
  pd->trqmax = zMax( min, max );
}

void pdJointPDTrqSetPgain(pdJoint *joint, double pgain)
{
  _pdJointPDTrq *pd;

  pd = joint->_prm;
  pd->pgain = pgain;
}

void pdJointPDTrqSetDgain(pdJoint *joint, double dgain)
{
  _pdJointPDTrq *pd;

  pd = joint->_prm;
  pd->dgain = dgain;
}
