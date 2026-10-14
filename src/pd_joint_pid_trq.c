#include <pedi2/pd_joint.h>

typedef struct{
  double pgain, igain, dgain;
  double trqmin, trqmax;
  double err;
} _pdJointPIDTrqPrp;

static void _pdJointPIDTrqDestroy(pdJoint *joint)
{
  zFree( joint->prp );
  pdJointDefaultDestroy( joint );
}

static void _pdJointPIDTrqUpdate(pdJoint *joint, double dt)
{
  double x, v;
  _pdJointPIDTrqPrp *prp;

  pdJointDefaultUpdate( joint, dt );
  x = pdJointRefDis(joint) - pdJointDis(joint);
  v = pdJointRefVel(joint) - pdJointVel(joint);
  prp = joint->prp;
  prp->err += x * dt;
  joint->output = prp->pgain * x + prp->dgain * v + prp->igain * prp->err;
  joint->output = zLimit( joint->output, prp->trqmin, prp->trqmax );
}

static void *_pdJointPIDTrqPGainFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPIDTrqPrp*)val)->pgain = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPIDTrqIGainFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPIDTrqPrp*)val)->igain = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPIDTrqDGainFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPIDTrqPrp*)val)->dgain = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPIDTrqMinFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPIDTrqPrp*)val)->trqmin = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPIDTrqMaxFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPIDTrqPrp*)val)->trqmax = ZTKDouble(ztk);
  return val;
}

static bool _pdJointPIDTrqPGainFPrintZTK(FILE *fp, int i, const void *prp){
  fprintf( fp, "%.10g\n", ((_pdJointPIDTrqPrp*)((pdJoint*)prp)->prp)->pgain );
  return true;
}
static bool _pdJointPIDTrqIGainFPrintZTK(FILE *fp, int i, const void *prp){
  fprintf( fp, "%.10g\n", ((_pdJointPIDTrqPrp*)((pdJoint*)prp)->prp)->igain );
  return true;
}
static bool _pdJointPIDTrqDGainFPrintZTK(FILE *fp, int i, const void *prp){
  fprintf( fp, "%.10g\n", ((_pdJointPIDTrqPrp*)((pdJoint*)prp)->prp)->dgain );
  return true;
}
static bool _pdJointPIDTrqMinFPrintZTK(FILE *fp, int i, const void *prp){
  double trqmin;
  trqmin = ((_pdJointPIDTrqPrp*)((pdJoint*)prp)->prp)->trqmin;
  if( zIsInf( -trqmin ) ) return false;
  fprintf( fp, "%.10g\n", trqmin );
  return true;
}
static bool _pdJointPIDTrqMaxFPrintZTK(FILE *fp, int i, const void *prp){
  double trqmax;
  trqmax = ((_pdJointPIDTrqPrp*)((pdJoint*)prp)->prp)->trqmax;
  if( zIsInf( trqmax ) ) return false;
  fprintf( fp, "%.10g\n", trqmax );
  return true;
}

static const ZTKPrp __ztk_prp_pdjoint_pid_trq[] = {
  { ZTK_KEY_PEDI2_JOINT_PGAIN, 1, _pdJointPIDTrqPGainFromZTK, _pdJointPIDTrqPGainFPrintZTK },
  { ZTK_KEY_PEDI2_JOINT_IGAIN, 1, _pdJointPIDTrqIGainFromZTK, _pdJointPIDTrqIGainFPrintZTK },
  { ZTK_KEY_PEDI2_JOINT_DGAIN, 1, _pdJointPIDTrqDGainFromZTK, _pdJointPIDTrqDGainFPrintZTK },
  { ZTK_KEY_PEDI2_JOINT_MIN,   1, _pdJointPIDTrqMinFromZTK,   _pdJointPIDTrqMinFPrintZTK   },
  { ZTK_KEY_PEDI2_JOINT_MAX,   1, _pdJointPIDTrqMaxFromZTK,   _pdJointPIDTrqMaxFPrintZTK   },
};

static pdJoint *_pdJointPIDTrqFromZTK(pdJoint *joint, ZTK *ztk)
{
  _pdJointPIDTrqPrp prp = { 0, 0, 0, -HUGE_VAL, HUGE_VAL, 0 };
  if( !_ZTKEvalKey( &prp, NULL, ztk, __ztk_prp_pdjoint_pid_trq ) ) return NULL;
  if( !pdJointPIDTrqCreate( joint, prp.pgain, prp.igain, prp.dgain ) ) return NULL;
  return pdJointPIDTrqSetLim( joint, prp.trqmin, prp.trqmax );
}

static void _pdJointPIDTrqFPrintZTK(FILE *fp, pdJoint *joint)
{
  _ZTKPrpKeyFPrint( fp, joint, __ztk_prp_pdjoint_pid_trq );
}

pdJointCom pd_joint_pid_trq_com = {
  .typestr = "PIDtrq",
  ._setdis = pdJointDefaultSetDis,
  ._setvel = pdJointDefaultSetVel,
  ._setrefdis = pdJointDefaultSetRefDis,
  ._setrefvel = pdJointDefaultSetRefVel,
  ._refresh = pdJointDefaultRefresh,
  ._update = _pdJointPIDTrqUpdate,
  ._destroy = _pdJointPIDTrqDestroy,
  ._fromZTK = _pdJointPIDTrqFromZTK,
  ._fprintZTK = _pdJointPIDTrqFPrintZTK,
};

pdJoint *pdJointPIDTrqCreate(pdJoint *joint, double pgain, double igain, double dgain)
{
  _pdJointPIDTrqPrp *prp;

  if( !( prp = zAlloc( _pdJointPIDTrqPrp, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  prp->pgain = pgain;
  prp->igain = igain;
  prp->dgain = dgain;
  prp->trqmin = -HUGE_VAL;
  prp->trqmax = HUGE_VAL;
  prp->err = 0;
  pdJointInit( joint );
  joint->prp = prp;
  joint->com = &pd_joint_pid_trq_com;
  return joint;
}

pdJoint *pdJointPIDTrqSetLim(pdJoint *joint, double min, double max)
{
  _pdJointPIDTrqPrp *prp;

  prp = joint->prp;
  prp->trqmin = zMin( min, max );
  prp->trqmax = zMax( min, max );
  return joint;
}

pdJoint *pdJointPIDTrqSetPgain(pdJoint *joint, double pgain)
{
  _pdJointPIDTrqPrp *prp;

  prp = joint->prp;
  prp->pgain = pgain;
  return joint;
}

pdJoint *pdJointPIDTrqSetIgain(pdJoint *joint, double igain)
{
  _pdJointPIDTrqPrp *prp;

  prp = joint->prp;
  prp->igain = igain;
  return joint;
}

pdJoint *pdJointPIDTrqSetDgain(pdJoint *joint, double dgain)
{
  _pdJointPIDTrqPrp *prp;

  prp = joint->prp;
  prp->dgain = dgain;
  return joint;
}
