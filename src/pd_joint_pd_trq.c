#include <pedi2/pd_joint.h>

typedef struct{
  double pgain, dgain;
  double trqmin, trqmax;
} _pdJointPDTrqPrp;

static void _pdJointPDTrqDestroy(pdJoint *joint)
{
  zFree( joint->prp );
  pdJointDefaultDestroy( joint );
}

static void _pdJointPDTrqUpdate(pdJoint *joint, double dt)
{
  double x, v;
  _pdJointPDTrqPrp *prp;

  pdJointDefaultUpdate( joint, dt );
  x = pdJointRefDis(joint) - pdJointDis(joint);
  v = pdJointRefVel(joint) - pdJointVel(joint);
  prp = joint->prp;
  joint->output = prp->pgain * x + prp->dgain * v;
  joint->output = zLimit( joint->output, prp->trqmin, prp->trqmax );
}

static void *_pdJointPDTrqPGainFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPDTrqPrp*)val)->pgain = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPDTrqDGainFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPDTrqPrp*)val)->dgain = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPDTrqMinFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPDTrqPrp*)val)->trqmin = ZTKDouble(ztk);
  return val;
}
static void *_pdJointPDTrqMaxFromZTK(void *val, int i, void *arg, ZTK *ztk){
  ((_pdJointPDTrqPrp*)val)->trqmax = ZTKDouble(ztk);
  return val;
}

static bool _pdJointPDTrqPGainFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", ((_pdJointPDTrqPrp*)((pdJoint*)prp)->prp)->pgain );
  return true;
}
static bool _pdJointPDTrqDGainFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", ((_pdJointPDTrqPrp*)((pdJoint*)prp)->prp)->dgain );
  return true;
}
static bool _pdJointPDTrqMinFPrintZTK(FILE *fp, int i, void *prp){
  double trqmin;
  trqmin = ((_pdJointPDTrqPrp*)((pdJoint*)prp)->prp)->trqmin;
  if( zIsInf( -trqmin ) ) return false;
  fprintf( fp, "%.10g\n", trqmin );
  return true;
}
static bool _pdJointPDTrqMaxFPrintZTK(FILE *fp, int i, void *prp){
  double trqmax;
  trqmax = ((_pdJointPDTrqPrp*)((pdJoint*)prp)->prp)->trqmax;
  if( zIsInf( trqmax ) ) return false;
  fprintf( fp, "%.10g\n", trqmax );
  return true;
}

static const ZTKPrp __ztk_prp_pdjoint_pd_trq[] = {
  { ZTK_KEY_PEDI2_JOINT_PGAIN, 1, _pdJointPDTrqPGainFromZTK, _pdJointPDTrqPGainFPrintZTK },
  { ZTK_KEY_PEDI2_JOINT_DGAIN, 1, _pdJointPDTrqDGainFromZTK, _pdJointPDTrqDGainFPrintZTK },
  { ZTK_KEY_PEDI2_JOINT_MIN,   1, _pdJointPDTrqMinFromZTK,   _pdJointPDTrqMinFPrintZTK   },
  { ZTK_KEY_PEDI2_JOINT_MAX,   1, _pdJointPDTrqMaxFromZTK,   _pdJointPDTrqMaxFPrintZTK   },
};

static pdJoint *_pdJointPDTrqFromZTK(pdJoint *joint, ZTK *ztk)
{
  _pdJointPDTrqPrp prp = { 0.0, 0.0, -HUGE_VAL, HUGE_VAL };
  if( !_ZTKEvalKey( &prp, NULL, ztk, __ztk_prp_pdjoint_pd_trq ) ) return NULL;
  if( !pdJointPDTrqCreate( joint, prp.pgain, prp.dgain ) ) return NULL;
  return pdJointPDTrqSetLim( joint, prp.trqmin, prp.trqmax );
}

static void _pdJointPDTrqFPrintZTK(FILE *fp, pdJoint *joint)
{
  _ZTKPrpKeyFPrint( fp, joint, __ztk_prp_pdjoint_pd_trq );
}

pdJointCom pd_joint_pd_trq_com = {
  .typestr = "PDtrq",
  ._setdis = pdJointDefaultSetDis,
  ._setvel = pdJointDefaultSetVel,
  ._setrefdis = pdJointDefaultSetRefDis,
  ._setrefvel = pdJointDefaultSetRefVel,
  ._refresh = pdJointDefaultRefresh,
  ._update = _pdJointPDTrqUpdate,
  ._destroy = _pdJointPDTrqDestroy,
  ._fromZTK = _pdJointPDTrqFromZTK,
  ._fprintZTK = _pdJointPDTrqFPrintZTK,
};

pdJoint *pdJointPDTrqCreate(pdJoint *joint, double pgain, double dgain)
{
  _pdJointPDTrqPrp *prp;

  if( !( prp = zAlloc( _pdJointPDTrqPrp, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  prp->pgain = pgain;
  prp->dgain = dgain;
  prp->trqmin = -HUGE_VAL;
  prp->trqmax = HUGE_VAL;
  pdJointInit( joint );
  joint->prp = prp;
  joint->com = &pd_joint_pd_trq_com;
  return joint;
}

pdJoint *pdJointPDTrqSetLim(pdJoint *joint, double min, double max)
{
  _pdJointPDTrqPrp *prp;

  prp = joint->prp;
  prp->trqmin = zMin( min, max );
  prp->trqmax = zMax( min, max );
  return joint;
}

pdJoint *pdJointPDTrqSetPgain(pdJoint *joint, double pgain)
{
  _pdJointPDTrqPrp *prp;

  prp = joint->prp;
  prp->pgain = pgain;
  return joint;
}

pdJoint *pdJointPDTrqSetDgain(pdJoint *joint, double dgain)
{
  _pdJointPDTrqPrp *prp;

  prp = joint->prp;
  prp->dgain = dgain;
  return joint;
}
