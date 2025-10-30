#include <pedi2/pd_joint.h>

void pdJointDefaultSetDis(pdJoint *joint, double dis)
{
  joint->disold = joint->dis;
  joint->dis = dis;
}

void pdJointDefaultSetVel(pdJoint *joint, double vel)
{
  joint->velold = joint->vel;
  joint->vel = vel;
  joint->is_set_vel = true;
}

void pdJointDefaultSetRefDis(pdJoint *joint, double refdis)
{
  joint->refdisold = joint->refdis;
  joint->refdis = refdis;
}

void pdJointDefaultSetRefVel(pdJoint *joint, double refvel)
{
  joint->refvelold = joint->refvel;
  joint->refvel = refvel;
  joint->is_set_refvel = true;
}

void pdJointDefaultRefresh(pdJoint *joint, double dis)
{
  joint->dis = dis;
  joint->disold = dis;
  joint->vel = 0;
  joint->velold = 0;
  joint->refdis = dis;
  joint->refdisold = dis;
  joint->refvel = 0;
  joint->refvelold = 0;
}

void pdJointDefaultUpdate(pdJoint *joint, double dt)
{
  double v, refv;

  if( !joint->is_set_vel ){
    v = ( pdJointDis(joint) - pdJointDisOld(joint) ) / dt;
    pdJointDefaultSetVel( joint, v );
  }
  if( !joint->is_set_refvel ){
    refv = ( pdJointRefDis(joint) - pdJointRefDisOld(joint) ) / dt;
    pdJointDefaultSetRefVel( joint, refv );
  }
  joint->is_set_vel = false;
  joint->is_set_refvel = false;
}

void pdJointDefaultDestroy(pdJoint *joint)
{
  zNameFree( joint );
  zFree( joint->prp );
  pdJointInit( joint );
}

static pdJoint *_pdJointAssignByStr(pdJoint *joint, const char *str)
{
  PD_JOINT_COM_ARRAY;
  register int i;

  for( i=0; pd_joint_com[i]; i++ ){
    if( strcmp( pd_joint_com[i]->typestr, str ) == 0 ){
      joint->com = pd_joint_com[i];
      return joint;
    }
  }
  ZRUNERROR( "cannot find a joint type %s", str );
  return NULL;
}

static void *_pdJointNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zNameSet( (pdJoint*)obj, ZTKVal(ztk) ) ? obj : NULL;
}
static void *_pdJointTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return _pdJointAssignByStr( (pdJoint*)obj, ZTKVal(ztk) ) ? obj : NULL;
}

static bool _pdJointNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((pdJoint*)obj) );
  return true;
}
static bool _pdJointTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", ((pdJoint*)obj)->com ? ((pdJoint*)obj)->com->typestr : "unknown" );
  return true;
}

static const ZTKPrp __ztk_prp_pdjoint[] = {
  { ZTK_KEY_PEDI2_JOINT_NAME, 1, _pdJointNameFromZTK, _pdJointNameFPrintZTK },
  { ZTK_KEY_PEDI2_JOINT_TYPE, 1, _pdJointTypeFromZTK, _pdJointTypeFPrintZTK },
};

pdJoint *pdJointFromZTK(pdJoint *joint, ZTK *ztk)
{
  char *name;
  if( !_ZTKEvalKey( joint, NULL, ztk, __ztk_prp_pdjoint ) ) return NULL;
  name = zNamePtr(joint);
  if( !joint->com || !joint->com->_fromZTK( joint, ztk ) ) return NULL;
  zNameSet( joint, name );
  return joint;
}

void pdJointFPrintZTK(FILE *fp, pdJoint *joint)
{
  _ZTKPrpKeyFPrint( fp, joint, __ztk_prp_pdjoint );
  if( joint->com )
    joint->com->_fprintZTK( fp, joint );
}

#define PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH \
  "Joint controller number is not matched with the vector size"

void pdJointArraySetDis(pdJointArray *arr, zVec q)
{
  register int i;

  if( (int)zArraySize(arr) != zVecSize(q) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetDis( zArrayElem( arr, i ), zVecElem( q, i) );
}

void pdJointArraySetVel(pdJointArray *arr, zVec v)
{
  register int i;

  if( (int)zArraySize(arr) != zVecSize(v) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetVel( zArrayElem( arr, i ), zVecElem( v, i) );
}

void pdJointArraySetRefDis(pdJointArray *arr, zVec q)
{
  register int i;

  if( (int)zArraySize(arr) != zVecSize(q) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetRefDis( zArrayElem( arr, i ), zVecElem( q, i) );
}

void pdJointArraySetRefVel(pdJointArray *arr, zVec v)
{
  register int i;

  if( (int)zArraySize(arr) != zVecSize(v) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetRefVel( zArrayElem( arr, i ), zVecElem( v, i) );
}

void pdJointArrayRefresh(pdJointArray *arr, zVec q)
{
  register int i;

  if( (int)zArraySize(arr) != zVecSize(q) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointRefresh( zArrayElem( arr, i ), zVecElem( q, i) );
}

void pdJointArrayUpdate(pdJointArray *arr, double dt)
{
  register int i;

  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointUpdate( zArrayElem(arr,i), dt );
}

void pdJointArrayDestroy(pdJointArray *arr)
{
  register uint i;

  for( i=0; i<zArraySize(arr); i++ )
    pdJointDestroy( zArrayElem(arr,i) );
  zArrayFree( arr );
}

pdJointArray *pdJointArrayAlloc(pdJointArray *arr, int size)
{
  register int i;

  zArrayAlloc( arr, pdJoint, size );
  if( !zArrayBuf(arr) || zArraySize(arr) != size ){
    ZALLOCERROR();
    return NULL;
  }
  for( i=0; i<size; i++ ){
    pdJointInit( zArrayElemNC(arr, i) );
    pdJointSetOffset( zArrayElem(arr,i), i );
  }
  return arr;
}

pdJoint *pdJointArrayNameFind(pdJointArray *arr, const char *name)
{
  pdJoint *joint = NULL;

  zArrayFindName( arr, name, joint );
  if( !joint ){
    ZRUNERROR( "invalid joint name %s", name );
    return NULL;
  }
  return joint;
}

static rkLink *_rkChainLinkFindName(rkChain *c, const char *name);
rkLink *_rkChainLinkFindName(rkChain *c, const char *name)
{
  rkLink *link;

  zNameFind( rkChainRoot(c), rkChainLinkNum(c), name, link );
  return link;
}

bool pdJointArraySetOffsetMapping(pdJointArray *arr, rkChain *c)
{
  register int i;
  rkLink *link;

  for( i=0; i<(int)zArraySize(arr); i++ ){
    link = _rkChainLinkFindName(c,zName(zArrayElem(arr,i)));
    if( !link ){
      ZRUNERROR( "joint %s cannot be found in robot model",
                 zName(zArrayElem(arr,i)) );
      return false;
    } else
      pdJointArraySetOffset( arr, i, rkLinkJointIDOffset(link) );
  }
  return true;
}

static void *_pdJointArrayJointFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return pdJointFromZTK( zArrayElemNC((pdJointArray*)obj,i), ztk ) ? obj : NULL;
}

static const ZTKPrp __ztk_prp_tag_pedi2_joint[] = {
  { ZTK_TAG_PEDI2_JOINT, -1, _pdJointArrayJointFromZTK, NULL },
};

static void _pdJointArrayInsertionSort(pdJointArray *arr, int (*cmp)(pdJoint*, pdJoint*))
{
  register int i, j;
  uint s;
  pdJoint *saved, *value;

  s = sizeof(pdJoint);
  saved = zAlloc( pdJoint, 1 );
  for( j=1; j<(int)zArraySize(arr); j++ ){
    i = j - 1;
    value = zArrayElem(arr, j);
    while( i >= 0 && cmp(zArrayElem(arr,i), value) > 0 ) i--;
    if( ++i == j ) continue;
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#elif defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wnonnull"
#endif
    memmove( saved, value, s );
    memmove( zArrayElem(arr,i+1), zArrayElem(arr,i), s*(j-i) );
    memmove( zArrayElem(arr,i), saved, s );
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(__clang__)
#pragma clang diagnostic pop
#endif
  }
  zFree( saved );
}

static int _pdJointOffsetCmp(pdJoint *p, pdJoint *q)
{
  if( pdJointOffset(p) == pdJointOffset(q) ) return 0;
  return ( pdJointOffset(p) > pdJointOffset(q) ) ? 1 : -1;
}

pdJointArray *pdJointArrayFromZTK(pdJointArray *arr, rkChain *c, ZTK *ztk)
{
  int num;
  bool result;

  if( ( num = ZTKCountTag( ztk, ZTK_TAG_PEDI2_JOINT ) ) == 0 ){
    ZRUNWARN( "empty array of joints specified." );
    return NULL;
  }
  if( !pdJointArrayAlloc( arr, num ) ) return NULL;
  _ZTKEvalTag( arr, NULL, ztk, __ztk_prp_tag_pedi2_joint );
  result = false;
  if( c )
    result = pdJointArraySetOffsetMapping( arr, c );
  if( result )
    _pdJointArrayInsertionSort( arr, _pdJointOffsetCmp );
  return arr;
}

void pdJointArrayFPrintZTK(FILE *fp, pdJointArray *arr)
{
  int i;

  for( i=0; i<zArraySize(arr); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_PEDI2_JOINT );
    pdJointFPrintZTK( fp, zArrayElemNC(arr,i) );
    fprintf( fp, "\n" );
  }
  _ZTKPrpTagFPrint( fp, arr, __ztk_prp_tag_pedi2_joint );
}

pdJointArray *pdJointArrayReadZTK(pdJointArray *arr, rkChain *c, char filename[])
{
  ZTK ztk;

  ZTKInit( &ztk );
  zArrayInit( arr );
  if( ZTKParse( &ztk, filename ) )
    arr = pdJointArrayFromZTK( arr, c, &ztk );
  ZTKDestroy( &ztk );
  return arr;
}

bool pdJointArrayWriteZTK(pdJointArray *arr, char filename[])
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( filename, "w" ) ) ) return false;
  pdJointArrayFPrintZTK( fp, arr );
  fclose(fp);
  return true;
}

zIndex pdJointArrayCreateIndex(pdJointArray *arr)
{
  register int i;
  zIndex index;

  if( !( index = zIndexCreate( zArraySize(arr) ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  for( i=0; i<(int)zArraySize(arr); i++ ){
    zIndexSetElem( index, i, pdJointArrayOffset( arr, i ) );
  }
  return index;
}

#define PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH \
  "Joint controller number is not matched with the index size"

void pdJointArraySetDisIndex(pdJointArray *arr, zIndex idx, zVec q)
{
  register int i;

  if( zArraySize(arr) != zArraySize(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetDis( zArrayElem(arr,i), zVecElem(q,zIndexElem(idx,i)) );
}

void pdJointArraySetVelIndex(pdJointArray *arr, zIndex idx, zVec v)
{
  register int i;

  if( zArraySize(arr) != zArraySize(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetVel( zArrayElem(arr,i), zVecElem(v,zIndexElem(idx,i)) );
}

void pdJointArraySetRefDisIndex(pdJointArray *arr, zIndex idx, zVec q)
{
  register int i;

  if( zArraySize(arr) != zArraySize(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetRefDis( zArrayElem(arr,i), zVecElem(q,zIndexElem(idx,i)) );
}

void pdJointArraySetRefVelIndex(pdJointArray *arr, zIndex idx, zVec v)
{
  register int i;

  if( zArraySize(arr) != zArraySize(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointSetRefVel( zArrayElem(arr,i), zVecElem(v,zIndexElem(idx,i)) );
}

void pdJointArrayRefreshIndex(pdJointArray *arr, zIndex idx, zVec q)
{
  register int i;

  if( zArraySize(arr) != zArraySize(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArraySize(arr); i++ )
    pdJointRefresh( zArrayElem(arr,i), zVecElem(q,zIndexElem(idx,i)) );
}
