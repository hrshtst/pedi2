#include <pedi2/pd_joint.h>

void pdJointSetDisDefault(pdJoint *joint, double dis)
{
  joint->disold = joint->dis;
  joint->dis = dis;
}

void pdJointSetVelDefault(pdJoint *joint, double vel)
{
  joint->velold = joint->vel;
  joint->vel = vel;
  joint->is_set_vel = true;
}

void pdJointSetRefDisDefault(pdJoint *joint, double refdis)
{
  joint->refdisold = joint->refdis;
  joint->refdis = refdis;
}

void pdJointSetRefVelDefault(pdJoint *joint, double refvel)
{
  joint->refvelold = joint->refvel;
  joint->refvel = refvel;
  joint->is_set_refvel = true;
}

void pdJointRefreshDefault(pdJoint *joint, double dis)
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

void pdJointUpdateDefault(pdJoint *joint, double dt)
{
  double v, refv;

  if( !joint->is_set_vel ){
    v = ( pdJointDis(joint) - pdJointDisOld(joint) ) / dt;
    pdJointSetVelDefault( joint, v );
  }
  if( !joint->is_set_refvel ){
    refv = ( pdJointRefDis(joint) - pdJointRefDisOld(joint) ) / dt;
    pdJointSetRefVelDefault( joint, refv );
  }
  joint->is_set_vel = false;
  joint->is_set_refvel = false;
}

void pdJointDestroyDefault(pdJoint *joint)
{
  zNameDestroy( joint );
  zFree( joint->_prm );
  pdJointInit( joint );
}

static pdJointMethod *_pdJointMethodByStr(char str[]);

pdJointMethod *_pdJointMethodByStr(char str[])
{
  static pdJointMethod *met_array[] = {
    &pd_joint_pd_trq_met, &pd_joint_pid_trq_met,
    NULL,
  };
  register int i;

  for( i=0; met_array[i]; i++ ){
    if( strcmp( met_array[i]->type, str ) == 0 ) return met_array[i];
  }
  ZRUNERROR( "cannot find a joint type %s", str );
  return NULL;
}

typedef struct{
  pdJointMethod *met;
  char name[BUFSIZ];
} _pdJointParam;

bool _pdJointFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "type" ) == 0 ){
    if( !( ((_pdJointParam *)instance)->met = _pdJointMethodByStr( zFToken(fp,buf,BUFSIZ) ) ) )
      *success = false;
  } else
  if( strcmp( buf, "name" ) == 0 ){
    if( !zFToken( fp, ((_pdJointParam *)instance)->name, BUFSIZ ) )
      *success = false;
  } else
    return false;
  return true;
}

pdJoint *pdJointFRead(FILE *fp, pdJoint *joint)
{
  _pdJointParam prm;
  int cur;

  prm.met = NULL;
  prm.name[0] = '\0';
  cur = ftell( fp );
  zFieldFRead( fp, _pdJointFRead, &prm );
  if( !prm.met ){
    ZRUNERROR( "type not specified" );
    return NULL;
  }
  fseek( fp, cur, SEEK_SET );
  if( prm.met->fread( fp, joint ) ){
    if( !zNameSet( joint, prm.name ) ){
      ZALLOCERROR();
      return NULL;
    }
    return joint;
  }
  return NULL;
}

#define PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH \
  "Joint controller number is not matched with the vecotr size"

void pdJointArraySetDis(pdJointArray *arr, zVec q)
{
  register int i;

  if( (int)zArrayNum(arr) != zVecSize(q) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetDis( zArrayElem( arr, i ), zVecElem( q, i) );
}

void pdJointArraySetVel(pdJointArray *arr, zVec v)
{
  register int i;

  if( (int)zArrayNum(arr) != zVecSize(v) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetVel( zArrayElem( arr, i ), zVecElem( v, i) );
}

void pdJointArraySetRefDis(pdJointArray *arr, zVec q)
{
  register int i;

  if( (int)zArrayNum(arr) != zVecSize(q) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetRefDis( zArrayElem( arr, i ), zVecElem( q, i) );
}

void pdJointArraySetRefVel(pdJointArray *arr, zVec v)
{
  register int i;

  if( (int)zArrayNum(arr) != zVecSize(v) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetRefVel( zArrayElem( arr, i ), zVecElem( v, i) );
}

void pdJointArrayRefresh(pdJointArray *arr, zVec q)
{
  register int i;

  if( (int)zArrayNum(arr) != zVecSize(q) )
    ZRUNERROR( PD_JOINT_ERR_MSG_VEC_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointRefresh( zArrayElem( arr, i ), zVecElem( q, i) );
}

void pdJointArrayUpdate(pdJointArray *arr, double dt)
{
  register int i;

  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointUpdate( zArrayElem(arr,i), dt );
}

void pdJointArrayDestroy(pdJointArray *arr)
{
  register uint i;

  for( i=0; i<zArrayNum(arr); i++ )
    pdJointDestroy( zArrayElem(arr,i) );
  zArrayFree( arr );
}

bool pdJointArrayAlloc(pdJointArray *arr, int n)
{
  register int i;

  zArrayAlloc( arr, pdJoint, n );
  if( !zArrayBuf(arr) ){
    ZALLOCERROR();
    return false;
  }
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetOffset( zArrayElem(arr,i), i );
  return true;
}

static bool _pdJointFAlloc(FILE *fp, pdJointArray *arr);

bool _pdJointFAlloc(FILE *fp, pdJointArray *arr)
{
  int n;

  n = zFCountTag( fp, PD_JOINT_TAG );
  return pdJointArrayAlloc( arr, n );
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

  zNameFind( rkChainRoot(c), rkChainNum(c), name, link );
  return link;
}

zIndex pdJointArrayCreateIndex(pdJointArray *arr)
{
  register int i;
  zIndex index;

  if( !( index = zIndexCreate( zArrayNum(arr) ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  for( i=0; i<(int)zArrayNum(arr); i++ ){
    zIndexSetElem( index, i, pdJointArrayOffset( arr, i ) );
  }
  return index;
}

bool pdJointArraySetOffsetMapping(pdJointArray *arr, rkChain *c)
{
  register int i;
  rkLink *link;

  for( i=0; i<(int)zArrayNum(arr); i++ ){
    link = _rkChainLinkFindName(c,zName(zArrayElem(arr,i)));
    if( !link ){
      ZRUNERROR( "joint %s cannot be found in robot model",
                 zName(zArrayElem(arr,i)) );
      return false;
    } else
      pdJointArraySetOffset( arr, i, rkLinkOffset(link) );
  }
  return true;
}

typedef struct{
  pdJointArray *arr;
  int count;
} _pdJointArrayParam;

bool _pdJointArrayFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _pdJointArrayParam *prm;

  prm = instance;
  if( strcmp( buf, PD_JOINT_TAG ) == 0 ){
    if( !pdJointFRead( fp, zArrayElem(prm->arr,prm->count++) ) ){
      *success = false;
      return false;
    }
  } else
    return false;
  return true;
}

static void _pdJointArrayInsertionSort(pdJointArray *arr, int (*cmp)(pdJoint*, pdJoint*));
void _pdJointArrayInsertionSort(pdJointArray *arr, int (*cmp)(pdJoint*, pdJoint*))
{
  register int i, j;
  uint s;
  pdJoint *saved, *value;

  s = sizeof(pdJoint);
  saved = zAlloc( pdJoint, 1 );
  for( j=1; j<(int)zArrayNum(arr); j++ ){
    i = j - 1;
    value = zArrayElem(arr, j);
    while( i >= 0 && cmp(zArrayElem(arr,i), value) > 0 ) i--;
    if( ++i == j ) continue;
    memmove( saved, value, s );
    memmove( zArrayElem(arr,i+1), zArrayElem(arr,i), s*(j-i) );
    memmove( zArrayElem(arr,i), saved, s );
  }
  zFree( saved );
}

static int _pdJointOffsetCmp(pdJoint *p, pdJoint *q);
int _pdJointOffsetCmp(pdJoint *p, pdJoint *q)
{
  if( pdJointOffset(p) == pdJointOffset(q) ) return 0;
  return ( pdJointOffset(p) > pdJointOffset(q) ) ? 1 : -1;
}

bool pdJointArrayFRead(FILE *fp, pdJointArray *arr, rkChain *c)
{
  _pdJointArrayParam prm;
  bool result;

  zArrayInit( arr );
  if( !_pdJointFAlloc( fp, arr ) ) return false;
  rewind( fp );
  prm.count = 0;
  prm.arr = arr;
  result = zTagFRead( fp, _pdJointArrayFRead, &prm );
  if( c )
    result &= pdJointArraySetOffsetMapping( arr, c );
  if( result )
    _pdJointArrayInsertionSort( arr, _pdJointOffsetCmp );
  return result;
}

bool pdJointArrayReadFile(pdJointArray *arr, const char *filename, rkChain *c)
{
  FILE *fp;
  bool result;

  if( !( fp = fopen( filename, "r" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  result = pdJointArrayFRead( fp, arr, c );
  fclose( fp );
  return result;
}

#define PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH \
  "Joint controller number is not matched with the index size"

void pdJointArraySetDisIndex(pdJointArray *arr, zIndex idx, zVec q)
{
  register int i;

  if( zArrayNum(arr) != zArrayNum(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetDis( zArrayElem(arr,i), zVecElem(q,zIndexElem(idx,i)) );
}

void pdJointArraySetVelIndex(pdJointArray *arr, zIndex idx, zVec v)
{
  register int i;

  if( zArrayNum(arr) != zArrayNum(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetVel( zArrayElem(arr,i), zVecElem(v,zIndexElem(idx,i)) );
}

void pdJointArraySetRefDisIndex(pdJointArray *arr, zIndex idx, zVec q)
{
  register int i;

  if( zArrayNum(arr) != zArrayNum(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetRefDis( zArrayElem(arr,i), zVecElem(q,zIndexElem(idx,i)) );
}

void pdJointArraySetRefVelIndex(pdJointArray *arr, zIndex idx, zVec v)
{
  register int i;

  if( zArrayNum(arr) != zArrayNum(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointSetRefVel( zArrayElem(arr,i), zVecElem(v,zIndexElem(idx,i)) );
}

void pdJointArrayRefreshIndex(pdJointArray *arr, zIndex idx, zVec q)
{
  register int i;

  if( zArrayNum(arr) != zArrayNum(idx) )
    ZRUNERROR( PD_JOINT_ERR_MSG_INDEX_SIZE_MISMATCH );
  for( i=0; i<(int)zArrayNum(arr); i++ )
    pdJointRefresh( zArrayElem(arr,i), zVecElem(q,zIndexElem(idx,i)) );
}
