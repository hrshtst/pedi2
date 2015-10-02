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
