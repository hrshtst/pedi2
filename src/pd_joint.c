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
