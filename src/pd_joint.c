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
}

void pdJointRefreshDefault(pdJoint *joint, double dis)
{
  joint->dis = dis;
  joint->disold = dis;
  joint->vel = 0;
  joint->velold = 0;
  joint->refdis = 0;
  joint->refdisold = 0;
  joint->refvel = 0;
  joint->refvelold = 0;
}

void pdJointDestroyDefault(pdJoint *joint)
{
  zNameDestroy( joint );
  zFree( joint->_prm );
  pdJointInit( joint );
}
