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

void pdJointSetRefDefault(pdJoint *joint, double refdis)
{
  joint->refdis = refdis;
}

void pdJointRefreshDefault(pdJoint *joint, double dis)
{
  joint->dis = dis;
  joint->disold = dis;
  joint->vel = 0;
  joint->velold = 0;
}

void pdJointDestroyDefault(pdJoint *joint)
{
  zNameDestroy( joint );
  zFree( joint->_prm );
  pdJointInit( joint );
}
