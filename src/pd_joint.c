#include <pedi2/pd_joint.h>

void pdJointSetDisDefault(pdJoint *joint, double dis)
{
  joint->dis = dis;
}

void pdJointSetVelDefault(pdJoint *joint, double vel)
{
  joint->vel = vel;
}

void pdJointSetRefDefault(pdJoint *joint, double refdis)
{
  joint->refdis = refdis;
}

void pdJointRefreshDefault(pdJoint *joint, double dis)
{
  joint->dis = dis;
  joint->vel = 0;
}

void pdJointDestroyDefault(pdJoint *joint)
{
  zNameDestroy( joint );
  zFree( joint->_prm );
  pdJointInit( joint );
}
