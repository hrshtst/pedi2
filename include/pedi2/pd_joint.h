#ifndef __PD_JOINT_H__
#define __PD_JOINT_H__

#include <zm/zm_vec.h>

__BEGIN_DECLS

struct _pdJoint;

typedef struct{
  const char *type;
  void (*setdis)(struct _pdJoint*, double);
  void (*setvel)(struct _pdJoint*, double);
  void (*setref)(struct _pdJoint*, double);
  void (*refresh)(struct _pdJoint*, double);
  void (*update)(struct _pdJoint*, double);
  void (*destroy)(struct _pdJoint*);
} pdJointMethod;

typedef struct _pdJoint{
  Z_NAMED_CLASS;
  double dis;
  double vel;
  double disold;
  double velold;
  double refdis;
  double output;
  void *_prm;
  pdJointMethod *_met;
} pdJoint;

#define pdJointDis(j) ( (j)->dis )
#define pdJointVel(j) ( (j)->vel )
#define pdJointDisOld(j) ( (j)->disold )
#define pdJointVelOld(j) ( (j)->velold )
#define pdJointRefDis(j) ( (j)->refdis )
#define pdJointOutput(j) ( (j)->output )

#define pdJointInit(j) do{\
  zNameSet( j, NULL );\
  pdJointDis(j) = 0;\
  pdJointVel(j) = 0;\
  pdJointDisOld(j) = 0;\
  pdJointVelOld(j) = 0;\
  pdJointRefDis(j) = 0;\
  pdJointOutput(j) = 0;\
  (j)->_prm = NULL;\
  (j)->_met = NULL;\
} while(0)

#define pdJointSetDis(j,q)  (j)->_met->setdis( j, q )
#define pdJointSetVel(j,v)  (j)->_met->setvel( j, v )
#define pdJointSetRef(j,q)  (j)->_met->setref( j, q )
#define pdJointRefresh(j,q) (j)->_met->refresh( j, q )
#define pdJointUpdate(j,dt) (j)->_met->update( j, dt )
#define pdJointDestroy(j)   (j)->_met->destroy( j )

__EXPORT void pdJointSetDisDefault(pdJoint *joint, double dis);
__EXPORT void pdJointSetVelDefault(pdJoint *joint, double vel);
__EXPORT void pdJointSetRefDefault(pdJoint *joint, double refdis);
__EXPORT void pdJointRefreshDefault(pdJoint *joint, double dis);
__EXPORT void pdJointDestroyDefault(pdJoint *joint);

__END_DECLS

#include <pedi2/pd_joint_pd_trq.h>  /* Torque PD control */

#endif /* __PD_JOINT_H__ */
