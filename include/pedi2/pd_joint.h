#ifndef __PD_JOINT_H__
#define __PD_JOINT_H__

#include <zm/zm_vec.h>
#include <roki/rk_chain.h>
#include <pedi2/pd_exportdecl.h>

__BEGIN_DECLS

struct _pdJoint;

typedef struct{
  const char *type;
  void (*setdis)(struct _pdJoint*, double);
  void (*setvel)(struct _pdJoint*, double);
  void (*setrefdis)(struct _pdJoint*, double);
  void (*setrefvel)(struct _pdJoint*, double);
  void (*refresh)(struct _pdJoint*, double);
  void (*update)(struct _pdJoint*, double);
  void (*destroy)(struct _pdJoint*);
  struct _pdJoint *(*fread)(FILE *fp, struct _pdJoint*);
} pdJointMethod;

typedef struct _pdJoint{
  Z_NAMED_CLASS
  double dis;
  double vel;
  double disold;
  double velold;
  double refdis;
  double refvel;
  double refdisold;
  double refvelold;
  double output;
  bool is_set_vel;
  bool is_set_refvel;
  int offset;
  void *_prm;
  pdJointMethod *_met;
} pdJoint;

#define pdJointDis(j)       ( (j)->dis )
#define pdJointVel(j)       ( (j)->vel )
#define pdJointDisOld(j)    ( (j)->disold )
#define pdJointVelOld(j)    ( (j)->velold )
#define pdJointRefDis(j)    ( (j)->refdis )
#define pdJointRefVel(j)    ( (j)->refvel )
#define pdJointRefDisOld(j) ( (j)->refdisold )
#define pdJointRefVelOld(j) ( (j)->refvelold )
#define pdJointOutput(j)    ( (j)->output )
#define pdJointOffset(j)    ( (j)->offset )

#define pdJointInit(j) do{\
  zNameSet( j, NULL );\
  pdJointDis(j) = 0;\
  pdJointVel(j) = 0;\
  pdJointDisOld(j) = 0;\
  pdJointVelOld(j) = 0;\
  pdJointRefDis(j) = 0;\
  pdJointRefVel(j) = 0;\
  pdJointRefDisOld(j) = 0;\
  pdJointRefVelOld(j) = 0;\
  pdJointOutput(j) = 0;\
  (j)->is_set_vel = false;\
  (j)->is_set_refvel = false;\
  pdJointSetOffset( j, 0 ); \
  (j)->_prm = NULL;\
  (j)->_met = NULL;\
} while(0)

#define pdJointSetDis(j,q)    (j)->_met->setdis( j, q )
#define pdJointSetVel(j,v)    (j)->_met->setvel( j, v )
#define pdJointSetRefDis(j,q) (j)->_met->setrefdis( j, q )
#define pdJointSetRefVel(j,q) (j)->_met->setrefvel( j, q )
#define pdJointRefresh(j,q)   (j)->_met->refresh( j, q )
#define pdJointUpdate(j,dt)   (j)->_met->update( j, dt )
#define pdJointDestroy(j)     (j)->_met->destroy( j )
#define pdJointSetOffset(j,o) ( (j)->offset = (o) )

__EXPORT void pdJointSetDisDefault(pdJoint *joint, double dis);
__EXPORT void pdJointSetVelDefault(pdJoint *joint, double vel);
__EXPORT void pdJointSetRefDisDefault(pdJoint *joint, double refdis);
__EXPORT void pdJointSetRefVelDefault(pdJoint *joint, double refvel);
__EXPORT void pdJointRefreshDefault(pdJoint *joint, double dis);
__EXPORT void pdJointUpdateDefault(pdJoint *joint, double dt);
__EXPORT void pdJointDestroyDefault(pdJoint *joint);

#define PD_JOINT_TAG "joint"
__EXPORT pdJoint *pdJointFRead(FILE *fp, pdJoint *joint);

zArrayClass( pdJointArray, pdJoint );

#define pdJointArrayDis(arr,i)      pdJointDis( zArrayElem(arr,i) )
#define pdJointArrayVel(arr,i)      pdJointVel( zArrayElem(arr,i) )
#define pdJointArrayRefDis(arr,i)   pdJointRefDis( zArrayElem(arr,i) )
#define pdJointArrayRefVel(arr,i)   pdJointRefVel( zArrayElem(arr,i) )
#define pdJointArrayOutput(arr,i)   pdJointOutput( zArrayElem(arr,i) )
#define pdJointArrayOffset(arr,i)   pdJointOffset( zArrayElem(arr,i) )
#define pdJointArraySetOffset(arr,i,o) pdJointSetOffset( zArrayElem(arr,i), o )

__EXPORT void pdJointArraySetDis(pdJointArray *arr, zVec q);
__EXPORT void pdJointArraySetVel(pdJointArray *arr, zVec v);
__EXPORT void pdJointArraySetRefDis(pdJointArray *arr, zVec qref);
__EXPORT void pdJointArraySetRefVel(pdJointArray *arr, zVec vref);
__EXPORT void pdJointArrayRefresh(pdJointArray *arr, zVec q);
__EXPORT void pdJointArrayUpdate(pdJointArray *arr, double dt);
__EXPORT void pdJointArrayDestroy(pdJointArray *arr);

__EXPORT bool pdJointArrayAlloc(pdJointArray *arr, int n);
__EXPORT pdJoint *pdJointArrayNameFind(pdJointArray *arr, const char *name);
__EXPORT bool pdJointArraySetOffsetMapping(pdJointArray *arr, rkChain *c);
__EXPORT bool pdJointArrayFRead(FILE *fp, pdJointArray *arr, rkChain *c);
__EXPORT bool pdJointArrayReadFile(pdJointArray *arr, const char *filename, rkChain *c);

__EXPORT zIndex pdJointArrayCreateIndex(pdJointArray *arr);
__EXPORT void pdJointArraySetDisIndex(pdJointArray *arr, zIndex idx, zVec q);
__EXPORT void pdJointArraySetVelIndex(pdJointArray *arr, zIndex idx, zVec v);
__EXPORT void pdJointArraySetRefDisIndex(pdJointArray *arr, zIndex idx, zVec qref);
__EXPORT void pdJointArraySetRefVelIndex(pdJointArray *arr, zIndex idx, zVec vref);
__EXPORT void pdJointArrayRefreshIndex(pdJointArray *arr, zIndex idx, zVec q);

__END_DECLS

#include <pedi2/pd_joint_pd_trq.h>   /* Torque PD control */
#include <pedi2/pd_joint_pid_trq.h>  /* Torque PID control */

#endif /* __PD_JOINT_H__ */
