#ifndef __PD_JOINT_H__
#define __PD_JOINT_H__

#include <zm/zm_vec.h>
#include <roki/rk_chain.h>
#include <pedi2/pd_misc.h>

__BEGIN_DECLS

struct _pdJoint;

typedef struct{
  const char *typestr;
  void (* _setdis)(struct _pdJoint*, double);
  void (* _setvel)(struct _pdJoint*, double);
  void (* _setrefdis)(struct _pdJoint*, double);
  void (* _setrefvel)(struct _pdJoint*, double);
  void (* _refresh)(struct _pdJoint*, double);
  void (* _update)(struct _pdJoint*, double);
  void (* _destroy)(struct _pdJoint*);
  struct _pdJoint *(* _fromZTK)(struct _pdJoint*, ZTK*);
  void (* _fprintZTK)(FILE*, struct _pdJoint*);
} pdJointCom;

typedef struct _pdJoint{
  Z_NAMED_CLASS;
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
  void *prp;
  pdJointCom *com;
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
  (j)->prp = NULL;\
  (j)->com = NULL;\
} while(0)

#define pdJointSetDis(j,q)    (j)->com->_setdis( j, q )
#define pdJointSetVel(j,v)    (j)->com->_setvel( j, v )
#define pdJointSetRefDis(j,q) (j)->com->_setrefdis( j, q )
#define pdJointSetRefVel(j,q) (j)->com->_setrefvel( j, q )
#define pdJointRefresh(j,q)   (j)->com->_refresh( j, q )
#define pdJointUpdate(j,dt)   (j)->com->_update( j, dt )
#define pdJointDestroy(j)     (j)->com->_destroy( j )
#define pdJointSetOffset(j,o) ( (j)->offset = (o) )

__PEDI2_EXPORT void pdJointDefaultSetDis(pdJoint *joint, double dis);
__PEDI2_EXPORT void pdJointDefaultSetVel(pdJoint *joint, double vel);
__PEDI2_EXPORT void pdJointDefaultSetRefDis(pdJoint *joint, double refdis);
__PEDI2_EXPORT void pdJointDefaultSetRefVel(pdJoint *joint, double refvel);
__PEDI2_EXPORT void pdJointDefaultRefresh(pdJoint *joint, double dis);
__PEDI2_EXPORT void pdJointDefaultUpdate(pdJoint *joint, double dt);
__PEDI2_EXPORT void pdJointDefaultDestroy(pdJoint *joint);

#define ZTK_TAG_PEDI2_JOINT       "pedi2::joint"

#define ZTK_KEY_PEDI2_JOINT_NAME  "name"
#define ZTK_KEY_PEDI2_JOINT_TYPE  "type"
#define ZTK_KEY_PEDI2_JOINT_PGAIN "pgain"
#define ZTK_KEY_PEDI2_JOINT_IGAIN "igain"
#define ZTK_KEY_PEDI2_JOINT_DGAIN "dgain"
#define ZTK_KEY_PEDI2_JOINT_MIN   "min"
#define ZTK_KEY_PEDI2_JOINT_MAX   "max"

__PEDI2_EXPORT pdJoint *pdJointFromZTK(pdJoint *joint, ZTK *ztk);
__PEDI2_EXPORT void pdJointFPrintZTK(FILE *fp, pdJoint *joint);

ZEDA_DEF_ARRAY_CLASS( pdJointArray, pdJoint );

#define pdJointArrayDis(arr,i)         pdJointDis( zArrayElem(arr,i) )
#define pdJointArrayVel(arr,i)         pdJointVel( zArrayElem(arr,i) )
#define pdJointArrayRefDis(arr,i)      pdJointRefDis( zArrayElem(arr,i) )
#define pdJointArrayRefVel(arr,i)      pdJointRefVel( zArrayElem(arr,i) )
#define pdJointArrayOutput(arr,i)      pdJointOutput( zArrayElem(arr,i) )
#define pdJointArrayOffset(arr,i)      pdJointOffset( zArrayElem(arr,i) )
#define pdJointArraySetOffset(arr,i,o) pdJointSetOffset( zArrayElem(arr,i), o )

__PEDI2_EXPORT void pdJointArraySetDis(pdJointArray *arr, zVec q);
__PEDI2_EXPORT void pdJointArraySetVel(pdJointArray *arr, zVec v);
__PEDI2_EXPORT void pdJointArraySetRefDis(pdJointArray *arr, zVec qref);
__PEDI2_EXPORT void pdJointArraySetRefVel(pdJointArray *arr, zVec vref);
__PEDI2_EXPORT void pdJointArrayRefresh(pdJointArray *arr, zVec q);
__PEDI2_EXPORT void pdJointArrayUpdate(pdJointArray *arr, double dt);
__PEDI2_EXPORT void pdJointArrayDestroy(pdJointArray *arr);

__PEDI2_EXPORT pdJointArray *pdJointArrayAlloc(pdJointArray *arr, int size);
__PEDI2_EXPORT pdJoint *pdJointArrayNameFind(pdJointArray *arr, const char *name);
__PEDI2_EXPORT bool pdJointArraySetOffsetMapping(pdJointArray *arr, rkChain *c);

__PEDI2_EXPORT pdJointArray *pdJointArrayFromZTK(pdJointArray *arr, rkChain *c, ZTK *ztk);
__PEDI2_EXPORT void pdJointArrayFPrintZTK(FILE *fp, pdJointArray *arr);

__PEDI2_EXPORT pdJointArray *pdJointArrayReadZTK(pdJointArray *arr, rkChain *c, char filename[]);
__PEDI2_EXPORT bool pdJointArrayWriteZTK(pdJointArray *arr, char filename[]);

__PEDI2_EXPORT zIndex pdJointArrayCreateIndex(pdJointArray *arr);
__PEDI2_EXPORT void pdJointArraySetDisIndex(pdJointArray *arr, zIndex idx, zVec q);
__PEDI2_EXPORT void pdJointArraySetVelIndex(pdJointArray *arr, zIndex idx, zVec v);
__PEDI2_EXPORT void pdJointArraySetRefDisIndex(pdJointArray *arr, zIndex idx, zVec qref);
__PEDI2_EXPORT void pdJointArraySetRefVelIndex(pdJointArray *arr, zIndex idx, zVec vref);
__PEDI2_EXPORT void pdJointArrayRefreshIndex(pdJointArray *arr, zIndex idx, zVec q);

__END_DECLS

#include <pedi2/pd_joint_pd_trq.h>   /* Torque PD control */
#include <pedi2/pd_joint_pid_trq.h>  /* Torque PID control */

__BEGIN_DECLS

/* add a handle to the following list when you create a new joint class. */
#define PD_JOINT_COM_ARRAY \
pdJointCom *pd_joint_com[] = { \
  &pd_joint_pd_trq_com, \
  &pd_joint_pid_trq_com, \
  NULL, \
}

__END_DECLS

#endif /* __PD_JOINT_H__ */
