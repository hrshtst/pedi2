#ifndef __PD_CORE_H__
#define __PD_CORE_H__

#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>
#include <pedi2/pd_robot.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_cmd.h>

__BEGIN_DECLS

typedef struct{
  double _t;
  double _dt;
  pdCZ cz;
  pdFoot lf, rf;
  pdRobot robot;
  pdState state;
  pdCmd *cmd;
} pdCore;

__EXPORT void pdCoreInit(pdCore *core, pdCmd *cmd, double dt);
__EXPORT bool pdCoreLoad(pdCore *core, char *filename);
__EXPORT void pdCoreDestroy(pdCore *core);

/* methods to get parameters */
#define pdCoreTime(c)      (c)->_t
#define pdCoreTimeStep(c)  (c)->_dt
#define pdCoreCZPtr(c)     ( &(c)->cz )
#define pdCoreLFPtr(c)     ( &(c)->lf )
#define pdCoreRFPtr(c)     ( &(c)->rf )
#define pdCoreRobotPtr(c)  ( &(c)->robot )
#define pdCoreStatePtr(c)  ( &(c)->state )
#define pdCoreCmd(c)       ( (c)->cmd )
#define pdCoreJointSize(c) pdRobotJointSize( pdCoreRobotPtr( c ) )
#define pdCoreJointDis(c)  pdRobotJointDis( pdCoreRobotPtr( c ) )

/* methods to set parameters */
#define pdCoreSetTime(c,t) do{ \
  (c)->_t = (t); \
  pdCZSetTime( pdCoreCZPtr( c ), t ); \
  pdFootSetTime( pdCoreLFPtr( c ), t ); \
  pdFootSetTime( pdCoreRFPtr( c ), t ); \
} while(0)
#define pdCoreResetTime(c) do{ \
  pdCoreSetTime( c, 0 ); \
  pdCZResetTime( pdCoreCZPtr( c ) ); \
  pdFootResetTime( pdCoreLFPtr( c ) ); \
  pdFootResetTime( pdCoreRFPtr( c ) ); \
} while(0)
#define pdCoreSetTimeStep(c,dt) do{ \
  (c)->_dt = (dt); \
  pdCZSetTimeStep( pdCoreCZPtr( c ), dt ); \
  pdFootSetTimeStep( pdCoreLFPtr( c ), dt ); \
  pdFootSetTimeStep( pdCoreRFPtr( c ), dt ); \
} while(0)
#define pdCoreIncrTime(c) do{ \
  pdCoreTime( c ) += pdCoreTimeStep( c ); \
  pdCZIncrTime( pdCoreCZPtr( c ) ); \
  pdFootIncrTime( pdCoreLFPtr( c ) ); \
  pdFootIncrTime( pdCoreRFPtr( c ) ); \
} while(0)

__END_DECLS

#endif /* __PD_CORE_H__ */
