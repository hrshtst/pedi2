#ifndef __PD_CORE_H__
#define __PD_CORE_H__

#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_cmd.h>

__BEGIN_DECLS

typedef struct{
  bool stand;
  bool step;
  bool walk;
  bool sidewalk;
  bool follow;
  bool brake;
} pdCoreMode;

typedef struct{
  double _t;
  double _dt;
  pdCZ cz;
  pdFoot lf, rf;
  pdState state;
  pdCmd *cmd;
  pdCoreMode mode;

  zVec3D ref_com_pos;
  zVec3D ref_base_att;
  zVec3D ref_lf_pos;
  zVec3D ref_lf_att;
  zVec3D ref_rf_pos;
  zVec3D ref_rf_att;
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
#define pdCoreStatePtr(c)  ( &(c)->state )
#define pdCoreCmd(c)       ( (c)->cmd )

#define pdCoreKFPtr(c)     pdFootKFPtr( &(c)->lf, &(c)->rf )
#define pdCoreFFPtr(c)     pdFootFFPtr( &(c)->lf, &(c)->rf, (c)->cmd->vwd )
#define pdCoreBFPtr(c)     pdFootBFPtr( &(c)->lf, &(c)->rf, (c)->cmd->vwd )

#define pdCoreRefCOMPos(c)   ( &(c)->ref_com_pos )
#define pdCoreRefCOMPosX(c)  zVec3DElem( pdCoreRefCOMPos(c), zX )
#define pdCoreRefCOMPosY(c)  zVec3DElem( pdCoreRefCOMPos(c), zY )
#define pdCoreRefCOMPosZ(c)  zVec3DElem( pdCoreRefCOMPos(c), zZ )
#define pdCoreRefBaseAtt(c)  ( &(c)->ref_base_att )
#define pdCoreRefBaseAttX(c) zVec3DElem( pdCoreRefBaseAtt(c), zX )
#define pdCoreRefBaseAttY(c) zVec3DElem( pdCoreRefBaseAtt(c), zY )
#define pdCoreRefBaseAttZ(c) zVec3DElem( pdCoreRefBaseAtt(c), zZ )
#define pdCoreRefLFPos(c)    ( &(c)->ref_lf_pos )
#define pdCoreRefLFPosX(c)   zVec3DElem( pdCoreRefLFPos(c), zX )
#define pdCoreRefLFPosY(c)   zVec3DElem( pdCoreRefLFPos(c), zY )
#define pdCoreRefLFPosZ(c)   zVec3DElem( pdCoreRefLFPos(c), zZ )
#define pdCoreRefLFAtt(c)    ( &(c)->ref_lf_att )
#define pdCoreRefLFAttX(c)   zVec3DElem( pdCoreRefLFAtt(c), zX )
#define pdCoreRefLFAttY(c)   zVec3DElem( pdCoreRefLFAtt(c), zY )
#define pdCoreRefLFAttZ(c)   zVec3DElem( pdCoreRefLFAtt(c), zZ )
#define pdCoreRefRFPos(c)    ( &(c)->ref_rf_pos )
#define pdCoreRefRFPosX(c)   zVec3DElem( pdCoreRefRFPos(c), zX )
#define pdCoreRefRFPosY(c)   zVec3DElem( pdCoreRefRFPos(c), zY )
#define pdCoreRefRFPosZ(c)   zVec3DElem( pdCoreRefRFPos(c), zZ )
#define pdCoreRefRFAtt(c)    ( &(c)->ref_rf_att )
#define pdCoreRefRFAttX(c)   zVec3DElem( pdCoreRefRFAtt(c), zX )
#define pdCoreRefRFAttY(c)   zVec3DElem( pdCoreRefRFAtt(c), zY )
#define pdCoreRefRFAttZ(c)   zVec3DElem( pdCoreRefRFAtt(c), zZ )

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
#define pdCoreIncrTime(c) pdCoreTime( c ) += pdCoreTimeStep( c )

bool pdCoreDoesIntendToStand(pdCore *core);
bool pdCoreDoesIntendToStep(pdCore *core);
bool pdCoreDoesIntendToWalk(pdCore *core);
bool pdCoreDoesIntendToSidewalk(pdCore *core);

#define pdCoreIsBothFeetOn(c) pdFootIsBothOn( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsEitherFootOn(c) pdFootIsEitherOn( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsEitherFootOff(c) pdFootIsEitherOff( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsBothFeetOnNext(c) pdFootIsBothOnNext( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsEitherFootOnNext(c) pdFootIsEitherOnNext( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsEitherFootOffNext(c) pdFootIsEitherOffNext( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsBothFeetOnAttempt(c) pdFootIsBothOnAttempt( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsEitherFootOnAttempt(c) pdFootIsEitherOnAttempt( pdCoreLFPtr(c), pdCoreRFPtr(c) )
#define pdCoreIsEitherFootOffAttempt(c) pdFootIsEitherOffAttempt( pdCoreLFPtr(c), pdCoreRFPtr(c) )

__EXPORT void pdCoreInitMode(pdCore *core);
__EXPORT void pdCoreUpdateMode(pdCore *core);
__EXPORT void pdCoreWriteMode(pdCore *core);

/* update method */
__EXPORT void pdCoreUpdate(pdCore *core);

/* output method */
__EXPORT void pdCoreFWrite(FILE *fp, pdCore *core);
#define pdCoreWrite(c) pdCoreFWrite( stdout, c )
__EXPORT void pdCoreDataFWrite(FILE *fp, pdCore *core);
#define pdCoreDataWrite(c) pdCoreDataFWrite( stdout, c )

__END_DECLS

#endif /* __PD_CORE_H__ */
