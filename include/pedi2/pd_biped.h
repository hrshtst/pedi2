#ifndef __PD_BIPED_H__
#define __PD_BIPED_H__

#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>
#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>

__BEGIN_DECLS

typedef struct{
  bool stand;
  bool step;
  bool walk;
  bool sidewalk;
  bool follow;
  bool brake;
} pdBipedMode;

typedef struct{
  double _t;
  double _dt;
  pdCZ cz;
  pdFoot lf, rf;
  pdCmd *cmd;
  pdBipedMode mode;

  zVec3D ref_com_pos;
  zVec3D ref_base_att;
  zVec3D ref_lf_pos;
  zVec3D ref_lf_att;
  zVec3D ref_rf_pos;
  zVec3D ref_rf_att;
} pdBiped;

__EXPORT void pdBipedInit(pdBiped *biped, pdCmd *cmd, double dt);
__EXPORT bool pdBipedDefaultPoseInit(pdBiped *biped, pdState *state);
__EXPORT void pdBipedDestroy(pdBiped *biped);

/* methods to get parameters */
#define pdBipedTime(c)      (c)->_t
#define pdBipedTimeStep(c)  (c)->_dt
#define pdBipedCZPtr(c)     ( &(c)->cz )
#define pdBipedLFPtr(c)     ( &(c)->lf )
#define pdBipedRFPtr(c)     ( &(c)->rf )
#define pdBipedCmd(c)       ( (c)->cmd )

#define pdBipedKFPtr(c)     pdFootKFPtr( &(c)->lf, &(c)->rf )
#define pdBipedFFPtr(c)     pdFootFFPtr( &(c)->lf, &(c)->rf, (c)->cmd->vwd )
#define pdBipedBFPtr(c)     pdFootBFPtr( &(c)->lf, &(c)->rf, (c)->cmd->vwd )

#define pdBipedRefCOMPos(c)   ( &(c)->ref_com_pos )
#define pdBipedRefCOMPosX(c)  zVec3DElem( pdBipedRefCOMPos(c), zX )
#define pdBipedRefCOMPosY(c)  zVec3DElem( pdBipedRefCOMPos(c), zY )
#define pdBipedRefCOMPosZ(c)  zVec3DElem( pdBipedRefCOMPos(c), zZ )
#define pdBipedRefBaseAtt(c)  ( &(c)->ref_base_att )
#define pdBipedRefBaseAttX(c) zVec3DElem( pdBipedRefBaseAtt(c), zX )
#define pdBipedRefBaseAttY(c) zVec3DElem( pdBipedRefBaseAtt(c), zY )
#define pdBipedRefBaseAttZ(c) zVec3DElem( pdBipedRefBaseAtt(c), zZ )
#define pdBipedRefLFPos(c)    ( &(c)->ref_lf_pos )
#define pdBipedRefLFPosX(c)   zVec3DElem( pdBipedRefLFPos(c), zX )
#define pdBipedRefLFPosY(c)   zVec3DElem( pdBipedRefLFPos(c), zY )
#define pdBipedRefLFPosZ(c)   zVec3DElem( pdBipedRefLFPos(c), zZ )
#define pdBipedRefLFAtt(c)    ( &(c)->ref_lf_att )
#define pdBipedRefLFAttX(c)   zVec3DElem( pdBipedRefLFAtt(c), zX )
#define pdBipedRefLFAttY(c)   zVec3DElem( pdBipedRefLFAtt(c), zY )
#define pdBipedRefLFAttZ(c)   zVec3DElem( pdBipedRefLFAtt(c), zZ )
#define pdBipedRefRFPos(c)    ( &(c)->ref_rf_pos )
#define pdBipedRefRFPosX(c)   zVec3DElem( pdBipedRefRFPos(c), zX )
#define pdBipedRefRFPosY(c)   zVec3DElem( pdBipedRefRFPos(c), zY )
#define pdBipedRefRFPosZ(c)   zVec3DElem( pdBipedRefRFPos(c), zZ )
#define pdBipedRefRFAtt(c)    ( &(c)->ref_rf_att )
#define pdBipedRefRFAttX(c)   zVec3DElem( pdBipedRefRFAtt(c), zX )
#define pdBipedRefRFAttY(c)   zVec3DElem( pdBipedRefRFAtt(c), zY )
#define pdBipedRefRFAttZ(c)   zVec3DElem( pdBipedRefRFAtt(c), zZ )

/* methods to set parameters */
#define pdBipedSetTime(c,t) do{ \
  (c)->_t = (t); \
  pdCZSetTime( pdBipedCZPtr( c ), t ); \
  pdFootSetTime( pdBipedLFPtr( c ), t ); \
  pdFootSetTime( pdBipedRFPtr( c ), t ); \
} while(0)
#define pdBipedResetTime(c) do{ \
  pdBipedSetTime( c, 0 ); \
  pdCZResetTime( pdBipedCZPtr( c ) ); \
  pdFootResetTime( pdBipedLFPtr( c ) ); \
  pdFootResetTime( pdBipedRFPtr( c ) ); \
} while(0)
#define pdBipedSetTimeStep(c,dt) do{ \
  (c)->_dt = (dt); \
  pdCZSetTimeStep( pdBipedCZPtr( c ), dt ); \
  pdFootSetTimeStep( pdBipedLFPtr( c ), dt ); \
  pdFootSetTimeStep( pdBipedRFPtr( c ), dt ); \
} while(0)
#define pdBipedIncrTime(c) pdBipedTime( c ) += pdBipedTimeStep( c )

bool pdBipedDoesIntendToStand(pdBiped *biped);
bool pdBipedDoesIntendToStep(pdBiped *biped);
bool pdBipedDoesIntendToWalk(pdBiped *biped);
bool pdBipedDoesIntendToSidewalk(pdBiped *biped);

#define pdBipedIsBothFeetOn(c) pdFootIsBothOn( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsEitherFootOn(c) pdFootIsEitherOn( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsEitherFootOff(c) pdFootIsEitherOff( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsBothFeetOnNext(c) pdFootIsBothOnNext( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsEitherFootOnNext(c) pdFootIsEitherOnNext( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsEitherFootOffNext(c) pdFootIsEitherOffNext( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsBothFeetOnAttempt(c) pdFootIsBothOnAttempt( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsEitherFootOnAttempt(c) pdFootIsEitherOnAttempt( pdBipedLFPtr(c), pdBipedRFPtr(c) )
#define pdBipedIsEitherFootOffAttempt(c) pdFootIsEitherOffAttempt( pdBipedLFPtr(c), pdBipedRFPtr(c) )

__EXPORT void pdBipedInitMode(pdBiped *biped);
__EXPORT void pdBipedUpdateMode(pdBiped *biped);
__EXPORT void pdBipedWriteMode(pdBiped *biped);

/* update method */
__EXPORT void pdBipedUpdate(pdBiped *biped, pdState *state);
__EXPORT void pdBipedUpdateState(pdBiped *biped, pdState *state);

/* output method */
__EXPORT void pdBipedFWrite(FILE *fp, pdBiped *biped);
#define pdBipedWrite(c) pdBipedFWrite( stdout, c )
__EXPORT void pdBipedDataFWrite(FILE *fp, pdBiped *biped);
#define pdBipedDataWrite(c) pdBipedDataFWrite( stdout, c )

__END_DECLS

#endif /* __PD_BIPED_H__ */
