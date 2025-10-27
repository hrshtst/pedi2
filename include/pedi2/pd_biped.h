#ifndef __PD_BIPED_H__
#define __PD_BIPED_H__

#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>
#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_mode.h>

__BEGIN_DECLS

typedef struct{
  double _t;
  double _dt;
  pdCZ cz;
  pdFoot lf, rf;
  pdCmd *cmd;
  pdMode mode;

  zVec3D ref_com_pos;
  zVec3D ref_torso_att;
  zVec3D ref_lf_pos;
  zVec3D ref_lf_att;
  zVec3D ref_rf_pos;
  zVec3D ref_rf_att;
} pdBiped;

__EXPORT void pdBipedInit(pdBiped *biped, pdCmd *cmd, double dt);
__EXPORT bool pdBipedDefaultPoseInit(pdBiped *biped, pdState *state);
__EXPORT void pdBipedDestroy(pdBiped *biped);

/* methods to get parameters */
#define pdBipedTime(b)        (b)->_t
#define pdBipedTimeStep(b)    (b)->_dt
#define pdBipedCZPtr(b)       ( &(b)->cz )
#define pdBipedLFPtr(b)       ( &(b)->lf )
#define pdBipedRFPtr(b)       ( &(b)->rf )
#define pdBipedCmd(b)         ( (b)->cmd )

#define pdBipedKFPtr(b)       pdFootKFPtr( &(b)->lf, &(b)->rf )
#define pdBipedFFPtr(b)       pdFootFFPtr( &(b)->lf, &(b)->rf, (b)->cmd->vwd )
#define pdBipedBFPtr(b)       pdFootBFPtr( &(b)->lf, &(b)->rf, (b)->cmd->vwd )

#define pdBipedRefCOMPos(b)    ( &(b)->ref_com_pos )
#define pdBipedRefCOMPosX(b)   pdBipedRefCOMPos(b)->c.x
#define pdBipedRefCOMPosY(b)   pdBipedRefCOMPos(b)->c.y
#define pdBipedRefCOMPosZ(b)   pdBipedRefCOMPos(b)->c.z
#define pdBipedRefTorsoAtt(b)  ( &(b)->ref_torso_att )
#define pdBipedRefTorsoAttX(b) pdBipedRefTorsoAtt(b)->c.x
#define pdBipedRefTorsoAttY(b) pdBipedRefTorsoAtt(b)->c.y
#define pdBipedRefTorsoAttZ(b) pdBipedRefTorsoAtt(b)->c.z
#define pdBipedRefLFPos(b)     ( &(b)->ref_lf_pos )
#define pdBipedRefLFPosX(b)    pdBipedRefLFPos(b)->c.x
#define pdBipedRefLFPosY(b)    pdBipedRefLFPos(b)->c.y
#define pdBipedRefLFPosZ(b)    pdBipedRefLFPos(b)->c.z
#define pdBipedRefLFAtt(b)     ( &(b)->ref_lf_att )
#define pdBipedRefLFAttX(b)    pdBipedRefLFAtt(b)->c.x
#define pdBipedRefLFAttY(b)    pdBipedRefLFAtt(b)->c.y
#define pdBipedRefLFAttZ(b)    pdBipedRefLFAtt(b)->c.z
#define pdBipedRefRFPos(b)     ( &(b)->ref_rf_pos )
#define pdBipedRefRFPosX(b)    pdBipedRefRFPos(b)->c.x
#define pdBipedRefRFPosY(b)    pdBipedRefRFPos(b)->c.y
#define pdBipedRefRFPosZ(b)    pdBipedRefRFPos(b)->c.z
#define pdBipedRefRFAtt(b)     ( &(b)->ref_rf_att )
#define pdBipedRefRFAttX(b)    pdBipedRefRFAtt(b)->c.x
#define pdBipedRefRFAttY(b)    pdBipedRefRFAtt(b)->c.y
#define pdBipedRefRFAttZ(b)    pdBipedRefRFAtt(b)->c.z

/* methods to set parameters */
#define pdBipedSetTime(b,t) do{ \
  (b)->_t = (t); \
  pdCZSetTime( pdBipedCZPtr(b), t ); \
  pdFootSetTime( pdBipedLFPtr(b), t ); \
  pdFootSetTime( pdBipedRFPtr(b), t ); \
} while(0)
#define pdBipedResetTime(b) do{ \
  pdBipedSetTime( b, 0 ); \
  pdCZResetTime( pdBipedCZPtr(b) ); \
  pdFootResetTime( pdBipedLFPtr(b) ); \
  pdFootResetTime( pdBipedRFPtr(b) ); \
} while(0)
#define pdBipedSetTimeStep(b,dt) do{ \
  (b)->_dt = (dt); \
  pdCZSetTimeStep( pdBipedCZPtr(b), dt ); \
  pdFootSetTimeStep( pdBipedLFPtr(b), dt ); \
  pdFootSetTimeStep( pdBipedRFPtr(b), dt ); \
} while(0)
#define pdBipedIncrTime(b) pdBipedTime(b) += pdBipedTimeStep(b)

bool pdBipedDoesIntendToStand(pdBiped *biped);
bool pdBipedDoesIntendToStep(pdBiped *biped);
bool pdBipedDoesIntendToWalk(pdBiped *biped);
bool pdBipedDoesIntendToSidewalk(pdBiped *biped);

#define pdBipedIsBothFeetOn(b) pdFootIsBothOn( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsEitherFootOn(b) pdFootIsEitherOn( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsEitherFootOff(b) pdFootIsEitherOff( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsBothFeetOnNext(b) pdFootIsBothOnNext( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsEitherFootOnNext(b) pdFootIsEitherOnNext( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsEitherFootOffNext(b) pdFootIsEitherOffNext( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsBothFeetOnAttempt(b) pdFootIsBothOnAttempt( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsEitherFootOnAttempt(b) pdFootIsEitherOnAttempt( pdBipedLFPtr(b), pdBipedRFPtr(b) )
#define pdBipedIsEitherFootOffAttempt(b) pdFootIsEitherOffAttempt( pdBipedLFPtr(b), pdBipedRFPtr(b) )

/* update method */
__EXPORT void pdBipedUpdate(pdBiped *biped, pdState *state);
__EXPORT void pdBipedUpdateState(pdBiped *biped, pdState *state);

/* output method */
__EXPORT void pdBipedFWrite(FILE *fp, pdBiped *biped);
#define pdBipedWrite(b) pdBipedFWrite( stdout, c )
__EXPORT void pdBipedDataFWrite(FILE *fp, pdBiped *biped);
#define pdBipedDataWrite(b) pdBipedDataFWrite( stdout, c )

__END_DECLS

#endif /* __PD_BIPED_H__ */
