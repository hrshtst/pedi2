#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_biped.h>

const double TIME_STEP = 0.01;
#define GTEST_TOL 1e-12
#define GTEST_TOL_LOOSE 1e-04

class pdBipedTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdStateInit( &state );
    pdBipedInit( &biped, &cmd, TIME_STEP );
    destroy_flag = false;
  };
  virtual void TearDown() {
    if( !destroy_flag )
      pdBipedDestroy( &biped );
    pdStateDestroy( &state );
  };

  void SetRandomValues() {
    ri.SetRandVec3D( biped.ref_com_pos );
    ri.SetRandVec3D( biped.ref_torso_att );
    ri.SetRandVec3D( biped.ref_lf_pos );
    ri.SetRandVec3D( biped.ref_lf_att );
    ri.SetRandVec3D( biped.ref_rf_pos );
    ri.SetRandVec3D( biped.ref_rf_att );
    ri.SetRandVec3D( pdBipedCZPtr(&biped)->refvel );
    ri.SetRandVec3D( pdBipedCZPtr(&biped)->refacc );
    ri.SetRandVec3D( pdBipedCZPtr(&biped)->refzmp );
    ri.SetRandScalar( pdCZVrtRF( &biped.cz._vrt ) );
  };

  void SupportOnBothFeet() {
    pdFootPosZ( pdBipedLFPtr(&biped) ) = 0.0;
    pdFootPosZ( pdBipedRFPtr(&biped) ) = 0.0;
    pdFootSR( pdBipedLFPtr(&biped) ) = &state.sr_lf;
    pdFootSR( pdBipedRFPtr(&biped) ) = &state.sr_lf;
  };

  void SupportOnLeftFoot() {
    pdFootPosZ( pdBipedLFPtr(&biped) ) = 0.0;
    pdFootPosZ( pdBipedRFPtr(&biped) ) = 0.01;
    pdFootSR( pdBipedLFPtr(&biped) ) = &state.sr_lf;
    pdFootSR( pdBipedRFPtr(&biped) ) = NULL;
  };

  void SupportOnRightFoot() {
    pdFootPosZ( pdBipedLFPtr(&biped) ) = 0.01;
    pdFootPosZ( pdBipedRFPtr(&biped) ) = 0.0;
    pdFootSR( pdBipedLFPtr(&biped) ) = NULL;
    pdFootSR( pdBipedRFPtr(&biped) ) = &state.sr_lf;
  };

  RandomInitializer ri;
  bool destroy_flag;
  pdBiped biped;
  pdCmd cmd;
  pdState state;
};

TEST_F(pdBipedTest, Init)
{
  SetRandomValues();
  pdBipedInit( &biped, &cmd, TIME_STEP );
  EXPECT_EQ( 0, pdBipedTime( &biped ) );
  EXPECT_EQ( TIME_STEP, pdBipedTimeStep( &biped ) );
  EXPECT_EQ( &cmd, pdBipedCmd( &biped ) );
  EXPECT_EQ( &biped.cz, pdBipedCZPtr( &biped ) );
  EXPECT_EQ( &biped.lf, pdBipedLFPtr( &biped ) );
  EXPECT_EQ( &biped.rf, pdBipedRFPtr( &biped ) );
  EXPECT_EQ( 0, pdBipedRefCOMPosX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefCOMPosY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefCOMPosZ( &biped ) );
  EXPECT_EQ( 0, pdBipedRefTorsoAttX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefTorsoAttY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefTorsoAttZ( &biped ) );
  EXPECT_EQ( 0, pdBipedRefLFPosX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefLFPosY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefLFPosZ( &biped ) );
  EXPECT_EQ( 0, pdBipedRefLFAttX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefLFAttY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefLFAttZ( &biped ) );
  EXPECT_EQ( 0, pdBipedRefRFPosX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefRFPosY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefRFPosZ( &biped ) );
  EXPECT_EQ( 0, pdBipedRefRFAttX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefRFAttY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefRFAttZ( &biped ) );
}

TEST_F(pdBipedTest, Destroy)
{
  pdBipedDestroy( &biped );
  EXPECT_EQ( 0, pdBipedTime( &biped ) );
  EXPECT_EQ( 0, pdBipedTimeStep( &biped ) );
  EXPECT_EQ( NULL, pdBipedCmd( &biped ) );
  destroy_flag = true;
}

TEST_F(pdBipedTest, SetTime)
{
  pdBipedSetTime( &biped, 10 );
  EXPECT_EQ( 10, pdBipedTime( &biped ) );
  EXPECT_EQ( 10, pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( 10, pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( 10, pdFootTime( pdBipedRFPtr( &biped ) ) );
}

TEST_F(pdBipedTest, ResetTime)
{
  pdBipedSetTime( &biped, 5 );
  EXPECT_EQ( 5, pdBipedTime( &biped ) );
  EXPECT_EQ( 5, pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( 5, pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( 5, pdFootTime( pdBipedRFPtr( &biped ) ) );
  pdBipedResetTime( &biped );
  EXPECT_EQ( 0, pdBipedTime( &biped ) );
  EXPECT_EQ( 0, pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( 0, pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( 0, pdFootTime( pdBipedRFPtr( &biped ) ) );
}

TEST_F(pdBipedTest, SetTimeStep)
{
  pdBipedSetTimeStep( &biped, 0.005 );
  EXPECT_EQ( 0.005, pdBipedTimeStep( &biped ) );
  EXPECT_EQ( 0.005, pdCZTimeStep( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( 0.005, pdFootTimeStep( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( 0.005, pdFootTimeStep( pdBipedRFPtr( &biped ) ) );
}

TEST_F(pdBipedTest, IncrTime)
{
  pdBipedIncrTime( &biped );
  EXPECT_EQ( TIME_STEP, pdBipedTime( &biped ) );
  pdBipedIncrTime( &biped );
  EXPECT_EQ( 2*TIME_STEP, pdBipedTime( &biped ) );
}

TEST_F(pdBipedTest, IncrTime_Update)
{
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
  pdBipedUpdate( &biped, &state );
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
  pdBipedUpdate( &biped, &state );
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
  pdBipedUpdate( &biped, &state );
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
}

TEST_F(pdBipedTest, RefVec)
{
  EXPECT_EQ( &biped.ref_com_pos, pdBipedRefCOMPos( &biped ) );
  EXPECT_EQ( &biped.ref_torso_att, pdBipedRefTorsoAtt( &biped ) );
  EXPECT_EQ( &biped.ref_lf_pos, pdBipedRefLFPos( &biped ) );
  EXPECT_EQ( &biped.ref_lf_att, pdBipedRefLFAtt( &biped ) );
  EXPECT_EQ( &biped.ref_rf_pos, pdBipedRefRFPos( &biped ) );
  EXPECT_EQ( &biped.ref_rf_att, pdBipedRefRFAtt( &biped ) );
}

TEST_F(pdBipedTest, DefaultPoseInit_ThrowException)
{
  zEchoOff();
  pdStateInit( &state );
  EXPECT_FALSE( pdBipedDefaultPoseInit( &biped, &state ) );
  zEchoOn();
}

TEST_F(pdBipedTest, DefaultPoseInit)
{
  pdStateInit( &state );
  zVec3DCreate( &state.lf_pos, 0,  0.1, -0.1 );
  zVec3DCreate( &state.rf_pos, 0, -0.1, -0.1 );
  zVec3DCreate( &state.com_pos, 0.01, 0, 0.1 );
  pdCmdDefaultInit( biped.cmd );
  EXPECT_TRUE( pdBipedDefaultPoseInit( &biped, &state ) );

  EXPECT_DOUBLE_EQ( 0.2, biped.cmd->dist );
  EXPECT_DOUBLE_EQ( 0.95*0.2, biped.cmd->zd );
  EXPECT_DOUBLE_EQ( -zPI_2, biped.cmd->thetad );
  EXPECT_DOUBLE_EQ( 0.01, biped.cmd->xd );
  EXPECT_DOUBLE_EQ( 0, biped.cmd->yd );

  EXPECT_DOUBLE_EQ( 0.01, pdBipedRefCOMPosX( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefCOMPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0.95*0.2, pdBipedRefCOMPosZ( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefTorsoAttX( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefTorsoAttY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefTorsoAttZ( &biped ) );
  EXPECT_DOUBLE_EQ( 0.01, pdBipedRefLFPosX( &biped ) );
  EXPECT_DOUBLE_EQ( 0.1, pdBipedRefLFPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefLFPosZ( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefLFAttX( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefLFAttY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefLFAttZ( &biped ) );
  EXPECT_DOUBLE_EQ( 0.01, pdBipedRefRFPosX( &biped ) );
  EXPECT_DOUBLE_EQ( -0.1, pdBipedRefRFPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefRFPosZ( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefRFAttX( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefRFAttY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefRFAttZ( &biped ) );
}

TEST_F(pdBipedTest, DoesIntendToStand_Step_Walk)
{
  pdCmdDefaultInit( &cmd );

  EXPECT_TRUE( pdBipedDoesIntendToStand( &biped ) );
  EXPECT_FALSE( pdBipedDoesIntendToStep( &biped ) );
  EXPECT_FALSE( pdBipedDoesIntendToWalk( &biped ) );

  cmd.rho = 1.0;
  EXPECT_FALSE( pdBipedDoesIntendToStand( &biped ) );
  EXPECT_TRUE( pdBipedDoesIntendToStep( &biped ) );
  EXPECT_FALSE( pdBipedDoesIntendToWalk( &biped ) );

  cmd.rho = 1.0;
  cmd.vud = 0.1;
  EXPECT_FALSE( pdBipedDoesIntendToStand( &biped ) );
  EXPECT_TRUE( pdBipedDoesIntendToStep( &biped ) );
  EXPECT_TRUE( pdBipedDoesIntendToWalk( &biped ) );

  destroy_flag = true;
}

TEST_F(pdBipedTest, DoesIntendToStand_Step_Sidewalk)
{
  pdCmdDefaultInit( &cmd );

  EXPECT_TRUE( pdBipedDoesIntendToStand( &biped ) );
  EXPECT_FALSE( pdBipedDoesIntendToStep( &biped ) );
  EXPECT_FALSE( pdBipedDoesIntendToSidewalk( &biped ) );

  cmd.rho = 1.0;
  EXPECT_FALSE( pdBipedDoesIntendToStand( &biped ) );
  EXPECT_TRUE( pdBipedDoesIntendToStep( &biped ) );
  EXPECT_FALSE( pdBipedDoesIntendToSidewalk( &biped ) );

  cmd.rho = 1.0;
  cmd.vwd = 0.1;
  EXPECT_FALSE( pdBipedDoesIntendToStand( &biped ) );
  EXPECT_TRUE( pdBipedDoesIntendToStep( &biped ) );
  EXPECT_TRUE( pdBipedDoesIntendToSidewalk( &biped ) );

  destroy_flag = true;
}

TEST_F(pdBipedTest, BothFeetOn)
{
  SupportOnBothFeet();
  EXPECT_TRUE( pdBipedIsBothFeetOn( &biped ) );
  EXPECT_TRUE( pdBipedIsEitherFootOn( &biped ) );
  EXPECT_FALSE( pdBipedIsEitherFootOff( &biped ) );

  SupportOnLeftFoot();
  EXPECT_FALSE( pdBipedIsBothFeetOn( &biped ) );
  EXPECT_TRUE( pdBipedIsEitherFootOn( &biped ) );
  EXPECT_TRUE( pdBipedIsEitherFootOff( &biped ) );

  SupportOnRightFoot();
  EXPECT_FALSE( pdBipedIsBothFeetOn( &biped ) );
  EXPECT_TRUE( pdBipedIsEitherFootOn( &biped ) );
  EXPECT_TRUE( pdBipedIsEitherFootOff( &biped ) );

  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateState)
{
  SetRandomValues();
  pdBipedUpdateState( &biped, &state );
  pdState *s = &state;
  EXPECT_EQ( pdBipedRefCOMPosX(&biped), s->com_pos.c.x );
  EXPECT_EQ( pdBipedRefCOMPosY(&biped), s->com_pos.c.y );
  EXPECT_EQ( pdBipedRefCOMPosZ(&biped), s->com_pos.c.z );
  EXPECT_EQ( pdBipedRefTorsoAttX(&biped), s->torso_att.c.x );
  EXPECT_EQ( pdBipedRefTorsoAttY(&biped), s->torso_att.c.y );
  EXPECT_EQ( pdBipedRefTorsoAttZ(&biped), s->torso_att.c.z );
  EXPECT_EQ( pdBipedRefLFPosX(&biped), s->lf_pos.c.x );
  EXPECT_EQ( pdBipedRefLFPosY(&biped), s->lf_pos.c.y );
  EXPECT_EQ( pdBipedRefLFPosZ(&biped), s->lf_pos.c.z );
  EXPECT_EQ( pdBipedRefLFAttX(&biped), s->lf_att.c.x );
  EXPECT_EQ( pdBipedRefLFAttY(&biped), s->lf_att.c.y );
  EXPECT_EQ( pdBipedRefLFAttZ(&biped), s->lf_att.c.z );
  EXPECT_EQ( pdBipedRefRFPosX(&biped), s->rf_pos.c.x );
  EXPECT_EQ( pdBipedRefRFPosY(&biped), s->rf_pos.c.y );
  EXPECT_EQ( pdBipedRefRFPosZ(&biped), s->rf_pos.c.z );
  EXPECT_EQ( pdBipedRefRFAttX(&biped), s->rf_att.c.x );
  EXPECT_EQ( pdBipedRefRFAttY(&biped), s->rf_att.c.y );
  EXPECT_EQ( pdBipedRefRFAttZ(&biped), s->rf_att.c.z );
  pdCZ *cz = pdBipedCZPtr(&biped);
  EXPECT_EQ( pdCZRefVelX(cz), s->com_vel.c.x );
  EXPECT_EQ( pdCZRefVelY(cz), s->com_vel.c.y );
  EXPECT_EQ( pdCZRefVelZ(cz), s->com_vel.c.z );
  EXPECT_EQ( pdCZRefAccX(cz), s->com_acc.c.x );
  EXPECT_EQ( pdCZRefAccY(cz), s->com_acc.c.y );
  EXPECT_EQ( pdCZRefAccZ(cz), s->com_acc.c.z );
  EXPECT_EQ( pdCZRefZMPX(cz), s->zmp.c.x );
  EXPECT_EQ( pdCZRefZMPY(cz), s->zmp.c.y );
  EXPECT_EQ( pdCZRefZMPZ(cz), s->zmp.c.z );
  EXPECT_EQ( pdCZVrtRF(&cz->_vrt), s->fz );
}

TEST_F(pdBipedTest, SingleSupportPoseInit_ThrowException)
{
  zEchoOff();
  pdStateInit( &state );
  EXPECT_FALSE( pdBipedSingleSupportPoseInit( &biped, &state, PD_FOOT_RIGHT, 0.02 ) );
  zVec3DCreate( &state.lf_pos, 0,  0.1, 0.02 );
  zVec3DCreate( &state.rf_pos, 0, -0.1, 0 );
  zVec3DCreate( &state.com_pos, 0, -0.1, 0.1 );
  pdCmdDefaultInit( biped.cmd );
  EXPECT_FALSE( pdBipedSingleSupportPoseInit( &biped, &state, 5, 0.02 ) );
  EXPECT_FALSE( pdBipedSingleSupportPoseInit( &biped, &state, PD_FOOT_RIGHT, 0.001 ) );
  zEchoOn();
}

TEST_F(pdBipedTest, SingleSupportPoseInit)
{
  pdStateInit( &state );
  zVec3DCreate( &state.lf_pos, 0,  0.1, 0.02 );
  zVec3DCreate( &state.rf_pos, 0, -0.1, 0 );
  zVec3DCreate( &state.com_pos, 0.01, 0, 0.1 );
  pdCmdDefaultInit( biped.cmd );
  biped.cmd->dist = 0.2;
  EXPECT_TRUE( pdBipedSingleSupportPoseInit( &biped, &state, PD_FOOT_RIGHT, 0.02 ) );

  EXPECT_DOUBLE_EQ( 0.2, biped.cmd->dist );
  EXPECT_DOUBLE_EQ( 0.95*0.1, biped.cmd->zd );
  EXPECT_DOUBLE_EQ( -zPI_2, biped.cmd->thetad );
  // the desired COM is the stance-foot center, synced with the warp target
  EXPECT_DOUBLE_EQ( 0, biped.cmd->xd );
  EXPECT_DOUBLE_EQ( -0.1, biped.cmd->yd );
  EXPECT_DOUBLE_EQ( biped.cmd->xd, biped.cmd->xdd );
  EXPECT_DOUBLE_EQ( biped.cmd->yd, biped.cmd->ydd );
  EXPECT_DOUBLE_EQ( biped.cmd->zd, biped.cmd->zdd );

  EXPECT_DOUBLE_EQ( 0, pdBipedRefCOMPosX( &biped ) );
  EXPECT_DOUBLE_EQ( -0.1, pdBipedRefCOMPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0.95*0.1, pdBipedRefCOMPosZ( &biped ) );
  // stance (right) reference foot on the ground at its position
  EXPECT_DOUBLE_EQ( 0, pdBipedRefRFPosX( &biped ) );
  EXPECT_DOUBLE_EQ( -0.1, pdBipedRefRFPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefRFPosZ( &biped ) );
  // swing (left) reference foot hovering above its nominal offset
  EXPECT_NEAR( 0, pdBipedRefLFPosX( &biped ), GTEST_TOL );
  EXPECT_DOUBLE_EQ( 0.1, pdBipedRefLFPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0.02, pdBipedRefLFPosZ( &biped ) );
  // foot structures seeded at the reference poses
  EXPECT_DOUBLE_EQ( 0.02, pdFootRefPosZ( pdBipedLFPtr( &biped ) ) );
  EXPECT_DOUBLE_EQ( 0.02, pdFootDesPosZ( pdBipedLFPtr( &biped ) ) );
  EXPECT_DOUBLE_EQ( 0, pdFootPivotPosZ( pdBipedLFPtr( &biped ) ) );
  EXPECT_DOUBLE_EQ( 0.1, pdFootRefPosY( pdBipedLFPtr( &biped ) ) );
  EXPECT_DOUBLE_EQ( -0.1, pdFootRefPosY( pdBipedRFPtr( &biped ) ) );
  EXPECT_DOUBLE_EQ( 0, pdFootRefPosZ( pdBipedRFPtr( &biped ) ) );

  EXPECT_TRUE( biped.mode.balancing );
}

TEST_F(pdBipedTest, Update_BalancingFreezesCommandAndFeet)
{
  zVec3D vert[4];
  zLoop3DCell cell[8];
  int i;

  pdStateInit( &state );
  zVec3DCreate( &state.lf_pos, 0,  0.1, 0.02 );
  zVec3DCreate( &state.rf_pos, 0, -0.1, 0 );
  zVec3DCreate( &state.com_pos, 0, -0.1, 0.095 );
  pdCmdDefaultInit( biped.cmd );
  biped.cmd->dist = 0.2;
  ASSERT_TRUE( pdBipedSingleSupportPoseInit( &biped, &state, PD_FOOT_RIGHT, 0.02 ) );
  // square support region under the right stance foot
  zVec3DCreate( &vert[0], -0.05, -0.15, 0 );
  zVec3DCreate( &vert[1],  0.05, -0.15, 0 );
  zVec3DCreate( &vert[2],  0.05, -0.05, 0 );
  zVec3DCreate( &vert[3], -0.05, -0.05, 0 );
  for( i=0; i<4; i++ ){
    cell[i].data = &vert[i];
    zStackPush( &state.sr_rf, &cell[i] );
    cell[i+4].data = &vert[i];
    zStackPush( &state.sr, &cell[i+4] );
  }
  for( i=0; i<10; i++ )
    pdBipedUpdate( &biped, &state );
  EXPECT_TRUE( biped.mode.balancing );
  EXPECT_TRUE( biped.mode.standing );
  EXPECT_FALSE( biped.mode.stepping );
  // the desired COM stays at the stance-foot center
  EXPECT_DOUBLE_EQ( 0, biped.cmd->xd );
  EXPECT_DOUBLE_EQ( -0.1, biped.cmd->yd );
  // the internal rho follows the (zero) command
  EXPECT_DOUBLE_EQ( 0, pdCZRho( pdBipedCZPtr( &biped ) ) );
  // the swing-foot reference keeps hovering
  EXPECT_DOUBLE_EQ( 0.02, pdFootRefPosZ( pdBipedLFPtr( &biped ) ) );
}

TEST_F(pdBipedTest, Update_BalancingReleaseResumesStepping)
{
  zVec3D vert[4];
  zLoop3DCell cell[8];
  int i;

  pdStateInit( &state );
  zVec3DCreate( &state.lf_pos, 0,  0.1, 0.02 );
  zVec3DCreate( &state.rf_pos, 0, -0.1, 0 );
  zVec3DCreate( &state.com_pos, 0, -0.1, 0.095 );
  pdCmdDefaultInit( biped.cmd );
  biped.cmd->dist = 0.2;
  ASSERT_TRUE( pdBipedSingleSupportPoseInit( &biped, &state, PD_FOOT_RIGHT, 0.02 ) );
  zVec3DCreate( &vert[0], -0.05, -0.15, 0 );
  zVec3DCreate( &vert[1],  0.05, -0.15, 0 );
  zVec3DCreate( &vert[2],  0.05, -0.05, 0 );
  zVec3DCreate( &vert[3], -0.05, -0.05, 0 );
  for( i=0; i<4; i++ ){
    cell[i].data = &vert[i];
    zStackPush( &state.sr_rf, &cell[i] );
    cell[i+4].data = &vert[i];
    zStackPush( &state.sr, &cell[i+4] );
  }
  pdBipedUpdate( &biped, &state );
  ASSERT_TRUE( biped.mode.balancing );
  // commanding a step releases the balancing hold
  biped.cmd->rho = 1.0;
  pdBipedUpdate( &biped, &state );
  EXPECT_FALSE( biped.mode.balancing );
  EXPECT_TRUE( biped.mode.stepping );
  EXPECT_DOUBLE_EQ( 1.0, pdCZRho( pdBipedCZPtr( &biped ) ) );
}
