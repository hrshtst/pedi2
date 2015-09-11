#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_biped.h>

const double TIME_STEP = 0.01;
#define GTEST_TOL 1e-12
#define GTEST_TOL_LOOSE 1e-04

class pdBipedTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdBipedInit( &biped, &cmd, &state, TIME_STEP );
    destroy_flag = false;
  };
  virtual void TearDown() {
    if( !destroy_flag )
      pdBipedDestroy( &biped );
  };

  void SetRandomValues() {
    ri.SetRandVec3D( biped.ref_com_pos );
    ri.SetRandVec3D( biped.ref_base_att );
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
    pdFootSR( pdBipedLFPtr(&biped) ) = &biped.state->sr_lf;
    pdFootSR( pdBipedRFPtr(&biped) ) = &biped.state->sr_lf;
  };

  void SupportOnLeftFoot() {
    pdFootPosZ( pdBipedLFPtr(&biped) ) = 0.0;
    pdFootPosZ( pdBipedRFPtr(&biped) ) = 0.01;
    pdFootSR( pdBipedLFPtr(&biped) ) = &biped.state->sr_lf;
    pdFootSR( pdBipedRFPtr(&biped) ) = NULL;
  };

  void SupportOnRightFoot() {
    pdFootPosZ( pdBipedLFPtr(&biped) ) = 0.01;
    pdFootPosZ( pdBipedRFPtr(&biped) ) = 0.0;
    pdFootSR( pdBipedLFPtr(&biped) ) = NULL;
    pdFootSR( pdBipedRFPtr(&biped) ) = &biped.state->sr_lf;
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
  pdBipedInit( &biped, &cmd, &state, TIME_STEP );
  EXPECT_EQ( 0, pdBipedTime( &biped ) );
  EXPECT_EQ( TIME_STEP, pdBipedTimeStep( &biped ) );
  EXPECT_EQ( &cmd, pdBipedCmd( &biped ) );
  EXPECT_EQ( &state, pdBipedState( &biped ) );
  EXPECT_EQ( &biped.cz, pdBipedCZPtr( &biped ) );
  EXPECT_EQ( &biped.lf, pdBipedLFPtr( &biped ) );
  EXPECT_EQ( &biped.rf, pdBipedRFPtr( &biped ) );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  EXPECT_EQ( 0, pdBipedRefCOMPosX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefCOMPosY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefCOMPosZ( &biped ) );
  EXPECT_EQ( 0, pdBipedRefBaseAttX( &biped ) );
  EXPECT_EQ( 0, pdBipedRefBaseAttY( &biped ) );
  EXPECT_EQ( 0, pdBipedRefBaseAttZ( &biped ) );
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
  pdBipedUpdate( &biped );
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
  pdBipedUpdate( &biped );
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
  pdBipedUpdate( &biped );
  EXPECT_EQ( pdBipedTime( &biped ), pdCZTime( pdBipedCZPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedLFPtr( &biped ) ) );
  EXPECT_EQ( pdBipedTime( &biped ), pdFootTime( pdBipedRFPtr( &biped ) ) );
}

TEST_F(pdBipedTest, RefVec)
{
  EXPECT_EQ( &biped.ref_com_pos, pdBipedRefCOMPos( &biped ) );
  EXPECT_EQ( &biped.ref_base_att, pdBipedRefBaseAtt( &biped ) );
  EXPECT_EQ( &biped.ref_lf_pos, pdBipedRefLFPos( &biped ) );
  EXPECT_EQ( &biped.ref_lf_att, pdBipedRefLFAtt( &biped ) );
  EXPECT_EQ( &biped.ref_rf_pos, pdBipedRefRFPos( &biped ) );
  EXPECT_EQ( &biped.ref_rf_att, pdBipedRefRFAtt( &biped ) );
}

TEST_F(pdBipedTest, DefaultPoseInit_ThrowException)
{
  zEchoOff();
  pdStateInit( biped.state );
  EXPECT_FALSE( pdBipedDefaultPoseInit( &biped ) );
  zEchoOn();
}

TEST_F(pdBipedTest, DefaultPoseInit)
{
  pdStateInit( biped.state );
  zVec3DCreate( &biped.state->lf_pos, 0,  0.1, -0.1 );
  zVec3DCreate( &biped.state->rf_pos, 0, -0.1, -0.1 );
  zVec3DCreate( &biped.state->com_pos, 0.01, 0, 0.1 );
  pdCmdDefaultInit( biped.cmd );
  EXPECT_TRUE( pdBipedDefaultPoseInit( &biped ) );

  EXPECT_DOUBLE_EQ( 0.2, biped.cmd->dist );
  EXPECT_DOUBLE_EQ( 0.95*0.2, biped.cmd->zd );
  EXPECT_DOUBLE_EQ( -zPI_2, biped.cmd->thetad );
  EXPECT_DOUBLE_EQ( 0.01, biped.cmd->xd );
  EXPECT_DOUBLE_EQ( 0, biped.cmd->yd );

  EXPECT_DOUBLE_EQ( 0.01, pdBipedRefCOMPosX( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefCOMPosY( &biped ) );
  EXPECT_DOUBLE_EQ( 0.95*0.2, pdBipedRefCOMPosZ( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefBaseAttX( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefBaseAttY( &biped ) );
  EXPECT_DOUBLE_EQ( 0, pdBipedRefBaseAttZ( &biped ) );
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


TEST_F(pdBipedTest, InitMode)
{
  biped.mode.stand    = false;
  biped.mode.step     = true;
  biped.mode.walk     = true;
  biped.mode.sidewalk = true;
  biped.mode.follow   = true;
  biped.mode.brake    = true;
  pdBipedInitMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateMode_stand)
{
  pdBipedInitMode( &biped );

  SupportOnBothFeet();
  pdCmdDefaultInit( &cmd );
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );

  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );

  pdCmdDefaultInit( &cmd );
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );

  pdCmdDefaultInit( &cmd );
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );

  pdCmdDefaultInit( &cmd );
  cmd.rho = 1.0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );

  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateMode_step)
{
  pdBipedInitMode( &biped );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  // attempt to step, but not initiate yet
  cmd.rho = 1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_TRUE( biped.mode.stand );
  // initiate stepping
  cmd.rho = 1;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.stand );
  // still stepping
  cmd.rho = 1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.stand );
  // still stepping
  cmd.rho = 1;
  SupportOnRightFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.stand );
  // attempt to stop, but still continue
  cmd.rho = 0;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.stand );
  // stop stepping
  cmd.rho = 0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_TRUE( biped.mode.stand );

  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateMode_walk)
{
  pdBipedInitMode( &biped );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  // attempt to walk, but not initiate yet
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  // initiate walking
  cmd.vud = 0.1;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  // still walking
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  // still walking
  cmd.vud = 0.1;
  SupportOnRightFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  // attempt to stop, but still continue
  cmd.vud = 0;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  // stop stepping
  cmd.vud = 0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );

  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateMode_sidewalk)
{
  pdBipedInitMode( &biped );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // attempt to walk, but not initiate yet
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // initiate walking
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_TRUE( biped.mode.brake );
  // still walking
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // still walking
  cmd.vwd = 0.1;
  SupportOnRightFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_TRUE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  // attempt to stop, but still continue
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_TRUE( biped.mode.brake );
  // attempt to stop, but still continue
  cmd.vwd = 0.0;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_TRUE( biped.mode.brake );
  // stop stepping
  cmd.vwd = 0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );

  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateMode_diagonal)
{
  pdBipedInitMode( &biped );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // attempt to walk, but not initiate yet
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // initiate walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnRightFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // attempt to walk diagonally
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_TRUE( biped.mode.brake );
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnRightFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_TRUE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // attempt to walk longitudinally again
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnRightFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_TRUE( biped.mode.sidewalk );
  EXPECT_TRUE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // attempt to stop
  cmd.vud = 0.0;
  cmd.vwd = 0.0;
  SupportOnLeftFoot();
  pdBipedUpdateMode( &biped );
  EXPECT_FALSE( biped.mode.stand );
  EXPECT_TRUE( biped.mode.step );
  EXPECT_TRUE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );
  // stop stepping
  cmd.vud = 0.0;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdBipedUpdateMode( &biped );
  EXPECT_TRUE( biped.mode.stand );
  EXPECT_FALSE( biped.mode.step );
  EXPECT_FALSE( biped.mode.walk );
  EXPECT_FALSE( biped.mode.sidewalk );
  EXPECT_FALSE( biped.mode.follow );
  EXPECT_FALSE( biped.mode.brake );

  destroy_flag = true;
}

TEST_F(pdBipedTest, UpdateState)
{
  SetRandomValues();
  pdBipedUpdateState( &biped );
  pdState *s = biped.state;
  EXPECT_EQ( pdBipedRefCOMPosX(&biped), zVec3DElem(&s->com_pos,zX) );
  EXPECT_EQ( pdBipedRefCOMPosY(&biped), zVec3DElem(&s->com_pos,zY) );
  EXPECT_EQ( pdBipedRefCOMPosZ(&biped), zVec3DElem(&s->com_pos,zZ) );
  EXPECT_EQ( pdBipedRefBaseAttX(&biped), zVec3DElem(&s->base_att,zX) );
  EXPECT_EQ( pdBipedRefBaseAttY(&biped), zVec3DElem(&s->base_att,zY) );
  EXPECT_EQ( pdBipedRefBaseAttZ(&biped), zVec3DElem(&s->base_att,zZ) );
  EXPECT_EQ( pdBipedRefLFPosX(&biped), zVec3DElem(&s->lf_pos,zX) );
  EXPECT_EQ( pdBipedRefLFPosY(&biped), zVec3DElem(&s->lf_pos,zY) );
  EXPECT_EQ( pdBipedRefLFPosZ(&biped), zVec3DElem(&s->lf_pos,zZ) );
  EXPECT_EQ( pdBipedRefLFAttX(&biped), zVec3DElem(&s->lf_att,zX) );
  EXPECT_EQ( pdBipedRefLFAttY(&biped), zVec3DElem(&s->lf_att,zY) );
  EXPECT_EQ( pdBipedRefLFAttZ(&biped), zVec3DElem(&s->lf_att,zZ) );
  EXPECT_EQ( pdBipedRefRFPosX(&biped), zVec3DElem(&s->rf_pos,zX) );
  EXPECT_EQ( pdBipedRefRFPosY(&biped), zVec3DElem(&s->rf_pos,zY) );
  EXPECT_EQ( pdBipedRefRFPosZ(&biped), zVec3DElem(&s->rf_pos,zZ) );
  EXPECT_EQ( pdBipedRefRFAttX(&biped), zVec3DElem(&s->rf_att,zX) );
  EXPECT_EQ( pdBipedRefRFAttY(&biped), zVec3DElem(&s->rf_att,zY) );
  EXPECT_EQ( pdBipedRefRFAttZ(&biped), zVec3DElem(&s->rf_att,zZ) );
  pdCZ *cz = pdBipedCZPtr(&biped);
  EXPECT_EQ( pdCZRefVelX(cz), zVec3DElem(&s->com_vel,zX) );
  EXPECT_EQ( pdCZRefVelY(cz), zVec3DElem(&s->com_vel,zY) );
  EXPECT_EQ( pdCZRefVelZ(cz), zVec3DElem(&s->com_vel,zZ) );
  EXPECT_EQ( pdCZRefAccX(cz), zVec3DElem(&s->com_acc,zX) );
  EXPECT_EQ( pdCZRefAccY(cz), zVec3DElem(&s->com_acc,zY) );
  EXPECT_EQ( pdCZRefAccZ(cz), zVec3DElem(&s->com_acc,zZ) );
  EXPECT_EQ( pdCZRefZMPX(cz), zVec3DElem(&s->zmp,zX) );
  EXPECT_EQ( pdCZRefZMPY(cz), zVec3DElem(&s->zmp,zY) );
  EXPECT_EQ( pdCZRefZMPZ(cz), zVec3DElem(&s->zmp,zZ) );
  EXPECT_EQ( pdCZVrtRF(&cz->_vrt), s->fz );
}
