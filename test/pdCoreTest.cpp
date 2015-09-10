#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_core.h>

const double TIME_STEP = 0.01;
#define GTEST_TOL 1e-12
#define GTEST_TOL_LOOSE 1e-04

class pdCoreTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCoreInit( &core, &cmd, TIME_STEP );
    destroy_flag = false;
  };
  virtual void TearDown() {
    if( !destroy_flag )
      pdCoreDestroy( &core );
  };

  void SetRandomValues() {
    ri.SetRandVec3D( core.ref_com_pos );
    ri.SetRandVec3D( core.ref_base_att );
    ri.SetRandVec3D( core.ref_lf_pos );
    ri.SetRandVec3D( core.ref_lf_att );
    ri.SetRandVec3D( core.ref_rf_pos );
    ri.SetRandVec3D( core.ref_rf_att );
    ri.SetRandVec3D( pdCoreCZPtr(&core)->refvel );
    ri.SetRandVec3D( pdCoreCZPtr(&core)->refacc );
    ri.SetRandVec3D( pdCoreCZPtr(&core)->refzmp );
    ri.SetRandScalar( pdCZVrtRF( &core.cz._vrt ) );
  };

  void SupportOnBothFeet() {
    pdFootPosZ( pdCoreLFPtr(&core) ) = 0.0;
    pdFootPosZ( pdCoreRFPtr(&core) ) = 0.0;
    pdFootSR( pdCoreLFPtr(&core) ) = &core.state.sr_lf;
    pdFootSR( pdCoreRFPtr(&core) ) = &core.state.sr_lf;
  };

  void SupportOnLeftFoot() {
    pdFootPosZ( pdCoreLFPtr(&core) ) = 0.0;
    pdFootPosZ( pdCoreRFPtr(&core) ) = 0.01;
    pdFootSR( pdCoreLFPtr(&core) ) = &core.state.sr_lf;
    pdFootSR( pdCoreRFPtr(&core) ) = NULL;
  };

  void SupportOnRightFoot() {
    pdFootPosZ( pdCoreLFPtr(&core) ) = 0.01;
    pdFootPosZ( pdCoreRFPtr(&core) ) = 0.0;
    pdFootSR( pdCoreLFPtr(&core) ) = NULL;
    pdFootSR( pdCoreRFPtr(&core) ) = &core.state.sr_lf;
  };

  RandomInitializer ri;
  bool destroy_flag;
  pdCore core;
  pdCmd cmd;
};

TEST_F(pdCoreTest, Init)
{
  SetRandomValues();
  pdCoreInit( &core, &cmd, TIME_STEP );
  EXPECT_EQ( 0, pdCoreTime( &core ) );
  EXPECT_EQ( TIME_STEP, pdCoreTimeStep( &core ) );
  EXPECT_EQ( &cmd, pdCoreCmd( &core ) );
  EXPECT_EQ( &core.cz, pdCoreCZPtr( &core ) );
  EXPECT_EQ( &core.lf, pdCoreLFPtr( &core ) );
  EXPECT_EQ( &core.rf, pdCoreRFPtr( &core ) );
  EXPECT_EQ( &core.state, pdCoreStatePtr( &core ) );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  EXPECT_EQ( 0, pdCoreRefCOMPosX( &core ) );
  EXPECT_EQ( 0, pdCoreRefCOMPosY( &core ) );
  EXPECT_EQ( 0, pdCoreRefCOMPosZ( &core ) );
  EXPECT_EQ( 0, pdCoreRefBaseAttX( &core ) );
  EXPECT_EQ( 0, pdCoreRefBaseAttY( &core ) );
  EXPECT_EQ( 0, pdCoreRefBaseAttZ( &core ) );
  EXPECT_EQ( 0, pdCoreRefLFPosX( &core ) );
  EXPECT_EQ( 0, pdCoreRefLFPosY( &core ) );
  EXPECT_EQ( 0, pdCoreRefLFPosZ( &core ) );
  EXPECT_EQ( 0, pdCoreRefLFAttX( &core ) );
  EXPECT_EQ( 0, pdCoreRefLFAttY( &core ) );
  EXPECT_EQ( 0, pdCoreRefLFAttZ( &core ) );
  EXPECT_EQ( 0, pdCoreRefRFPosX( &core ) );
  EXPECT_EQ( 0, pdCoreRefRFPosY( &core ) );
  EXPECT_EQ( 0, pdCoreRefRFPosZ( &core ) );
  EXPECT_EQ( 0, pdCoreRefRFAttX( &core ) );
  EXPECT_EQ( 0, pdCoreRefRFAttY( &core ) );
  EXPECT_EQ( 0, pdCoreRefRFAttZ( &core ) );
}

TEST_F(pdCoreTest, Destroy)
{
  pdCoreDestroy( &core );
  EXPECT_EQ( 0, pdCoreTime( &core ) );
  EXPECT_EQ( 0, pdCoreTimeStep( &core ) );
  EXPECT_EQ( NULL, pdCoreCmd( &core ) );
  destroy_flag = true;
}

TEST_F(pdCoreTest, SetTime)
{
  pdCoreSetTime( &core, 10 );
  EXPECT_EQ( 10, pdCoreTime( &core ) );
  EXPECT_EQ( 10, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 10, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 10, pdFootTime( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, ResetTime)
{
  pdCoreSetTime( &core, 5 );
  EXPECT_EQ( 5, pdCoreTime( &core ) );
  EXPECT_EQ( 5, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 5, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 5, pdFootTime( pdCoreRFPtr( &core ) ) );
  pdCoreResetTime( &core );
  EXPECT_EQ( 0, pdCoreTime( &core ) );
  EXPECT_EQ( 0, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 0, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 0, pdFootTime( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, SetTimeStep)
{
  pdCoreSetTimeStep( &core, 0.005 );
  EXPECT_EQ( 0.005, pdCoreTimeStep( &core ) );
  EXPECT_EQ( 0.005, pdCZTimeStep( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 0.005, pdFootTimeStep( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 0.005, pdFootTimeStep( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, IncrTime)
{
  pdCoreIncrTime( &core );
  EXPECT_EQ( TIME_STEP, pdCoreTime( &core ) );
  pdCoreIncrTime( &core );
  EXPECT_EQ( 2*TIME_STEP, pdCoreTime( &core ) );
}

TEST_F(pdCoreTest, IncrTime_Update)
{
  EXPECT_EQ( pdCoreTime( &core ), pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreRFPtr( &core ) ) );
  pdCoreUpdate( &core );
  EXPECT_EQ( pdCoreTime( &core ), pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreRFPtr( &core ) ) );
  pdCoreUpdate( &core );
  EXPECT_EQ( pdCoreTime( &core ), pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreRFPtr( &core ) ) );
  pdCoreUpdate( &core );
  EXPECT_EQ( pdCoreTime( &core ), pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( pdCoreTime( &core ), pdFootTime( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, RefVec)
{
  EXPECT_EQ( &core.ref_com_pos, pdCoreRefCOMPos( &core ) );
  EXPECT_EQ( &core.ref_base_att, pdCoreRefBaseAtt( &core ) );
  EXPECT_EQ( &core.ref_lf_pos, pdCoreRefLFPos( &core ) );
  EXPECT_EQ( &core.ref_lf_att, pdCoreRefLFAtt( &core ) );
  EXPECT_EQ( &core.ref_rf_pos, pdCoreRefRFPos( &core ) );
  EXPECT_EQ( &core.ref_rf_att, pdCoreRefRFAtt( &core ) );
}

TEST_F(pdCoreTest, DefaultPoseInit_ThrowException)
{
  zEchoOff();
  pdStateInit( &core.state );
  EXPECT_FALSE( pdCoreDefaultPoseInit( &core ) );
  zEchoOn();
}

TEST_F(pdCoreTest, DefaultPoseInit)
{
  pdStateInit( &core.state );
  zVec3DCreate( &core.state.lf_pos, 0,  0.1, -0.1 );
  zVec3DCreate( &core.state.rf_pos, 0, -0.1, -0.1 );
  zVec3DCreate( &core.state.com_pos, 0.01, 0, 0.1 );
  pdCmdDefaultInit( core.cmd );
  EXPECT_TRUE( pdCoreDefaultPoseInit( &core ) );

  EXPECT_DOUBLE_EQ( 0.2, core.cmd->dist );
  EXPECT_DOUBLE_EQ( 0.95*0.2, core.cmd->zd );
  EXPECT_DOUBLE_EQ( -zPI_2, core.cmd->thetad );
  EXPECT_DOUBLE_EQ( 0.01, core.cmd->xd );
  EXPECT_DOUBLE_EQ( 0, core.cmd->yd );

  EXPECT_DOUBLE_EQ( 0.01, pdCoreRefCOMPosX( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefCOMPosY( &core ) );
  EXPECT_DOUBLE_EQ( 0.95*0.2, pdCoreRefCOMPosZ( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefBaseAttX( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefBaseAttY( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefBaseAttZ( &core ) );
  EXPECT_DOUBLE_EQ( 0.01, pdCoreRefLFPosX( &core ) );
  EXPECT_DOUBLE_EQ( 0.1, pdCoreRefLFPosY( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefLFPosZ( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefLFAttX( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefLFAttY( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefLFAttZ( &core ) );
  EXPECT_DOUBLE_EQ( 0.01, pdCoreRefRFPosX( &core ) );
  EXPECT_DOUBLE_EQ( -0.1, pdCoreRefRFPosY( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefRFPosZ( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefRFAttX( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefRFAttY( &core ) );
  EXPECT_DOUBLE_EQ( 0, pdCoreRefRFAttZ( &core ) );
}

#if 0
TEST_F(pdCoreTest, Load)
{
  char filename[] = "model/mighty.zkc";

  pdCoreInit( &core, &cmd, TIME_STEP );
  pdCoreLoad( &core, filename );
  EXPECT_EQ( 25, (int)rkChainNum(pdRobotChainPtr(pdCoreRobotPtr(&core))));
}

TEST_F(pdCoreTest, JointSize)
{
  char model[] = "model/mighty.zkc";

  pdCoreLoad( &core, model );
  EXPECT_EQ( 26, pdCoreJointSize( &core ) );
}

TEST_F(pdCoreTest, JointDis)
{
  char model[] = "model/mighty.zkc";

  pdCoreLoad( &core, model );
  EXPECT_EQ( 26, pdCoreJointSize( &core ) );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 0 ),
               zVecElem( pdCoreJointDis( &core ), 0 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 1 ),
               zVecElem( pdCoreJointDis( &core ), 1 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 2 ),
               zVecElem( pdCoreJointDis( &core ), 2 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 10 ),
               zVecElem( pdCoreJointDis( &core ), 10 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 20 ),
               zVecElem( pdCoreJointDis( &core ), 20 ), GTEST_TOL );
}
#endif

TEST_F(pdCoreTest, DoesIntendToStand_Step_Walk)
{
  pdCmdDefaultInit( &cmd );

  EXPECT_TRUE( pdCoreDoesIntendToStand( &core ) );
  EXPECT_FALSE( pdCoreDoesIntendToStep( &core ) );
  EXPECT_FALSE( pdCoreDoesIntendToWalk( &core ) );

  cmd.rho = 1.0;
  EXPECT_FALSE( pdCoreDoesIntendToStand( &core ) );
  EXPECT_TRUE( pdCoreDoesIntendToStep( &core ) );
  EXPECT_FALSE( pdCoreDoesIntendToWalk( &core ) );

  cmd.rho = 1.0;
  cmd.vud = 0.1;
  EXPECT_FALSE( pdCoreDoesIntendToStand( &core ) );
  EXPECT_TRUE( pdCoreDoesIntendToStep( &core ) );
  EXPECT_TRUE( pdCoreDoesIntendToWalk( &core ) );

  destroy_flag = true;
}

TEST_F(pdCoreTest, DoesIntendToStand_Step_Sidewalk)
{
  pdCmdDefaultInit( &cmd );

  EXPECT_TRUE( pdCoreDoesIntendToStand( &core ) );
  EXPECT_FALSE( pdCoreDoesIntendToStep( &core ) );
  EXPECT_FALSE( pdCoreDoesIntendToSidewalk( &core ) );

  cmd.rho = 1.0;
  EXPECT_FALSE( pdCoreDoesIntendToStand( &core ) );
  EXPECT_TRUE( pdCoreDoesIntendToStep( &core ) );
  EXPECT_FALSE( pdCoreDoesIntendToSidewalk( &core ) );

  cmd.rho = 1.0;
  cmd.vwd = 0.1;
  EXPECT_FALSE( pdCoreDoesIntendToStand( &core ) );
  EXPECT_TRUE( pdCoreDoesIntendToStep( &core ) );
  EXPECT_TRUE( pdCoreDoesIntendToSidewalk( &core ) );

  destroy_flag = true;
}

TEST_F(pdCoreTest, BothFeetOn)
{
  SupportOnBothFeet();
  EXPECT_TRUE( pdCoreIsBothFeetOn( &core ) );
  EXPECT_TRUE( pdCoreIsEitherFootOn( &core ) );
  EXPECT_FALSE( pdCoreIsEitherFootOff( &core ) );

  SupportOnLeftFoot();
  EXPECT_FALSE( pdCoreIsBothFeetOn( &core ) );
  EXPECT_TRUE( pdCoreIsEitherFootOn( &core ) );
  EXPECT_TRUE( pdCoreIsEitherFootOff( &core ) );

  SupportOnRightFoot();
  EXPECT_FALSE( pdCoreIsBothFeetOn( &core ) );
  EXPECT_TRUE( pdCoreIsEitherFootOn( &core ) );
  EXPECT_TRUE( pdCoreIsEitherFootOff( &core ) );

  destroy_flag = true;
}


TEST_F(pdCoreTest, InitMode)
{
  core.mode.stand    = false;
  core.mode.step     = true;
  core.mode.walk     = true;
  core.mode.sidewalk = true;
  core.mode.follow   = true;
  core.mode.brake    = true;
  pdCoreInitMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  destroy_flag = true;
}

TEST_F(pdCoreTest, UpdateMode_stand)
{
  pdCoreInitMode( &core );

  SupportOnBothFeet();
  pdCmdDefaultInit( &cmd );
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );

  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );

  pdCmdDefaultInit( &cmd );
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );

  pdCmdDefaultInit( &cmd );
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );

  pdCmdDefaultInit( &cmd );
  cmd.rho = 1.0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );

  destroy_flag = true;
}

TEST_F(pdCoreTest, UpdateMode_step)
{
  pdCoreInitMode( &core );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  // attempt to step, but not initiate yet
  cmd.rho = 1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.step );
  EXPECT_TRUE( core.mode.stand );
  // initiate stepping
  cmd.rho = 1;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.stand );
  // still stepping
  cmd.rho = 1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.stand );
  // still stepping
  cmd.rho = 1;
  SupportOnRightFoot();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.stand );
  // attempt to stop, but still continue
  cmd.rho = 0;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.stand );
  // stop stepping
  cmd.rho = 0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.step );
  EXPECT_TRUE( core.mode.stand );

  destroy_flag = true;
}

TEST_F(pdCoreTest, UpdateMode_walk)
{
  pdCoreInitMode( &core );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  // attempt to walk, but not initiate yet
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  // initiate walking
  cmd.vud = 0.1;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  // still walking
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  // still walking
  cmd.vud = 0.1;
  SupportOnRightFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  // attempt to stop, but still continue
  cmd.vud = 0;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  // stop stepping
  cmd.vud = 0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );

  destroy_flag = true;
}

TEST_F(pdCoreTest, UpdateMode_sidewalk)
{
  pdCoreInitMode( &core );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // attempt to walk, but not initiate yet
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // initiate walking
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_TRUE( core.mode.brake );
  // still walking
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // still walking
  cmd.vwd = 0.1;
  SupportOnRightFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_TRUE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  // attempt to stop, but still continue
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_TRUE( core.mode.brake );
  // attempt to stop, but still continue
  cmd.vwd = 0.0;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_TRUE( core.mode.brake );
  // stop stepping
  cmd.vwd = 0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );

  destroy_flag = true;
}

TEST_F(pdCoreTest, UpdateMode_diagonal)
{
  pdCoreInitMode( &core );

  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // attempt to walk, but not initiate yet
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // initiate walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnRightFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // attempt to walk diagonally
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_TRUE( core.mode.brake );
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnRightFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_TRUE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // attempt to walk longitudinally again
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnRightFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_TRUE( core.mode.sidewalk );
  EXPECT_TRUE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // attempt to stop
  cmd.vud = 0.0;
  cmd.vwd = 0.0;
  SupportOnLeftFoot();
  pdCoreUpdateMode( &core );
  EXPECT_FALSE( core.mode.stand );
  EXPECT_TRUE( core.mode.step );
  EXPECT_TRUE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );
  // stop stepping
  cmd.vud = 0.0;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdCoreUpdateMode( &core );
  EXPECT_TRUE( core.mode.stand );
  EXPECT_FALSE( core.mode.step );
  EXPECT_FALSE( core.mode.walk );
  EXPECT_FALSE( core.mode.sidewalk );
  EXPECT_FALSE( core.mode.follow );
  EXPECT_FALSE( core.mode.brake );

  destroy_flag = true;
}

TEST_F(pdCoreTest, UpdateState)
{
  SetRandomValues();
  pdCoreUpdateState( &core );
  pdState *s = &core.state;
  EXPECT_EQ( pdCoreRefCOMPosX(&core), zVec3DElem(&s->com_pos,zX) );
  EXPECT_EQ( pdCoreRefCOMPosY(&core), zVec3DElem(&s->com_pos,zY) );
  EXPECT_EQ( pdCoreRefCOMPosZ(&core), zVec3DElem(&s->com_pos,zZ) );
  EXPECT_EQ( pdCoreRefBaseAttX(&core), zVec3DElem(&s->base_att,zX) );
  EXPECT_EQ( pdCoreRefBaseAttY(&core), zVec3DElem(&s->base_att,zY) );
  EXPECT_EQ( pdCoreRefBaseAttZ(&core), zVec3DElem(&s->base_att,zZ) );
  EXPECT_EQ( pdCoreRefLFPosX(&core), zVec3DElem(&s->lf_pos,zX) );
  EXPECT_EQ( pdCoreRefLFPosY(&core), zVec3DElem(&s->lf_pos,zY) );
  EXPECT_EQ( pdCoreRefLFPosZ(&core), zVec3DElem(&s->lf_pos,zZ) );
  EXPECT_EQ( pdCoreRefLFAttX(&core), zVec3DElem(&s->lf_att,zX) );
  EXPECT_EQ( pdCoreRefLFAttY(&core), zVec3DElem(&s->lf_att,zY) );
  EXPECT_EQ( pdCoreRefLFAttZ(&core), zVec3DElem(&s->lf_att,zZ) );
  EXPECT_EQ( pdCoreRefRFPosX(&core), zVec3DElem(&s->rf_pos,zX) );
  EXPECT_EQ( pdCoreRefRFPosY(&core), zVec3DElem(&s->rf_pos,zY) );
  EXPECT_EQ( pdCoreRefRFPosZ(&core), zVec3DElem(&s->rf_pos,zZ) );
  EXPECT_EQ( pdCoreRefRFAttX(&core), zVec3DElem(&s->rf_att,zX) );
  EXPECT_EQ( pdCoreRefRFAttY(&core), zVec3DElem(&s->rf_att,zY) );
  EXPECT_EQ( pdCoreRefRFAttZ(&core), zVec3DElem(&s->rf_att,zZ) );
  pdCZ *cz = pdCoreCZPtr(&core);
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
