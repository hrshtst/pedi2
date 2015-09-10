#include "gtest/gtest.h"
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

  void LoadMighty() {
    char model[] = "model/mighty.zkc";

    pdCmdDefaultInit( &cmd );
    pdCoreLoad( &core, model );
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

  bool destroy_flag;
  pdCore core;
  pdCmd cmd;
};

TEST_F(pdCoreTest, Init)
{
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
  LoadMighty();
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
