#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_mode.h>

class pdModeTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCmdInit( &cmd );
    pdStateInit( &state );
    pdModeInit( &mode );
  };
  virtual void TearDown() {
    pdModeDestroy( &mode );
    pdStateDestroy( &state );
    pdCmdDestroy( &cmd );
  };

  void SetRandomValues() {
    ri.SetRandBool( mode.standing );
    ri.SetRandBool( mode.trymove );
    ri.SetRandBool( mode.stepping );
    ri.SetRandBool( mode.walking );
    ri.SetRandBool( mode.sideways );
    ri.SetRandBool( mode.following );
    ri.SetRandBool( mode.braking );
    ri.SetRandBool( mode.rotating );
  };

  void LFOn() {
    zVec3DSetElem( &state.lf_pos, zZ, 0.0 );
    if( zListNum( &state.sr_lf) == 0 )
      zStackPush( &state.sr_lf, &cell[0] );
  };

  void LFOff() {
    zVec3DListCell *cp;

    zVec3DSetElem( &state.lf_pos, zZ, 0.01 );
    if( zListNum( &state.sr_lf) > 0 )
      zStackPop( &state.sr_lf, &cp );
  };

  void RFOn() {
    zVec3DSetElem( &state.rf_pos, zZ, 0.0 );
    if( zListNum( &state.sr_rf) == 0 )
      zStackPush( &state.sr_rf, &cell[1] );
  };

  void RFOff() {
    zVec3DListCell *cp;

    zVec3DSetElem( &state.rf_pos, zZ, 0.01 );
    if( zListNum( &state.sr_rf) > 0 )
      zStackPop( &state.sr_rf, &cp );
  };

  void SupportOnBothFeet() {
    LFOn();
    RFOn();
  };

  void SupportOnLeftFoot() {
    LFOn();
    RFOff();
  };

  void SupportOnRightFoot() {
    LFOff();
    RFOn();
  };

  RandomInitializer ri;
  pdMode mode;
  pdState state;
  pdCmd cmd;
  zVec3DListCell cell[2];
};

TEST_F(pdModeTest, Init)
{
  SetRandomValues();
  pdModeInit( &mode );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.trymove );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  EXPECT_FALSE( mode.rotating );
}

TEST_F(pdModeTest, Destroy)
{
  SetRandomValues();
  pdModeDestroy( &mode );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.trymove );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  EXPECT_FALSE( mode.rotating );
}

TEST_F(pdModeTest, Update_trymove)
{
  SupportOnBothFeet();

  pdCmdDefaultInit( &cmd );
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.trymove );

  pdCmdDefaultInit( &cmd );
  cmd.rho = 1;
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_TRUE( mode.trymove );

  pdCmdDefaultInit( &cmd );
  cmd.vud = 0.1;
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_TRUE( mode.trymove );

  pdCmdDefaultInit( &cmd );
  cmd.vwd = 0.1;
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_TRUE( mode.trymove );

  pdCmdDefaultInit( &cmd );
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.trymove );
}

TEST_F(pdModeTest, Update_stand)
{
  SupportOnBothFeet();
  pdCmdDefaultInit( &cmd );
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );

  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );

  pdCmdDefaultInit( &cmd );
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );

  pdCmdDefaultInit( &cmd );
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );

  pdCmdDefaultInit( &cmd );
  cmd.rho = 1.0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
}

TEST_F(pdModeTest, Update_step)
{
  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  // attempt to step, but not initiate yet
  cmd.rho = 1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.stepping );
  EXPECT_TRUE( mode.standing );
  // initiate stepping
  cmd.rho = 1;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.standing );
  // still stepping
  cmd.rho = 1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.standing );
  // still stepping
  cmd.rho = 1;
  SupportOnRightFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.standing );
  // attempt to stop, but still continue
  cmd.rho = 0;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.standing );
  // stop stepping
  cmd.rho = 0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.stepping );
  EXPECT_TRUE( mode.standing );
}

TEST_F(pdModeTest, Update_walk)
{
  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  // attempt to walk, but not initiate yet
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  // initiate walking
  cmd.vud = 0.1;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  // still walking
  cmd.vud = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  // still walking
  cmd.vud = 0.1;
  SupportOnRightFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  // attempt to stop, but still continue
  cmd.vud = 0;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  // stop stepping
  cmd.vud = 0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
}

TEST_F(pdModeTest, Update_sideways)
{
  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // attempt to walk, but not initiate yet
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // initiate walking
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_TRUE( mode.braking );
  // still walking
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // still walking
  cmd.vwd = 0.1;
  SupportOnRightFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_TRUE( mode.following );
  EXPECT_FALSE( mode.braking );
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  // attempt to stop, but still continue
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_TRUE( mode.braking );
  // attempt to stop, but still continue
  cmd.vwd = 0.0;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_TRUE( mode.braking );
  // stop stepping
  cmd.vwd = 0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
}

TEST_F(pdModeTest, Update_diagonal)
{
  pdCmdDefaultInit( &cmd );
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // attempt to walk, but not initiate yet
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // initiate walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnRightFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // attempt to walk diagonally
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_TRUE( mode.braking );
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.1;
  SupportOnRightFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_TRUE( mode.following );
  EXPECT_FALSE( mode.braking );
  // attempt to walk longitudinally again
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnRightFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_TRUE( mode.sideways );
  EXPECT_TRUE( mode.following );
  EXPECT_FALSE( mode.braking );
  // still walking
  cmd.vud = 0.1;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // attempt to stop
  cmd.vud = 0.0;
  cmd.vwd = 0.0;
  SupportOnLeftFoot();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_FALSE( mode.standing );
  EXPECT_TRUE( mode.stepping );
  EXPECT_TRUE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  // stop stepping
  cmd.vud = 0.0;
  cmd.vwd = 0.0;
  SupportOnBothFeet();
  pdModeUpdate( &mode, &cmd, &state );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
}
