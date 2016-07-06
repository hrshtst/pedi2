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
