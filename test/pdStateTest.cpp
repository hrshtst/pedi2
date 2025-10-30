#include "gtest/gtest.h"
#include <pedi2/pd_state.h>

class pdStateTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdStateInit( &state );
  };
  virtual void TearDown() {
    pdStateDestroy( &state );
  };

  void LFOn() {
    state.lf_pos.c.z = 0.0;
    if( zListSize( &state.sr_lf) == 0 )
      zStackPush( &state.sr_lf, &cell[0] );
  };

  void LFOff() {
    zLoop3DCell *cp;

    state.lf_pos.c.z = 0.01;
    if( zListSize( &state.sr_lf) > 0 )
      zStackPop( &state.sr_lf, &cp );
  };

  void RFOn() {
    state.rf_pos.c.z = 0.0;
    if( zListSize( &state.sr_rf) == 0 )
      zStackPush( &state.sr_rf, &cell[1] );
  };

  void RFOff() {
    zLoop3DCell *cp;

    state.rf_pos.c.z = 0.01;
    if( zListSize( &state.sr_rf) > 0 )
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

  pdState state;
  zLoop3DCell cell[2];
};

TEST_F(pdStateTest, Init)
{
  EXPECT_TRUE( zVec3DIsTiny( &state.com_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.com_vel ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.com_acc ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lh_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rh_pos ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.torso_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.lf_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.rf_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.lh_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.rh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.torso_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lf_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rf_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.zmp ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.deszmp ) );
  EXPECT_EQ( 0, state.fz );
  EXPECT_TRUE( zVec3DIsTiny( &state.ef ) );
  EXPECT_EQ( 0, zListSize( &state.sr_lf ) );
  EXPECT_EQ( 0, zListSize( &state.sr_rf ) );
  EXPECT_EQ( 0, zListSize( &state.sr ) );
}

TEST_F(pdStateTest, Destroy)
{
  EXPECT_TRUE( zVec3DIsTiny( &state.com_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.com_vel ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.com_acc ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lh_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rh_pos ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.torso_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.lf_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.rf_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.lh_att ) );
  // EXPECT_TRUE( zMat3DIsTiny( &state.rh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.torso_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lf_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rf_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.zmp ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.deszmp ) );
  EXPECT_EQ( 0, state.fz );
  EXPECT_TRUE( zVec3DIsTiny( &state.ef ) );
  EXPECT_EQ( 0, zListSize( &state.sr_lf ) );
  EXPECT_EQ( 0, zListSize( &state.sr_rf ) );
  EXPECT_EQ( 0, zListSize( &state.sr ) );
}

TEST_F(pdStateTest, FootIsOn)
{
  // on left foot
  SupportOnLeftFoot();
  EXPECT_TRUE( pdStateLFIsOn( &state ) );
  EXPECT_FALSE( pdStateLFIsOff( &state ) );
  EXPECT_FALSE( pdStateRFIsOn( &state ) );
  EXPECT_TRUE( pdStateRFIsOff( &state ) );

  // on right foot
  SupportOnRightFoot();
  EXPECT_FALSE( pdStateLFIsOn( &state ) );
  EXPECT_TRUE( pdStateLFIsOff( &state ) );
  EXPECT_TRUE( pdStateRFIsOn( &state ) );
  EXPECT_FALSE( pdStateRFIsOff( &state ) );

  // on both feet
  SupportOnBothFeet();
  EXPECT_TRUE( pdStateLFIsOn( &state ) );
  EXPECT_FALSE( pdStateLFIsOff( &state ) );
  EXPECT_TRUE( pdStateRFIsOn( &state ) );
  EXPECT_FALSE( pdStateRFIsOff( &state ) );
}

TEST_F(pdStateTest, BothFeetOn)
{
  // on left foot
  SupportOnLeftFoot();
  EXPECT_TRUE( pdStateEitherFootOn( &state ) );
  EXPECT_TRUE( pdStateEitherFootOff( &state ) );
  EXPECT_FALSE( pdStateBothFeetOn( &state ) );

  // on right foot
  SupportOnRightFoot();
  EXPECT_TRUE( pdStateEitherFootOn( &state ) );
  EXPECT_TRUE( pdStateEitherFootOff( &state ) );
  EXPECT_FALSE( pdStateBothFeetOn( &state ) );

  // on both feet
  SupportOnBothFeet();
  EXPECT_TRUE( pdStateEitherFootOn( &state ) );
  EXPECT_FALSE( pdStateEitherFootOff( &state ) );
  EXPECT_TRUE( pdStateBothFeetOn( &state ) );
}

TEST_F(pdStateTest, FFOn)
{
  SupportOnBothFeet();
  EXPECT_TRUE( pdStateFFOn( &state, 0.1 ) );
  EXPECT_FALSE( pdStateFFOff( &state, 0.1 ) );
  EXPECT_TRUE( pdStateFFOn( &state, -0.1 ) );
  EXPECT_FALSE( pdStateFFOff( &state, -0.1 ) );

  SupportOnLeftFoot();
  EXPECT_FALSE( pdStateFFOn( &state, 0.1 ) );
  EXPECT_TRUE( pdStateFFOff( &state, 0.1 ) );
  EXPECT_TRUE( pdStateFFOn( &state, -0.1 ) );
  EXPECT_FALSE( pdStateFFOff( &state, -0.1 ) );

  SupportOnRightFoot();
  EXPECT_TRUE( pdStateFFOn( &state, 0.1 ) );
  EXPECT_FALSE( pdStateFFOff( &state, 0.1 ) );
  EXPECT_FALSE( pdStateFFOn( &state, -0.1 ) );
  EXPECT_TRUE( pdStateFFOff( &state, -0.1 ) );
}

TEST_F(pdStateTest, BFOn)
{
  SupportOnBothFeet();
  EXPECT_TRUE( pdStateBFOn( &state, 0.1 ) );
  EXPECT_FALSE( pdStateBFOff( &state, 0.1 ) );
  EXPECT_TRUE( pdStateBFOn( &state, -0.1 ) );
  EXPECT_FALSE( pdStateBFOff( &state, -0.1 ) );

  SupportOnLeftFoot();
  EXPECT_TRUE( pdStateBFOn( &state, 0.1 ) );
  EXPECT_FALSE( pdStateBFOff( &state, 0.1 ) );
  EXPECT_FALSE( pdStateBFOn( &state, -0.1 ) );
  EXPECT_TRUE( pdStateBFOff( &state, -0.1 ) );

  SupportOnRightFoot();
  EXPECT_FALSE( pdStateBFOn( &state, 0.1 ) );
  EXPECT_TRUE( pdStateBFOff( &state, 0.1 ) );
  EXPECT_TRUE( pdStateBFOn( &state, -0.1 ) );
  EXPECT_FALSE( pdStateBFOff( &state, -0.1 ) );
}
