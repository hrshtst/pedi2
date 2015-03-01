#include "gtest/gtest.h"
#include <pedi2/pd_ctrl.h>

class pdCtrlTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdCtrl ctrl;
};

TEST_F(pdCtrlTest, Init)
{
  pdCtrl _ctrl;

  pdCtrlInit( &_ctrl );
  // vertical
  EXPECT_EQ( 0, _ctrl.vrt.z );
  // tangential
  EXPECT_EQ( 0, _ctrl.tan.q1 );
  EXPECT_EQ( 0, _ctrl.tan.q2 );
  EXPECT_EQ( 0, _ctrl.tan.kappa );
  EXPECT_EQ( 0, _ctrl.tan.rho );
  EXPECT_EQ( 0, _ctrl.tan.kr );
  EXPECT_EQ( &_ctrl.vrt, _ctrl.tan.vrt );
  // radial
  EXPECT_EQ( 0, _ctrl.rad.q1 );
  EXPECT_EQ( 0, _ctrl.rad.q2 );
  EXPECT_EQ( 0, _ctrl.rad.kappa );
  EXPECT_EQ( 0, _ctrl.rad.rho );
  EXPECT_EQ( 0, _ctrl.rad.kr );
  EXPECT_EQ( &_ctrl.vrt, _ctrl.rad.vrt );
}

TEST_F(pdCtrlTest, Update)
{
  pdCtrl _ctrl;

  pdCtrlInit( &_ctrl );
  pdCtrlUpdate( &_ctrl, 0.26 );
  EXPECT_EQ( 0.26, _ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCtrlZeta(&_ctrl) );

  pdCtrlUpdate( &_ctrl, 0.3 );
  EXPECT_EQ( 0.3, _ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.3), pdCtrlZeta(&_ctrl) );
}

TEST_F(pdCtrlTest, SetParameters)
{
  pdCtrlSetParam( &ctrl, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 1, ctrl.tan.q1 );
  EXPECT_EQ( 2, ctrl.tan.q2 );
  EXPECT_EQ( 3, ctrl.rad.q1 );
  EXPECT_EQ( 4, ctrl.rad.q2 );
  EXPECT_EQ( 5, ctrl.rad.kappa );
  EXPECT_EQ( 6, ctrl.rad.rho );
  EXPECT_EQ( 7, ctrl.rad.kr );
}
