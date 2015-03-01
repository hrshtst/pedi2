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
  EXPECT_EQ( 0, _ctrl.vrt.zd );
  EXPECT_EQ( 0, _ctrl.vrt.z );
  EXPECT_EQ( 0, _ctrl.vrt.zeta );
  // tangential
  EXPECT_EQ( (byte)PD_HRZ_TAN, _ctrl.tan.dir );
  EXPECT_EQ( 0, ((pdHrzPrmTan *)_ctrl.tan.prm)->vd );
  EXPECT_EQ( 0, ((pdHrzPrmTan *)_ctrl.tan.prm)->q1 );
  EXPECT_EQ( 0, ((pdHrzPrmTan *)_ctrl.tan.prm)->q2 );
  EXPECT_EQ( 0, ((pdHrzPrmTan *)_ctrl.tan.prm)->kappa );
  EXPECT_EQ( &_ctrl.vrt, ((pdHrzPrmTan *)_ctrl.tan.prm)->vrt );
  EXPECT_EQ( 0, _ctrl.vrt.z );
  // radial
  EXPECT_EQ( (byte)PD_HRZ_RAD, _ctrl.rad.dir );
  EXPECT_EQ( 0, ((pdHrzPrmRad *)_ctrl.rad.prm)->vd );
  EXPECT_EQ( 0, ((pdHrzPrmRad *)_ctrl.rad.prm)->q1 );
  EXPECT_EQ( 0, ((pdHrzPrmRad *)_ctrl.rad.prm)->q2 );
  EXPECT_EQ( 0, ((pdHrzPrmRad *)_ctrl.rad.prm)->kappa );
  EXPECT_EQ( &_ctrl.vrt, ((pdHrzPrmRad *)_ctrl.rad.prm)->vrt );
  EXPECT_EQ( 0, _ctrl.vrt.z );
}

TEST_F(pdCtrlTest, Destroy)
{
  pdCtrl _ctrl;

  pdCtrlInit( &_ctrl );
  pdCtrlDestroy( &_ctrl );
  // vertical
  EXPECT_EQ( 0, _ctrl.vrt.zd );
  EXPECT_EQ( 0, _ctrl.vrt.z );
  EXPECT_EQ( 0, _ctrl.vrt.zeta );
  // tangential
  EXPECT_EQ( -1, _ctrl.tan.dir );
  EXPECT_EQ( NULL, _ctrl.tan.prm );
  EXPECT_EQ( NULL, _ctrl.tan.com );
  // radial
  EXPECT_EQ( -1, _ctrl.rad.dir );
  EXPECT_EQ( NULL, _ctrl.rad.prm );
  EXPECT_EQ( NULL, _ctrl.rad.com );
}

#if 0
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
#endif
