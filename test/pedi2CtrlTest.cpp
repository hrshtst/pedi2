#include "gtest/gtest.h"
#include <pedi2/pedi2_ctrl.h>

TEST(pedi2VertTest, UpdateVert)
{
  pedi2Vert vert;

  vert.z = 0.26;
  pedi2VertUpdate( &vert );
  EXPECT_EQ( sqrt( RK_G/0.26 ), vert.zeta );
}

class pedi2CtrlTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pedi2CtrlInit( &_ctrl );
    _ctrl.vert = &_vert;
    _ctrl.vert->z = 0.26;
    pedi2VertUpdate( _ctrl.vert );
  };
  virtual void TearDown() {};

  void SetDefaultParamWhenVelFol() {
    _ctrl.qu1 = 1.0;
    _ctrl.qu2 = 0.0;
    _ctrl.qw1 = 1.0;
    _ctrl.qw2 = 1.5;
    _ctrl.kappa = 0.0;
    _ctrl.rho = 1.0;
    _ctrl.kr = 1.0;
    _ctrl.vd = 0.25;
    _ctrl.w = 0.1;
  }

  void SetDefaultParamWhenVelFolCurve() {
    SetDefaultParamWhenVelFol();
    _ctrl.kappa = 2.0;
  }

  pedi2Vert _vert;
  pedi2Ctrl _ctrl;
};

TEST_F(pedi2CtrlTest, InitCtrlParams)
{
  pedi2Ctrl ctrl;

  pedi2CtrlInit( &ctrl );
  EXPECT_EQ( 0, ctrl.qu1 );
  EXPECT_EQ( 0, ctrl.qu2 );
  EXPECT_EQ( 0, ctrl.qw1 );
  EXPECT_EQ( 0, ctrl.qw2 );
  EXPECT_EQ( 0, ctrl.kappa );
  EXPECT_EQ( 0, ctrl.rho );
  EXPECT_EQ( 0, ctrl.kr );
  EXPECT_EQ( 0, ctrl.vd );
  EXPECT_EQ( 0, ctrl.w );
}

TEST_F(pedi2CtrlTest, SetupProcedure)
{
  pedi2Ctrl ctrl;
  pedi2Vert vert;

  pedi2CtrlInit( &ctrl );
  ctrl.vert = &vert;
  ctrl.vert->z = 0.26;
  pedi2VertUpdate( ctrl.vert );
  EXPECT_EQ( sqrt( RK_G/0.26 ), vert.zeta );
}

TEST_F(pedi2CtrlTest, SetParameters)
{
  SetDefaultParamWhenVelFol();
  EXPECT_EQ( 1.0, _ctrl.qw1 );
}

TEST_F(pedi2CtrlTest, CheckZMPWhenAllStateZero)
{
  SetDefaultParamWhenVelFol();
  pedi2CtrlZMP( &_ctrl, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, _ctrl.uz );
  EXPECT_DOUBLE_EQ( 0, _ctrl.wz );
}

TEST_F(pedi2CtrlTest, CheckZMPWhenVelFol)
{
  SetDefaultParamWhenVelFol();
  pedi2CtrlZMP( &_ctrl, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, _ctrl.uz );
  EXPECT_DOUBLE_EQ( 0.000310222468637089826560, _ctrl.wz );
}

TEST_F(pedi2CtrlTest, CheckZMPWhenVelFolCurve)
{
  SetDefaultParamWhenVelFolCurve();
  pedi2CtrlZMP( &_ctrl, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, _ctrl.uz );
  EXPECT_DOUBLE_EQ( -0.001854072725677786536574, _ctrl.wz );
}
