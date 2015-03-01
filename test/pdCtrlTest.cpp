#include "gtest/gtest.h"
#include <pedi2/pd_ctrl.h>

class pdCtrlTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCtrlInit( &ctrl );
  };
  virtual void TearDown() {
    pdCtrlDestroy( &ctrl );
  };

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

TEST_F(pdCtrlTest, SetPrm)
{
  pdCtrlSetPrm( &ctrl, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 1, pdCtrlPrmTan(&ctrl)->q1 );
  EXPECT_EQ( 2, pdCtrlPrmTan(&ctrl)->q2 );
  EXPECT_EQ( 5, pdCtrlPrmTan(&ctrl)->kappa );
  EXPECT_EQ( 3, pdCtrlPrmRad(&ctrl)->q1 );
  EXPECT_EQ( 4, pdCtrlPrmRad(&ctrl)->q2 );
  EXPECT_EQ( 5, pdCtrlPrmRad(&ctrl)->kappa );
  EXPECT_EQ( 6, pdCtrlPrmRad(&ctrl)->rho );
  EXPECT_EQ( 7, pdCtrlPrmRad(&ctrl)->kr );
}

TEST_F(pdCtrlTest, SetRefVrt)
{
  pdCtrlSetRefVrt( &ctrl, 1 );
  EXPECT_EQ( 1, ctrl.vrt.zd );
}

TEST_F(pdCtrlTest, SetRefHrz)
{
  pdCtrlSetRefHrz( &ctrl, 1, 2, 3 );
  EXPECT_EQ( 1, pdCtrlPrmTan(&ctrl)->vd );
  EXPECT_EQ( 2, pdCtrlPrmRad(&ctrl)->vd );
  EXPECT_EQ( 3, pdCtrlPrmRad(&ctrl)->dist );
}

TEST_F(pdCtrlTest, Kappa)
{
  pdCtrlSetPrm( &ctrl, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 5, pdCtrlKappa( &ctrl ) );
  EXPECT_EQ( pdCtrlPrmTan(&ctrl)->kappa, pdCtrlKappa( &ctrl ) );
  EXPECT_EQ( pdCtrlPrmRad(&ctrl)->kappa, pdCtrlKappa( &ctrl ) );
}

TEST_F(pdCtrlTest, GetZMP)
{
  pdCtrlPrmTan(&ctrl)->uz = 0;
  EXPECT_EQ( 0, pdCtrlZMPTan(&ctrl) );
  pdCtrlPrmTan(&ctrl)->uz = 0.5;
  EXPECT_EQ( 0.5, pdCtrlZMPTan(&ctrl) );

  pdCtrlPrmRad(&ctrl)->wz = 0;
  EXPECT_EQ( 0, pdCtrlZMPRad(&ctrl) );
  pdCtrlPrmRad(&ctrl)->wz = 0.5;
  EXPECT_EQ( 0.5, pdCtrlZMPRad(&ctrl) );
}

TEST_F(pdCtrlTest, UpdateAllStateZero)
{
  pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCtrlSetRefVrt( &ctrl, 0.26 );
  pdCtrlSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  pdCtrlUpdate( &ctrl, 0.0, 0.0, 0.0, 0.0 );
  EXPECT_EQ( 0.26, ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCtrlZeta(&ctrl) );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdCtrlZMPTan(&ctrl) );
  EXPECT_DOUBLE_EQ( 0.0, pdCtrlZMPRad(&ctrl) );
}

TEST_F(pdCtrlTest, UpdateVelocityFollow)
{
  pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCtrlSetRefVrt( &ctrl, 0.26 );
  pdCtrlSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  pdCtrlUpdate( &ctrl, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCtrlZeta(&ctrl) );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCtrlZMPTan(&ctrl) );
  //EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdCtrlZMPRad(&ctrl));
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdCtrlZMPRad(&ctrl) );
}

TEST_F(pdCtrlTest, UpdateVelocityFollowCurve)
{
  pdCtrlSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 2.0, 1.0, 1.0 );
  pdCtrlSetRefVrt( &ctrl, 0.26 );
  pdCtrlSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  pdCtrlUpdate( &ctrl, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCtrlZeta(&ctrl) );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCtrlZMPTan(&ctrl) );
  //EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCtrlZMPRad(&ctrl));
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCtrlZMPRad(&ctrl) );
}
