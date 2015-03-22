#include "gtest/gtest.h"
#include <pedi2/pd_cz.h>

class pdCZTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZInit( &ctrl );
  };
  virtual void TearDown() {
    pdCZDestroy( &ctrl );
  };

  pdCZ ctrl;
};

TEST_F(pdCZTest, Init)
{
  pdCZ _ctrl;

  pdCZInit( &_ctrl );
  // vertical
  EXPECT_EQ( 0, _ctrl.vrt.zd );
  EXPECT_EQ( 0, _ctrl.vrt.z );
  EXPECT_EQ( 0, _ctrl.vrt.zeta );
  // tangential
  EXPECT_EQ( (byte)PD_CZ_HRZ_TAN, _ctrl.tan.dir );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_ctrl.tan.prm)->vd );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_ctrl.tan.prm)->q1 );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_ctrl.tan.prm)->q2 );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_ctrl.tan.prm)->kappa );
  EXPECT_EQ( &_ctrl.vrt, ((pdCZHrzPrmTan *)_ctrl.tan.prm)->vrt );
  EXPECT_EQ( 0, _ctrl.vrt.z );
  // radial
  EXPECT_EQ( (byte)PD_CZ_HRZ_RAD, _ctrl.rad.dir );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_ctrl.rad.prm)->vd );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_ctrl.rad.prm)->q1 );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_ctrl.rad.prm)->q2 );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_ctrl.rad.prm)->kappa );
  EXPECT_EQ( &_ctrl.vrt, ((pdCZHrzPrmRad *)_ctrl.rad.prm)->vrt );
  EXPECT_EQ( 0, _ctrl.vrt.z );
}

TEST_F(pdCZTest, Destroy)
{
  pdCZ _ctrl;

  pdCZInit( &_ctrl );
  pdCZDestroy( &_ctrl );
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

TEST_F(pdCZTest, SetPrm)
{
  pdCZSetPrm( &ctrl, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 1, pdCZPrmTan(&ctrl)->q1 );
  EXPECT_EQ( 2, pdCZPrmTan(&ctrl)->q2 );
  EXPECT_EQ( 5, pdCZPrmTan(&ctrl)->kappa );
  EXPECT_EQ( 3, pdCZPrmRad(&ctrl)->q1 );
  EXPECT_EQ( 4, pdCZPrmRad(&ctrl)->q2 );
  EXPECT_EQ( 5, pdCZPrmRad(&ctrl)->kappa );
  EXPECT_EQ( 6, pdCZPrmRad(&ctrl)->rho );
  EXPECT_EQ( 7, pdCZPrmRad(&ctrl)->kr );
}

TEST_F(pdCZTest, SetRefVrt)
{
  pdCZSetRefVrt( &ctrl, 1 );
  EXPECT_EQ( 1, ctrl.vrt.zd );
}

TEST_F(pdCZTest, SetRefHrz)
{
  pdCZSetRefHrz( &ctrl, 1, 2, 3 );
  EXPECT_EQ( 1, pdCZPrmTan(&ctrl)->vd );
  EXPECT_EQ( 2, pdCZPrmRad(&ctrl)->vd );
  EXPECT_EQ( 3, pdCZPrmRad(&ctrl)->dist );
}

TEST_F(pdCZTest, Kappa)
{
  pdCZSetPrm( &ctrl, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 5, pdCZKappa( &ctrl ) );
  EXPECT_EQ( pdCZPrmTan(&ctrl)->kappa, pdCZKappa( &ctrl ) );
  EXPECT_EQ( pdCZPrmRad(&ctrl)->kappa, pdCZKappa( &ctrl ) );
}

TEST_F(pdCZTest, GetZMP)
{
  pdCZPrmTan(&ctrl)->uz = 0;
  EXPECT_EQ( 0, pdCZZMPTan(&ctrl) );
  pdCZPrmTan(&ctrl)->uz = 0.5;
  EXPECT_EQ( 0.5, pdCZZMPTan(&ctrl) );

  pdCZPrmRad(&ctrl)->wz = 0;
  EXPECT_EQ( 0, pdCZZMPRad(&ctrl) );
  pdCZPrmRad(&ctrl)->wz = 0.5;
  EXPECT_EQ( 0.5, pdCZZMPRad(&ctrl) );
}

TEST_F(pdCZTest, UpdateAllStateZero)
{
  pdCZSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCZSetRefVrt( &ctrl, 0.26 );
  pdCZSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  pdCZUpdate( &ctrl, 0.0, 0.0, 0.0, 0.0 );
  EXPECT_EQ( 0.26, ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&ctrl) );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdCZZMPTan(&ctrl) );
  EXPECT_DOUBLE_EQ( 0.0, pdCZZMPRad(&ctrl) );
}

TEST_F(pdCZTest, UpdateVelocityFollow)
{
  pdCZSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCZSetRefVrt( &ctrl, 0.26 );
  pdCZSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  pdCZUpdate( &ctrl, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&ctrl) );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZZMPTan(&ctrl) );
  //EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdCZZMPRad(&ctrl));
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdCZZMPRad(&ctrl) );
}

TEST_F(pdCZTest, UpdateVelocityFollowCurve)
{
  pdCZSetPrm( &ctrl, 1.0, 0.0, 1.0, 1.5, 2.0, 1.0, 1.0 );
  pdCZSetRefVrt( &ctrl, 0.26 );
  pdCZSetRefHrz( &ctrl, 0.25, 0.0, 0.1 );
  pdCZUpdate( &ctrl, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, ctrl.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&ctrl) );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZZMPTan(&ctrl) );
  //EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCZZMPRad(&ctrl));
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCZZMPRad(&ctrl) );
}
