#include "gtest/gtest.h"
#include <pedi2/pd_cz.h>

class pdCZTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZInit( &cz );
  };
  virtual void TearDown() {
    pdCZDestroy( &cz );
  };

  pdCZ cz;
};

TEST_F(pdCZTest, Init)
{
  pdCZ _cz;

  pdCZInit( &_cz );
  // vertical
  EXPECT_EQ( 0, _cz.vrt.zd );
  EXPECT_EQ( 0, _cz.vrt.z );
  EXPECT_EQ( 0, _cz.vrt.zeta );
  // tangential
  EXPECT_EQ( (byte)PD_CZ_HRZ_TAN, _cz.tan.dir );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->vd );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->q1 );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->q2 );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->kappa );
  EXPECT_EQ( &_cz.vrt, ((pdCZHrzPrmTan *)_cz.tan.prm)->vrt );
  EXPECT_EQ( 0, _cz.vrt.z );
  // radial
  EXPECT_EQ( (byte)PD_CZ_HRZ_RAD, _cz.rad.dir );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->vd );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->q1 );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->q2 );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->kappa );
  EXPECT_EQ( &_cz.vrt, ((pdCZHrzPrmRad *)_cz.rad.prm)->vrt );
  EXPECT_EQ( 0, _cz.vrt.z );
}

TEST_F(pdCZTest, Destroy)
{
  pdCZ _cz;

  pdCZInit( &_cz );
  pdCZDestroy( &_cz );
  // vertical
  EXPECT_EQ( 0, _cz.vrt.zd );
  EXPECT_EQ( 0, _cz.vrt.z );
  EXPECT_EQ( 0, _cz.vrt.zeta );
  // tangential
  EXPECT_EQ( -1, _cz.tan.dir );
  EXPECT_EQ( NULL, _cz.tan.prm );
  EXPECT_EQ( NULL, _cz.tan.com );
  // radial
  EXPECT_EQ( -1, _cz.rad.dir );
  EXPECT_EQ( NULL, _cz.rad.prm );
  EXPECT_EQ( NULL, _cz.rad.com );
}

TEST_F(pdCZTest, SetPrm)
{
  pdCZSetPrm( &cz, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 1, pdCZPrmTan(&cz)->q1 );
  EXPECT_EQ( 2, pdCZPrmTan(&cz)->q2 );
  EXPECT_EQ( 5, pdCZPrmTan(&cz)->kappa );
  EXPECT_EQ( 3, pdCZPrmRad(&cz)->q1 );
  EXPECT_EQ( 4, pdCZPrmRad(&cz)->q2 );
  EXPECT_EQ( 5, pdCZPrmRad(&cz)->kappa );
  EXPECT_EQ( 6, pdCZPrmRad(&cz)->rho );
  EXPECT_EQ( 7, pdCZPrmRad(&cz)->kr );
}

TEST_F(pdCZTest, SetRefVrt)
{
  pdCZSetRefVrt( &cz, 1 );
  EXPECT_EQ( 1, cz.vrt.zd );
}

TEST_F(pdCZTest, SetRefHrz)
{
  pdCZSetRefHrz( &cz, 1, 2, 3 );
  EXPECT_EQ( 1, pdCZPrmTan(&cz)->vd );
  EXPECT_EQ( 2, pdCZPrmRad(&cz)->vd );
  EXPECT_EQ( 3, pdCZPrmRad(&cz)->dist );
}

TEST_F(pdCZTest, Kappa)
{
  pdCZSetPrm( &cz, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 5, pdCZKappa( &cz ) );
  EXPECT_EQ( pdCZPrmTan(&cz)->kappa, pdCZKappa( &cz ) );
  EXPECT_EQ( pdCZPrmRad(&cz)->kappa, pdCZKappa( &cz ) );
}

TEST_F(pdCZTest, UpdateAllStateZero)
{
  pdCZSetPrm( &cz, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCZSetRefVrt( &cz, 0.26 );
  pdCZSetRefHrz( &cz, 0.25, 0.0, 0.1 );
  pdCZUpdate( &cz, 0.0, 0.0, 0.0, 0.0 );
  EXPECT_EQ( 0.26, cz.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&cz) );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdCZZMPTan( &cz, 0.0, 0.0, 0.0, 0.0 ) );
  EXPECT_DOUBLE_EQ( 0.0, pdCZZMPRad(&cz, 0.0, 0.0, 0.0, 0.0 ) );
}

TEST_F(pdCZTest, UpdateVelocityFollow)
{
  pdCZSetPrm( &cz, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCZSetRefVrt( &cz, 0.26 );
  pdCZSetRefHrz( &cz, 0.25, 0.0, 0.1 );
  pdCZUpdate( &cz, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, cz.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&cz) );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZZMPTan( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  //EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdCZZMPRad( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdCZZMPRad( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542, pdCZAccTan( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0.011700937663480142150729, pdCZAccRad( &cz, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZTest, UpdateVelocityFollowCurve)
{
  pdCZSetPrm( &cz, 1.0, 0.0, 1.0, 1.5, 2.0, 1.0, 1.0 );
  pdCZSetRefVrt( &cz, 0.26 );
  pdCZSetRefHrz( &cz, 0.25, 0.0, 0.1 );
  pdCZUpdate( &cz, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, cz.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&cz) );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZZMPTan( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  //EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCZZMPRad( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCZZMPRad( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0.276367023942938716540851, pdCZAccTan( &cz, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0.011700937663480154293794, pdCZAccRad( &cz, 0.0, 0.2, 0.01, 0.1 ) );
}
