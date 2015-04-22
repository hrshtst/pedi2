#include "gtest/gtest.h"
#include <pedi2/pd_cz_hrz_mov.h>

class pdCZHrzMovTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzMovInit( &hrz, &vrt );
  };
  virtual void TearDown() {
    pdCZHrzMovDestroy( &hrz );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdCZVrtSetRef( &vrt, 1 );
    pdCZHrzMovSetRefVelU( &hrz, 2 );
    pdCZHrzMovSetQ1U( &hrz, 3 );
    pdCZHrzMovSetQ2U( &hrz, 4 );
    pdCZHrzMovSetRefVelW( &hrz, 5 );
    pdCZHrzMovSetQ1W( &hrz, 6 );
    pdCZHrzMovSetQ2W( &hrz, 7 );
    pdCZHrzMovSetRho( &hrz, 8 );
    pdCZHrzMovSetKr( &hrz, 9 );
    pdCZHrzMovSetDist( &hrz, 10 );
    pdCZHrzMovSetKappa( &hrz, 11 );
    hrz._vrt = NULL;
    hrz._sr = &sr;
    hrz.refuz = 12;
    hrz.refwz = 13;
    hrz.refddu = 14;
    hrz.refddw = 15;
  };

  void SetDefaultPrmVelocityFollow() {
    pdCZVrtSetRef( &vrt, 0.26 );
    pdCZHrzMovSetRefVelU( &hrz, 0.25 );
    pdCZHrzMovSetQ1U( &hrz, 1.0 );
    pdCZHrzMovSetQ2U( &hrz, 0.0 );
    pdCZHrzMovSetRefVelW( &hrz, 0.0 );
    pdCZHrzMovSetQ1W( &hrz, 1.0 );
    pdCZHrzMovSetQ2W( &hrz, 1.5 );
    pdCZHrzMovSetRho( &hrz, 1.0 );
    pdCZHrzMovSetKr( &hrz, 1.0 );
    pdCZHrzMovSetDist( &hrz, 0.1 );
    pdCZHrzMovSetKappa( &hrz, 0.0 );
  };

  void SetDefaultPrmVelocityFollowCurve() {
    SetDefaultPrmVelocityFollow();
    pdCZHrzMovSetKappa( &hrz, 2.0 );
  };

  zVec3DList sr;
  pdCZVrt vrt;
  pdCZHrzMov hrz;
};

TEST_F(pdCZHrzMovTest, Init)
{
  SetVacuousPrm();
  pdCZVrtInit( &vrt );
  pdCZHrzMovInit( &hrz, &vrt );
  EXPECT_EQ( 0, pdCZHrzMovRefVelU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRefVelW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRho( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKr( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovDist( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKappa( &hrz ) );
  EXPECT_EQ( &vrt, hrz._vrt );
  EXPECT_EQ( &vrt.zeta, hrz._u._zeta );
  EXPECT_EQ( &vrt.zeta, hrz._w._zeta );
  EXPECT_EQ( &hrz._kappa, hrz._u._kappa );
  EXPECT_EQ( &hrz._kappa, hrz._w._kappa );
  EXPECT_EQ( NULL, hrz._sr );
  EXPECT_EQ( 0, pdCZHrzMovZMPU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovZMPW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovAccU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovAccW( &hrz ) );
}

TEST_F(pdCZHrzMovTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzMovDestroy( &hrz );
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, pdCZHrzMovRefVelU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRefVelW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRho( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKr( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovDist( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKappa( &hrz ) );
  EXPECT_EQ( NULL, hrz._vrt );
  EXPECT_EQ( NULL, hrz._u._zeta );
  EXPECT_EQ( NULL, hrz._w._zeta );
  EXPECT_EQ( NULL, hrz._u._kappa );
  EXPECT_EQ( NULL, hrz._w._kappa );
  EXPECT_EQ( NULL, hrz._sr );
}

#if 0
TEST_F(pdCZHrzTest, CheckZMPTanAllStateZero)
{
  pdCZHrzPrmTan pt;

  pt.vd = 0.0;
  pt.q1 = 0.0;
  pt.q2 = 0.0;
  pt.kappa = 0.0;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &tan, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( 0.0, pdCZHrzZMP( &tan ) );

  pt.vd = 0.25;
  pt.q1 = 1.0;
  pt.q2 = 0.0;
  pt.kappa = 0.0;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &tan, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdCZHrzZMP( &tan ) );
}

TEST_F(pdCZHrzTest, CheckZMPRadAllStateZero)
{
  pdCZHrzPrmRad pr;

  pr.vd = 0.0;
  pr.q1 = 1.0;
  pr.q2 = 1.5;
  pr.kappa = 0.0;
  pr.rho = 1.0;
  pr.kr = 1.0;
  pr.dist = 0.1;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &rad, &pr );
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &rad, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( 0.0, pdCZHrzZMP( &rad ) );
}

TEST_F(pdCZHrzTest, CheckZMPTanVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &tan, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZHrzZMP( &tan ) );
}

TEST_F(pdCZHrzTest, CheckZMPRadVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &rad, 0.0, 0.2, 0.01, 0.1 );
  // EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdCZHrzZMP( &rad ) );
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdCZHrzZMP( &rad ) );
}

TEST_F(pdCZHrzTest, CheckZMPTanVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &tan, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZHrzZMP( &tan ) );
}

TEST_F(pdCZHrzTest, CheckZMPRadVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &rad, 0.0, 0.2, 0.01, 0.1 );
  // EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCZHrzZMP( &rad ) );
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCZHrzZMP( &rad ) );
}

TEST_F(pdCZHrzTest, CheckAccTanVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &tan, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542, pdCZHrzAcc( &tan ) );
}

TEST_F(pdCZHrzTest, CheckAccRadVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &rad, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( 0.011700937663480142150729, pdCZHrzAcc( &rad ) );
}

TEST_F(pdCZHrzTest, CheckAccTanVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &tan, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( 0.276367023942938716540851, pdCZHrzAcc( &tan ) );
}

TEST_F(pdCZHrzTest, CheckAccRadVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  pdCZHrzUpdate( &rad, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( 0.011700937663480154293794, pdCZHrzAcc( &rad ) );
}
#endif
