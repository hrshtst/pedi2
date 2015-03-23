#include "gtest/gtest.h"
#include <pedi2/pd_cz_vrt.h>
#include <pedi2/pd_cz_hrz.h>

class pdCZHrzTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzSetup( &tan, &vrt, PD_CZ_HRZ_TAN );
    pdCZHrzSetup( &rad, &vrt, PD_CZ_HRZ_RAD );
  };
  virtual void TearDown() {
    pdCZHrzDestroy( &tan );
    pdCZHrzDestroy( &rad );
    pdCZVrtDestroy( &vrt );
  };

  void SetDefaultPrmVelocityFollow() {
    pdCZVrtSetRef( &vrt, 0.26 );
    ((pdCZHrzPrmTan *)tan.prm)->vd = 0.25;
    ((pdCZHrzPrmTan *)tan.prm)->q1 = 1.0;
    ((pdCZHrzPrmTan *)tan.prm)->q2 = 0.0;
    ((pdCZHrzPrmTan *)tan.prm)->kappa = 0.0;
    ((pdCZHrzPrmRad *)rad.prm)->q1 = 1.0;
    ((pdCZHrzPrmRad *)rad.prm)->q2 = 1.5;
    ((pdCZHrzPrmRad *)rad.prm)->kappa = 0.0;
    ((pdCZHrzPrmRad *)rad.prm)->rho = 1.0;
    ((pdCZHrzPrmRad *)rad.prm)->kr = 1.0;
    ((pdCZHrzPrmRad *)rad.prm)->dist = 0.1;
  }
  void SetDefaultPrmVelocityFollowCurve() {
    SetDefaultPrmVelocityFollow();
    ((pdCZHrzPrmTan *)tan.prm)->kappa = 2.0;
    ((pdCZHrzPrmRad *)rad.prm)->kappa = 2.0;
  }

  pdCZVrt vrt;
  pdCZHrz tan;
  pdCZHrz rad;
};

TEST_F(pdCZHrzTest, Init)
{
  pdCZHrz _hrz;

  pdCZHrzInit( &_hrz );
  EXPECT_EQ( -1, _hrz.dir );
  EXPECT_EQ( NULL, _hrz.prm );
  EXPECT_EQ( NULL, _hrz.com );
}

TEST_F(pdCZHrzTest, SetupOutOfRange)
{
  pdCZVrt _vrt;
  pdCZHrz _hrz;
  pdCZHrz *ret;

  ret = pdCZHrzSetup( &_hrz, &_vrt, -1 );
  EXPECT_EQ( NULL, ret );
  ret = pdCZHrzSetup( &_hrz, &_vrt, 9 );
  EXPECT_EQ( NULL, ret );
}

TEST_F(pdCZHrzTest, SetupHrzTan)
{
  pdCZVrt _vrt;
  pdCZHrz _tan;
  pdCZHrzPrmTan *prm;

  pdCZVrtInit( &_vrt );
  pdCZHrzSetup( &_tan, &_vrt, PD_CZ_HRZ_TAN );
  prm = (pdCZHrzPrmTan *)_tan.prm;
  EXPECT_EQ( (byte)PD_CZ_HRZ_TAN, _tan.dir );
  EXPECT_EQ( 0, prm->vd );
  EXPECT_EQ( 0, prm->q1 );
  EXPECT_EQ( 0, prm->q2 );
  EXPECT_EQ( 0, prm->kappa );
  EXPECT_EQ( &_vrt, prm->vrt );
  EXPECT_EQ( 0, _vrt.z );
}

TEST_F(pdCZHrzTest, SetupHrzRad)
{
  pdCZVrt _vrt;
  pdCZHrz _rad;
  pdCZHrzPrmRad *prm;

  pdCZVrtInit( &_vrt );
  pdCZHrzSetup( &_rad, &_vrt, PD_CZ_HRZ_RAD );
  prm = (pdCZHrzPrmRad *)_rad.prm;
  EXPECT_EQ( (byte)PD_CZ_HRZ_RAD, _rad.dir );
  EXPECT_EQ( 0, prm->vd );
  EXPECT_EQ( 0, prm->q1 );
  EXPECT_EQ( 0, prm->q2 );
  EXPECT_EQ( 0, prm->kappa );
  EXPECT_EQ( 0, prm->rho );
  EXPECT_EQ( 0, prm->kr );
  EXPECT_EQ( 0, prm->dist );
  EXPECT_EQ( &_vrt, prm->vrt );
  EXPECT_EQ( 0, _vrt.z );
}

TEST_F(pdCZHrzTest, Destroy)
{
  pdCZVrt _vrt;
  pdCZHrz _tan;

  pdCZHrzSetup( &_tan, &_vrt, PD_CZ_HRZ_TAN );
  pdCZHrzDestroy( &_tan );
  EXPECT_EQ( -1, _tan.dir );
  EXPECT_EQ( NULL, _tan.prm );
  EXPECT_EQ( NULL, _tan.com );
}

TEST_F(pdCZHrzTest, SetPrmTan)
{
  pdCZHrzPrmTan pt;

  pt.vd = 1;
  pt.q1 = 2;
  pt.q2 = 3;
  pt.kappa = 4;
  pdCZHrzSetPrm( &tan, &pt );
  EXPECT_EQ( 1, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->vd );
  EXPECT_EQ( 2, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->q1 );
  EXPECT_EQ( 3, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->q2 );
  EXPECT_EQ( 4, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->kappa );

  pt.q1 = 5;
  pt.q2 = 6;
  pt.kappa = 7;
  pdCZHrzSetPrm( &tan, &pt );
  EXPECT_EQ( 1, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->vd );
  EXPECT_EQ( 5, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->q1 );
  EXPECT_EQ( 6, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->q2 );
  EXPECT_EQ( 7, ((pdCZHrzPrmTan *)pdCZHrzPrm(&tan))->kappa );
}

TEST_F(pdCZHrzTest, SetPrmRad)
{
  pdCZHrzPrmRad pr;

  pr.vd    = 1;
  pr.q1    = 2;
  pr.q2    = 3;
  pr.kappa = 4;
  pr.rho   = 5;
  pr.kr    = 6;
  pr.dist  = 7;
  pdCZHrzSetPrm( &rad, &pr );
  EXPECT_EQ( 1, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->vd );
  EXPECT_EQ( 2, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->q1 );
  EXPECT_EQ( 3, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->q2 );
  EXPECT_EQ( 4, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->kappa );
  EXPECT_EQ( 5, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->rho );
  EXPECT_EQ( 6, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->kr );
  EXPECT_EQ( 7, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->dist );

  pr.q1 = 8;
  pr.q2 = 9;
  pr.kappa = 10;
  pr.rho   = 11;
  pr.kr    = 12;
  pdCZHrzSetPrm( &rad, &pr );
  EXPECT_EQ( 1,  ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->vd );
  EXPECT_EQ( 8,  ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->q1 );
  EXPECT_EQ( 9,  ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->q2 );
  EXPECT_EQ( 10, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->kappa );
  EXPECT_EQ( 11, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->rho );
  EXPECT_EQ( 12, ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->kr );
  EXPECT_EQ( 7,  ((pdCZHrzPrmRad *)pdCZHrzPrm(&rad))->dist );
}

TEST_F(pdCZHrzTest, HrzK1Tan)
{
  pdCZHrzPrmTan pt;

  pt.q1 = 1.0;
  pt.q2 = 0.5;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.5, pdCZHrzK1( &tan ) );

  pt.q1 = 0.8;
  pt.q2 = 1.3;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.8*1.3, pdCZHrzK1( &tan ) );
}

TEST_F(pdCZHrzTest, HrzK2Tan)
{
  pdCZHrzPrmTan pt;

  pt.q1 = 1.0;
  pt.q2 = 0.5;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 1.5/sqrt(RK_G/0.26), pdCZHrzK2( &tan ) );

  pt.q1 = 0.8;
  pt.q2 = 1.3;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 2.1/sqrt(RK_G/0.26), pdCZHrzK2( &tan ) );
}

TEST_F(pdCZHrzTest, HrzK1Rad)
{
  pdCZHrzPrmRad pr;

  pr.q1 = 1.0;
  pr.q2 = 0.5;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &rad, &pr );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.5, pdCZHrzK1( &rad ) );

  pr.q1 = 0.8;
  pr.q2 = 1.3;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &rad, &pr );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.8*1.3, pdCZHrzK1( &rad ) );
}

TEST_F(pdCZHrzTest, HrzK2Rad)
{
  pdCZHrzPrmRad pr;

  pr.q1 = 1.0;
  pr.q2 = 0.5;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &rad, &pr );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 1.5/sqrt(RK_G/0.26), pdCZHrzK2( &rad ) );

  pr.q1 = 0.8;
  pr.q2 = 1.3;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &rad, &pr );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 2.1/sqrt(RK_G/0.26), pdCZHrzK2( &rad ) );
}

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
  EXPECT_DOUBLE_EQ( 0.0, pdCZHrzZMP( &tan, 0, 0, 0, 0 ) );

  pt.vd = 0.25;
  pt.q1 = 1.0;
  pt.q2 = 0.0;
  pt.kappa = 0.0;
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzSetPrm( &tan, &pt );
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdCZHrzZMP( &tan, 0, 0, 0, 0 ) );
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
  EXPECT_DOUBLE_EQ( 0.0, pdCZHrzZMP( &rad, 0, 0, 0, 0 ) );
}

TEST_F(pdCZHrzTest, CheckZMPTanVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZHrzZMP( &tan, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckZMPRadVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  // EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdCZHrzZMP( &rad, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdCZHrzZMP( &rad, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckZMPTanVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZHrzZMP( &tan, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckZMPRadVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  // EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCZHrzZMP( &rad, 0.0, 0.2, 0.01, 0.1 ) );
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCZHrzZMP( &rad, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckAccTanVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542, pdCZHrzAcc( &tan, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckAccRadVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( 0.011700937663480142150729, pdCZHrzAcc( &rad, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckAccTanVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( 0.276367023942938716540851, pdCZHrzAcc( &tan, 0.0, 0.2, 0.01, 0.1 ) );
}

TEST_F(pdCZHrzTest, CheckAccRadVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt );
  EXPECT_DOUBLE_EQ( 0.011700937663480154293794, pdCZHrzAcc( &rad, 0.0, 0.2, 0.01, 0.1 ) );
}
