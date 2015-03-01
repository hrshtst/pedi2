#include "gtest/gtest.h"
#include <pedi2/pd_vrt.h>
#include <pedi2/pd_hrz.h>

class pdHrzTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdVrtInit( &vrt );
    pdHrzSetup( &tan, &vrt, PD_HRZ_TAN );
    pdHrzSetup( &rad, &vrt, PD_HRZ_RAD );
  };
  virtual void TearDown() {
    pdHrzDestroy( &tan );
    pdHrzDestroy( &rad );
    pdVrtDestroy( &vrt );
  };

  void SetDefaultPrmVelocityFollow() {
    pdVrtSetPrm( &vrt, 0.26 );
    ((pdHrzPrmTan *)tan.prm)->vd = 0.25;
    ((pdHrzPrmTan *)tan.prm)->q1 = 1.0;
    ((pdHrzPrmTan *)tan.prm)->q2 = 0.0;
    ((pdHrzPrmTan *)tan.prm)->kappa = 0.0;
    ((pdHrzPrmRad *)rad.prm)->q1 = 1.0;
    ((pdHrzPrmRad *)rad.prm)->q2 = 1.5;
    ((pdHrzPrmRad *)rad.prm)->kappa = 0.0;
    ((pdHrzPrmRad *)rad.prm)->rho = 1.0;
    ((pdHrzPrmRad *)rad.prm)->kr = 1.0;
    ((pdHrzPrmRad *)rad.prm)->dist = 0.1;
  }
  void SetDefaultPrmVelocityFollowCurve() {
    SetDefaultPrmVelocityFollow();
    ((pdHrzPrmTan *)tan.prm)->kappa = 2.0;
    ((pdHrzPrmRad *)rad.prm)->kappa = 2.0;
  }

  pdVrt vrt;
  pdHrz tan;
  pdHrz rad;
};

TEST_F(pdHrzTest, Init)
{
  pdHrz _hrz;

  pdHrzInit( &_hrz );
  EXPECT_EQ( -1, _hrz.dir );
  EXPECT_EQ( NULL, _hrz.prm );
  EXPECT_EQ( NULL, _hrz.com );
}

TEST_F(pdHrzTest, SetupOutOfRange)
{
  pdVrt _vrt;
  pdHrz _hrz;
  pdHrz *ret;

  ret = pdHrzSetup( &_hrz, &_vrt, -1 );
  EXPECT_EQ( NULL, ret );
  ret = pdHrzSetup( &_hrz, &_vrt, 9 );
  EXPECT_EQ( NULL, ret );
}

TEST_F(pdHrzTest, SetupHrzTan)
{
  pdVrt _vrt;
  pdHrz _tan;
  pdHrzPrmTan *prm;

  pdVrtInit( &_vrt );
  pdHrzSetup( &_tan, &_vrt, PD_HRZ_TAN );
  prm = (pdHrzPrmTan *)_tan.prm;
  EXPECT_EQ( (byte)PD_HRZ_TAN, _tan.dir );
  EXPECT_EQ( 0, prm->vd );
  EXPECT_EQ( 0, prm->q1 );
  EXPECT_EQ( 0, prm->q2 );
  EXPECT_EQ( 0, prm->kappa );
  EXPECT_EQ( &_vrt, prm->vrt );
  EXPECT_EQ( 0, _vrt.z );
}

TEST_F(pdHrzTest, SetupHrzRad)
{
  pdVrt _vrt;
  pdHrz _rad;
  pdHrzPrmRad *prm;

  pdVrtInit( &_vrt );
  pdHrzSetup( &_rad, &_vrt, PD_HRZ_RAD );
  prm = (pdHrzPrmRad *)_rad.prm;
  EXPECT_EQ( (byte)PD_HRZ_RAD, _rad.dir );
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

TEST_F(pdHrzTest, Destroy)
{
  pdVrt _vrt;
  pdHrz _tan;

  pdHrzSetup( &_tan, &_vrt, PD_HRZ_TAN );
  pdHrzDestroy( &_tan );
  EXPECT_EQ( -1, _tan.dir );
  EXPECT_EQ( NULL, _tan.prm );
  EXPECT_EQ( NULL, _tan.com );
}

TEST_F(pdHrzTest, SetPrmTan)
{
  pdHrzPrmTan pt;

  pt.vd = 1;
  pt.q1 = 2;
  pt.q2 = 3;
  pt.kappa = 4;
  pdHrzSetPrm( &tan, &pt );
  EXPECT_EQ( 1, ((pdHrzPrmTan *)pdHrzPrm(&tan))->vd );
  EXPECT_EQ( 2, ((pdHrzPrmTan *)pdHrzPrm(&tan))->q1 );
  EXPECT_EQ( 3, ((pdHrzPrmTan *)pdHrzPrm(&tan))->q2 );
  EXPECT_EQ( 4, ((pdHrzPrmTan *)pdHrzPrm(&tan))->kappa );

  pt.q1 = 5;
  pt.q2 = 6;
  pt.kappa = 7;
  pdHrzSetPrm( &tan, &pt );
  EXPECT_EQ( 1, ((pdHrzPrmTan *)pdHrzPrm(&tan))->vd );
  EXPECT_EQ( 5, ((pdHrzPrmTan *)pdHrzPrm(&tan))->q1 );
  EXPECT_EQ( 6, ((pdHrzPrmTan *)pdHrzPrm(&tan))->q2 );
  EXPECT_EQ( 7, ((pdHrzPrmTan *)pdHrzPrm(&tan))->kappa );
}

TEST_F(pdHrzTest, SetPrmRad)
{
  pdHrzPrmRad pr;

  pr.vd    = 1;
  pr.q1    = 2;
  pr.q2    = 3;
  pr.kappa = 4;
  pr.rho   = 5;
  pr.kr    = 6;
  pr.dist  = 7;
  pdHrzSetPrm( &rad, &pr );
  EXPECT_EQ( 1, ((pdHrzPrmRad *)pdHrzPrm(&rad))->vd );
  EXPECT_EQ( 2, ((pdHrzPrmRad *)pdHrzPrm(&rad))->q1 );
  EXPECT_EQ( 3, ((pdHrzPrmRad *)pdHrzPrm(&rad))->q2 );
  EXPECT_EQ( 4, ((pdHrzPrmRad *)pdHrzPrm(&rad))->kappa );
  EXPECT_EQ( 5, ((pdHrzPrmRad *)pdHrzPrm(&rad))->rho );
  EXPECT_EQ( 6, ((pdHrzPrmRad *)pdHrzPrm(&rad))->kr );
  EXPECT_EQ( 7, ((pdHrzPrmRad *)pdHrzPrm(&rad))->dist );

  pr.q1 = 8;
  pr.q2 = 9;
  pr.kappa = 10;
  pr.rho   = 11;
  pr.kr    = 12;
  pdHrzSetPrm( &rad, &pr );
  EXPECT_EQ( 1,  ((pdHrzPrmRad *)pdHrzPrm(&rad))->vd );
  EXPECT_EQ( 8,  ((pdHrzPrmRad *)pdHrzPrm(&rad))->q1 );
  EXPECT_EQ( 9,  ((pdHrzPrmRad *)pdHrzPrm(&rad))->q2 );
  EXPECT_EQ( 10, ((pdHrzPrmRad *)pdHrzPrm(&rad))->kappa );
  EXPECT_EQ( 11, ((pdHrzPrmRad *)pdHrzPrm(&rad))->rho );
  EXPECT_EQ( 12, ((pdHrzPrmRad *)pdHrzPrm(&rad))->kr );
  EXPECT_EQ( 7,  ((pdHrzPrmRad *)pdHrzPrm(&rad))->dist );
}

TEST_F(pdHrzTest, HrzK1Tan)
{
  pdHrzPrmTan pt;

  pt.q1 = 1.0;
  pt.q2 = 0.5;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &tan, &pt );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 0.5, pdHrzK1( &tan ) );

  pt.q1 = 0.8;
  pt.q2 = 1.3;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &tan, &pt );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 0.8*1.3, pdHrzK1( &tan ) );
}

TEST_F(pdHrzTest, HrzK2Tan)
{
  pdHrzPrmTan pt;

  pt.q1 = 1.0;
  pt.q2 = 0.5;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &tan, &pt );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 1.5/sqrt(RK_G/0.26), pdHrzK2( &tan ) );

  pt.q1 = 0.8;
  pt.q2 = 1.3;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &tan, &pt );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 2.1/sqrt(RK_G/0.26), pdHrzK2( &tan ) );
}

TEST_F(pdHrzTest, HrzK1Rad)
{
  pdHrzPrmRad pr;

  pr.q1 = 1.0;
  pr.q2 = 0.5;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &rad, &pr );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 0.5, pdHrzK1( &rad ) );

  pr.q1 = 0.8;
  pr.q2 = 1.3;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &rad, &pr );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 0.8*1.3, pdHrzK1( &rad ) );
}

TEST_F(pdHrzTest, HrzK2Rad)
{
  pdHrzPrmRad pr;

  pr.q1 = 1.0;
  pr.q2 = 0.5;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &rad, &pr );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 1.5/sqrt(RK_G/0.26), pdHrzK2( &rad ) );

  pr.q1 = 0.8;
  pr.q2 = 1.3;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &rad, &pr );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 2.1/sqrt(RK_G/0.26), pdHrzK2( &rad ) );
}

TEST_F(pdHrzTest, GetZMP)
{
  ((pdHrzPrmTan *)tan.prm)->uz = 0;
  EXPECT_EQ( 0, pdHrzZMP( &tan ) );
  ((pdHrzPrmTan *)tan.prm)->uz = 0.5;
  EXPECT_EQ( 0.5, pdHrzZMP( &tan ) );

  ((pdHrzPrmRad *)rad.prm)->wz = 0;
  EXPECT_EQ( 0, pdHrzZMP( &rad ) );
  ((pdHrzPrmRad *)rad.prm)->wz = 0.5;
  EXPECT_EQ( 0.5, pdHrzZMP( &rad ) );
}

TEST_F(pdHrzTest, CheckZMPTanAllStateZero)
{
  pdHrzPrmTan pt;

  pt.vd = 0.0;
  pt.q1 = 0.0;
  pt.q2 = 0.0;
  pt.kappa = 0.0;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &tan, &pt );
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &tan, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( 0.0, pdHrzZMP( &tan ) );

  pt.vd = 0.25;
  pt.q1 = 1.0;
  pt.q2 = 0.0;
  pt.kappa = 0.0;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &tan, &pt );
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &tan, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdHrzZMP( &tan ) );
}

TEST_F(pdHrzTest, CheckZMPRadAllStateZero)
{
  pdHrzPrmRad pr;

  pr.vd = 0.0;
  pr.q1 = 1.0;
  pr.q2 = 1.5;
  pr.kappa = 0.0;
  pr.rho = 1.0;
  pr.kr = 1.0;
  pr.dist = 0.1;
  pdVrtSetPrm( &vrt, 0.26 );
  pdHrzSetPrm( &rad, &pr );
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &rad, 0, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( 0.0, pdHrzZMP( &rad ) );
}

TEST_F(pdHrzTest, CheckZMPTanVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &tan, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdHrzZMP( &tan ) );
}

TEST_F(pdHrzTest, CheckZMPRadVelocityFollow)
{
  SetDefaultPrmVelocityFollow();
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &rad, 0.0, 0.2, 0.01, 0.1 );
  // EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdHrzZMP( &rad ) );
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdHrzZMP( &rad ) );
}

TEST_F(pdHrzTest, CheckZMPTanVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &tan, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdHrzZMP( &tan ) );
}

TEST_F(pdHrzTest, CheckZMPRadVelocityFollowCurve)
{
  SetDefaultPrmVelocityFollowCurve();
  pdVrtUpdate( &vrt );
  pdHrzUpdate( &rad, 0.0, 0.2, 0.01, 0.1 );
  // EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdHrzZMP( &rad ) );
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdHrzZMP( &rad ) );
}
