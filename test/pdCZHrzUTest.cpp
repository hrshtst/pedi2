#include "gtest/gtest.h"
#include <pedi2/pd_cz_hrz_u.h>

class pdCZHrzUTest : public testing::Test {
protected:
  virtual void SetUp() {
    kappa = 0;
    zeta = 1;
    pdCZHrzUInit( &u, &kappa, &zeta );
  };
  virtual void TearDown() {
    pdCZHrzUDestroy( &u );
  };

  void SetVacuousPrm() {
    pdCZHrzUSetRefVel( &u, 1 );
    pdCZHrzUSetQ1( &u, 2 );
    pdCZHrzUSetQ2( &u, 3 );
    u._kappa = NULL;
    u._zeta = NULL;
  };

  double kappa;
  double zeta;
  pdCZHrzU u;
};

TEST_F(pdCZHrzUTest, Init)
{
  SetVacuousPrm();
  pdCZHrzUInit( &u, &kappa, &zeta );
  EXPECT_EQ( 0, pdCZHrzURefVel( &u ) );
  EXPECT_EQ( 0, pdCZHrzUQ1( &u ) );
  EXPECT_EQ( 0, pdCZHrzUQ2( &u ) );
  EXPECT_EQ( &kappa, u._kappa );
  EXPECT_EQ( &zeta, u._zeta );
}

TEST_F(pdCZHrzUTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzUDestroy( &u );
  EXPECT_EQ( 0, pdCZHrzURefVel( &u ) );
  EXPECT_EQ( 0, pdCZHrzUQ1( &u ) );
  EXPECT_EQ( 0, pdCZHrzUQ2( &u ) );
  EXPECT_EQ( NULL, u._kappa );
  EXPECT_EQ( NULL, u._zeta );
}

TEST_F(pdCZHrzUTest, SetRefVel)
{
  pdCZHrzUSetRefVel( &u, 1 );
  EXPECT_EQ( 1, pdCZHrzURefVel( &u ) );
  SetVacuousPrm();
  pdCZHrzUSetRefVel( &u, 10 );
  EXPECT_EQ( 10, pdCZHrzURefVel( &u ) );
}

TEST_F(pdCZHrzUTest, SetQ1)
{
  pdCZHrzUSetQ1( &u, 1 );
  EXPECT_EQ( 1, pdCZHrzUQ1( &u ) );
  SetVacuousPrm();
  pdCZHrzUSetQ1( &u, 10 );
  EXPECT_EQ( 10, pdCZHrzUQ1( &u ) );
}

TEST_F(pdCZHrzUTest, SetQ2)
{
  pdCZHrzUSetQ2( &u, 1 );
  EXPECT_EQ( 1, pdCZHrzUQ2( &u ) );
  SetVacuousPrm();
  pdCZHrzUSetQ2( &u, 10 );
  EXPECT_EQ( 10, pdCZHrzUQ2( &u ) );
}

TEST_F(pdCZHrzUTest, SetKappa)
{
  pdCZHrzUSetKappa( &u, 1 );
  EXPECT_EQ( 1, pdCZHrzUKappa( &u ) );
  pdCZHrzUSetKappa( &u, 10 );
  EXPECT_EQ( 10, pdCZHrzUKappa( &u ) );
  kappa = 100;
  EXPECT_EQ( 100, pdCZHrzUKappa( &u ) );
}

TEST_F(pdCZHrzUTest, SetZeta)
{
  zeta = 1;
  EXPECT_EQ( 1, pdCZHrzUZeta( &u ) );
  zeta = 10;
  EXPECT_EQ( 10, pdCZHrzUZeta( &u ) );
}

TEST_F(pdCZHrzUTest, SetPrm)
{
  pdCZHrzUSetPrm( &u, 0.2, 1.0, 0.5 );
  EXPECT_EQ( 0.2, pdCZHrzURefVel( &u ) );
  EXPECT_EQ( 1.0, pdCZHrzUQ1( &u ) );
  EXPECT_EQ( 0.5, pdCZHrzUQ2( &u ) );
}

TEST_F(pdCZHrzUTest, CalcSimZMP)
{
  zeta = 1.0;
  kappa = 0.0;
  pdCZHrzUSetPrm( &u, 0, 1, 1 );
  EXPECT_DOUBLE_EQ( 0, pdCZHrzUCalcSimZMP( &u, 0, 0, 0, 0 ) );
  EXPECT_DOUBLE_EQ( 0.8, pdCZHrzUCalcSimZMP( &u, 0.2, 0.5, 0, 0 ) );
  EXPECT_DOUBLE_EQ( -1.2, pdCZHrzUCalcSimZMP( &u, 0.8, -0.2, 0, 0 ) );
  pdCZHrzUSetPrm( &u, 0.2, 1, 1 );
  EXPECT_NEAR( 0, pdCZHrzUCalcSimZMP( &u, 0.2, 0.3, 0, 0 ), 1e-04 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzUCalcSimZMP( &u, -0.1, 0.2, 0, 0 ) );
  kappa = 1.0;
  pdCZHrzUSetPrm( &u, 0.2, 1, 1 );
  EXPECT_DOUBLE_EQ( -0.18, pdCZHrzUCalcSimZMP( &u, 0.2, 0.2, -0.2, -0.2 ) );
  EXPECT_DOUBLE_EQ(  0.34, pdCZHrzUCalcSimZMP( &u, -0.2, 0.2, -0.2, 0.2 ) );
}

TEST_F(pdCZHrzUTest, CalcRegZMP)
{
  zeta = 1.0;
  kappa = 0.0;
  pdCZHrzUSetPrm( &u, 0, 1, 1 );
  EXPECT_DOUBLE_EQ( 0, pdCZHrzUCalcRegZMP( &u, 0, 0, 0, 0 ) );
  EXPECT_DOUBLE_EQ( 0.8, pdCZHrzUCalcRegZMP( &u, 0.2, 0.5, 0, 0 ) );
  EXPECT_DOUBLE_EQ( -1.2, pdCZHrzUCalcRegZMP( &u, 0.8, -0.2, 0, 0 ) );
  pdCZHrzUSetPrm( &u, 0.2, 1, 1 );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzUCalcRegZMP( &u, 0.2, 0.3, 0, 0 ) );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzUCalcRegZMP( &u, -0.1, 0.2, 0, 0 ) );
  kappa = 1.0;
  pdCZHrzUSetPrm( &u, 0.2, 1, 1 );
  EXPECT_DOUBLE_EQ( 0.14, pdCZHrzUCalcRegZMP( &u, 0.2, 0.2, -0.2, -0.2 ) );
  EXPECT_DOUBLE_EQ( 0.66, pdCZHrzUCalcRegZMP( &u, -0.2, 0.2, -0.2, 0.2 ) );
}
