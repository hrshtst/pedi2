#include "gtest/gtest.h"
#include <pedi2/pd_cz_hrz_w.h>

class pdCZHrzWTest : public testing::Test {
protected:
  virtual void SetUp() {
    kappa = 0;
    zeta = 1;
    pdCZHrzWInit( &w, &kappa, &zeta );
  };
  virtual void TearDown() {
    pdCZHrzWDestroy( &w );
  };

  void SetVacuousPrm() {
    pdCZHrzWSetRefVel( &w, 1 );
    pdCZHrzWSetQ1( &w, 2 );
    pdCZHrzWSetQ2( &w, 3 );
    pdCZHrzWSetRho( &w, 4 );
    pdCZHrzWSetKr( &w, 5 );
    pdCZHrzWSetDist( &w, 6 );
    w._kappa = NULL;
    w._zeta = NULL;
  };

  double kappa;
  double zeta;
  pdCZHrzW w;
};

TEST_F(pdCZHrzWTest, Init)
{
  SetVacuousPrm();
  pdCZHrzWInit( &w, &kappa, &zeta );
  EXPECT_EQ( 0, pdCZHrzWRefVel( &w ) );
  EXPECT_EQ( 0, pdCZHrzWQ1( &w ) );
  EXPECT_EQ( 0, pdCZHrzWQ2( &w ) );
  EXPECT_EQ( 0, pdCZHrzWRho( &w ) );
  EXPECT_EQ( 0, pdCZHrzWKr( &w ) );
  EXPECT_EQ( 0, pdCZHrzWDist( &w ) );
  EXPECT_EQ( &kappa, w._kappa );
  EXPECT_EQ( &zeta, w._zeta );
}

TEST_F(pdCZHrzWTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzWDestroy( &w );
  EXPECT_EQ( 0, pdCZHrzWRefVel( &w ) );
  EXPECT_EQ( 0, pdCZHrzWQ1( &w ) );
  EXPECT_EQ( 0, pdCZHrzWQ2( &w ) );
  EXPECT_EQ( 0, pdCZHrzWRho( &w ) );
  EXPECT_EQ( 0, pdCZHrzWKr( &w ) );
  EXPECT_EQ( 0, pdCZHrzWDist( &w ) );
  EXPECT_EQ( NULL, w._kappa );
  EXPECT_EQ( NULL, w._zeta );
}

TEST_F(pdCZHrzWTest, SetRefVel)
{
  pdCZHrzWSetRefVel( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWRefVel( &w ) );
  SetVacuousPrm();
  pdCZHrzWSetRefVel( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWRefVel( &w ) );
}

TEST_F(pdCZHrzWTest, SetQ1)
{
  pdCZHrzWSetQ1( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWQ1( &w ) );
  SetVacuousPrm();
  pdCZHrzWSetQ1( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWQ1( &w ) );
}

TEST_F(pdCZHrzWTest, SetQ2)
{
  pdCZHrzWSetQ2( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWQ2( &w ) );
  SetVacuousPrm();
  pdCZHrzWSetQ2( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWQ2( &w ) );
}

TEST_F(pdCZHrzWTest, SetRho)
{
  pdCZHrzWSetRho( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWRho( &w ) );
  SetVacuousPrm();
  pdCZHrzWSetRho( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWRho( &w ) );
}

TEST_F(pdCZHrzWTest, SetKr)
{
  pdCZHrzWSetKr( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWKr( &w ) );
  SetVacuousPrm();
  pdCZHrzWSetKr( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWKr( &w ) );
}

TEST_F(pdCZHrzWTest, SetDist)
{
  pdCZHrzWSetDist( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWDist( &w ) );
  SetVacuousPrm();
  pdCZHrzWSetDist( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWDist( &w ) );
}

TEST_F(pdCZHrzWTest, SetKappa)
{
  pdCZHrzWSetKappa( &w, 1 );
  EXPECT_EQ( 1, pdCZHrzWKappa( &w ) );
  pdCZHrzWSetKappa( &w, 10 );
  EXPECT_EQ( 10, pdCZHrzWKappa( &w ) );
  kappa = 100;
  EXPECT_EQ( 100, pdCZHrzWKappa( &w ) );
}

TEST_F(pdCZHrzWTest, SetZeta)
{
  zeta = 1;
  EXPECT_EQ( 1, pdCZHrzWZeta( &w ) );
  zeta = 10;
  EXPECT_EQ( 10, pdCZHrzWZeta( &w ) );
}

TEST_F(pdCZHrzWTest, SetPrm)
{
  pdCZHrzWSetPrm( &w, 0.2, 1.0, 0.5, 1.0, 1.0, 0.15 );
  EXPECT_EQ( 0.2, pdCZHrzWRefVel( &w ) );
  EXPECT_EQ( 1.0, pdCZHrzWQ1( &w ) );
  EXPECT_EQ( 0.5, pdCZHrzWQ2( &w ) );
  EXPECT_EQ( 1.0, pdCZHrzWRho( &w ) );
  EXPECT_EQ( 1.0, pdCZHrzWKr( &w ) );
  EXPECT_EQ( 0.15, pdCZHrzWDist( &w ) );
}

TEST_F(pdCZHrzWTest, CalcSimZMP)
{
  zeta = 1.0;
  kappa = 0.0;
  pdCZHrzWSetPrm( &w, 0, 1, 1, 0, 0, 1 );
  EXPECT_DOUBLE_EQ( 0, pdCZHrzWCalcSimZMP( &w, 0, 0, 0, 0 ) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzWCalcSimZMP( &w, 0.1, 0.1, 0.1, 0.1 ) );
  EXPECT_DOUBLE_EQ( -0.2, pdCZHrzWCalcSimZMP( &w, 0.2, -0.2, 0.2, -0.2 ) );
  pdCZHrzWSetPrm( &w, 0.2, 1, 1, 1, 1, 1 );
  EXPECT_NEAR( -0.0947755464461, pdCZHrzWCalcSimZMP( &w, 0.1, 0.1, 0.1, 0.1 ), 1e-04 );
  EXPECT_NEAR( 0.102313496582, pdCZHrzWCalcSimZMP( &w, 0.2, -0.2, 0.2, -0.2 ), 1e-04 );
  kappa = 1.0;
  pdCZHrzWSetPrm( &w, 0.2, 1, 1, 1, 1, 1 );
  EXPECT_NEAR( -0.103866455537, pdCZHrzWCalcSimZMP( &w, 0.1, 0.1, 0.1, 0.1 ), 1e-04 );
  EXPECT_NEAR( 0.068980163249, pdCZHrzWCalcSimZMP( &w, 0.2, -0.2, 0.2, -0.2 ), 1e-04 );
}

TEST_F(pdCZHrzWTest, CalcRegZMP)
{
  zeta = 1.0;
  kappa = 0.0;
  pdCZHrzWSetPrm( &w, 0, 1, 1, 0, 0, 1 );
  EXPECT_DOUBLE_EQ( 0, pdCZHrzWCalcRegZMP( &w, 0, 0, 0, 0 ) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzWCalcRegZMP( &w, 0.1, 0.1, 0.1, 0.1 ) );
  EXPECT_DOUBLE_EQ( -0.2, pdCZHrzWCalcRegZMP( &w, 0.2, -0.2, 0.2, -0.2 ) );
  pdCZHrzWSetPrm( &w, 0.2, 1, 1, 1, 1, 1 );
  EXPECT_NEAR( 0.3, pdCZHrzWCalcRegZMP( &w, 0.1, 0.1, 0.1, 0.1 ), 1e-04 );
  EXPECT_NEAR( -0.2, pdCZHrzWCalcRegZMP( &w, 0.2, -0.2, 0.2, -0.2 ), 1e-04 );
  kappa = 1.0;
  pdCZHrzWSetPrm( &w, 0.2, 1, 1, 1, 1, 1 );
  EXPECT_NEAR( 0.290909090909, pdCZHrzWCalcRegZMP( &w, 0.1, 0.1, 0.1, 0.1 ), 1e-04 );
  EXPECT_NEAR( -0.233333333333, pdCZHrzWCalcRegZMP( &w, 0.2, -0.2, 0.2, -0.2 ), 1e-04 );
}
