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
    u._vd = 1;
    u._q1 = 2;
    u._q2 = 3;
    u._kappa = NULL;
    u._zeta = NULL;
    u.refzmp = 4;
    u.refacc = 5;
  };

  double kappa;
  double zeta;
  pdCZHrzU u;
};

TEST_F(pdCZHrzUTest, Init)
{
  SetVacuousPrm();
  pdCZHrzUInit( &u, &kappa, &zeta );
  EXPECT_EQ( 0, u._vd );
  EXPECT_EQ( 0, u._q1 );
  EXPECT_EQ( 0, u._q2 );
  EXPECT_EQ( &kappa, u._kappa );
  EXPECT_EQ( &zeta, u._zeta );
  EXPECT_EQ( 0, u.refzmp );
  EXPECT_EQ( 0, u.refacc );
}

TEST_F(pdCZHrzUTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzUDestroy( &u );
  EXPECT_EQ( 0, u._vd );
  EXPECT_EQ( 0, u._q1 );
  EXPECT_EQ( 0, u._q2 );
  EXPECT_EQ( NULL, u._kappa );
  EXPECT_EQ( NULL, u._zeta );
  EXPECT_EQ( 0, u.refzmp );
  EXPECT_EQ( 0, u.refacc );
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
