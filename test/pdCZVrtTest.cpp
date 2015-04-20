#include "gtest/gtest.h"
#include <pedi2/pd_cz_vrt.h>

class pdCZVrtTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
  };

  virtual void TearDown() {
    pdCZVrtDestroy( &vrt );
  };

  void SetRandPrm() {
    pdCZVrtSetRef( &vrt, 1 );
    pdCZVrtSetQ1( &vrt, 3 );
    pdCZVrtSetQ2( &vrt, 4 );
    pdCZVrtSetMass( &vrt, 2 );
    vrt._reffz = 5;
    vrt._refacc = 6;
    vrt.zeta = 7;
  };

  pdCZVrt vrt;
};

TEST_F(pdCZVrtTest, Init)
{
  SetRandPrm();
  pdCZVrtInit( &vrt );
  EXPECT_EQ( 0, pdCZVrtGetRef( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtGetQ1( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtGetQ2( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtGetMass( &vrt ) );
  EXPECT_EQ( 0, vrt._reffz );
  EXPECT_EQ( 0, vrt._refacc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, Destroy)
{
  SetRandPrm();
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, pdCZVrtGetRef( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtGetQ1( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtGetQ2( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtGetMass( &vrt ) );
  EXPECT_EQ( 0, vrt._reffz );
  EXPECT_EQ( 0, vrt._refacc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, SetRef)
{
  pdCZVrtSetRef( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtGetRef( &vrt ) );
  pdCZVrtSetRef( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtGetRef( &vrt ) );
}

TEST_F(pdCZVrtTest, SetQ1)
{
  pdCZVrtSetQ1( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtGetQ1( &vrt ) );
  pdCZVrtSetQ1( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtGetQ1( &vrt ) );
}

TEST_F(pdCZVrtTest, SetQ2)
{
  pdCZVrtSetQ2( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtGetQ2( &vrt ) );
  pdCZVrtSetQ2( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtGetQ2( &vrt ) );
}

TEST_F(pdCZVrtTest, SetMass)
{
  pdCZVrtSetMass( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtGetMass( &vrt ) );
  pdCZVrtSetMass( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtGetMass( &vrt ) );
}

TEST_F(pdCZVrtTest, SetPrm)
{
  pdCZVrtInit( &vrt );
  pdCZVrtSetPrm( &vrt, 0.26, 1.0, 0.5, 5.5 );
  EXPECT_EQ( 0.26, pdCZVrtGetRef( &vrt ) );
  EXPECT_EQ( 1.0, pdCZVrtGetQ1( &vrt ) );
  EXPECT_EQ( 0.5, pdCZVrtGetQ2( &vrt ) );
  EXPECT_EQ( 5.5, pdCZVrtGetMass( &vrt ) );
}

TEST_F(pdCZVrtTest, Update)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.26, vrt.z );
  EXPECT_EQ( 0.26, vrt._zd );
  EXPECT_EQ( sqrt(RK_G/0.26), vrt.zeta );

  pdCZVrtSetRef( &vrt, 0.3 );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.3, vrt.z );
  EXPECT_EQ( 0.3, vrt._zd );
  EXPECT_EQ( sqrt(RK_G/0.3), vrt.zeta );
}
