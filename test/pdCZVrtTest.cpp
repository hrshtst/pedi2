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
    vrt._zd  = 1;
    vrt._q1  = 2;
    vrt._q2  = 3;
    vrt.mass = 4;
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
  EXPECT_EQ( 0, vrt._zd );
  EXPECT_EQ( 0, vrt._q1 );
  EXPECT_EQ( 0, vrt._q2 );
  EXPECT_EQ( 0, vrt.mass );
  EXPECT_EQ( 0, vrt._reffz );
  EXPECT_EQ( 0, vrt._refacc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, Destroy)
{
  SetRandPrm();
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, vrt._zd );
  EXPECT_EQ( 0, vrt._q1 );
  EXPECT_EQ( 0, vrt._q2 );
  EXPECT_EQ( 0, vrt.mass );
  EXPECT_EQ( 0, vrt._reffz );
  EXPECT_EQ( 0, vrt._refacc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, SetRef)
{
  pdCZVrtSetRef( &vrt, 1 );
  EXPECT_EQ( 1, vrt._zd );
  pdCZVrtSetRef( &vrt, 10 );
  EXPECT_EQ( 10, vrt._zd );
}

TEST_F(pdCZVrtTest, SetMass)
{
  pdCZVrtSetMass( &vrt, 1 );
  EXPECT_EQ( 1, vrt.mass );
  pdCZVrtSetMass( &vrt, 10 );
  EXPECT_EQ( 10, vrt.mass );
}

TEST_F(pdCZVrtTest, SetQ1)
{
  pdCZVrtSetQ1( &vrt, 1 );
  EXPECT_EQ( 1, vrt._q1 );
  pdCZVrtSetQ1( &vrt, 10 );
  EXPECT_EQ( 10, vrt._q1 );
}

TEST_F(pdCZVrtTest, SetQ2)
{
  pdCZVrtSetQ2( &vrt, 1 );
  EXPECT_EQ( 1, vrt._q2 );
  pdCZVrtSetQ2( &vrt, 10 );
  EXPECT_EQ( 10, vrt._q2 );
}

TEST_F(pdCZVrtTest, SetPrm)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  pdCZVrtSetRef( &vrt, 0.26 );
  EXPECT_EQ( 0.26, vrt._zd );

  pdCZVrtSetRef( &vrt, 0.3 );
  EXPECT_EQ( 0.3, vrt._zd );
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
