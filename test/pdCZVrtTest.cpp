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

  pdCZVrt vrt;
};

TEST_F(pdCZVrtTest, Init)
{
  pdCZVrtInit( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.q1 );
  EXPECT_EQ( 0, vrt.q2 );
  EXPECT_EQ( 0, vrt.mass );
  EXPECT_EQ( 0, vrt.fz );
  EXPECT_EQ( 0, vrt.acc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, Destroy)
{
  vrt.zd = 1;
  vrt.q1 = 2;
  vrt.q2 = 3;
  vrt.mass  = 4;
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.q1 );
  EXPECT_EQ( 0, vrt.q2 );
  EXPECT_EQ( 0, vrt.mass );
  EXPECT_EQ( 0, vrt.fz );
  EXPECT_EQ( 0, vrt.acc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, SetRef)
{
  pdCZVrtSetRef( &vrt, 1 );
  EXPECT_EQ( 1, vrt.zd );
  pdCZVrtSetRef( &vrt, 10 );
  EXPECT_EQ( 10, vrt.zd );
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
  EXPECT_EQ( 1, vrt.q1 );
  pdCZVrtSetQ1( &vrt, 10 );
  EXPECT_EQ( 10, vrt.q1 );
}

TEST_F(pdCZVrtTest, SetQ2)
{
  pdCZVrtSetQ2( &vrt, 1 );
  EXPECT_EQ( 1, vrt.q2 );
  pdCZVrtSetQ2( &vrt, 10 );
  EXPECT_EQ( 10, vrt.q2 );
}

TEST_F(pdCZVrtTest, SetPrm)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  pdCZVrtSetRef( &vrt, 0.26 );
  EXPECT_EQ( 0.26, vrt.zd );

  pdCZVrtSetRef( &vrt, 0.3 );
  EXPECT_EQ( 0.3, vrt.zd );
}

TEST_F(pdCZVrtTest, Update)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.26, vrt.z );
  EXPECT_EQ( 0.26, vrt.zd );
  EXPECT_EQ( sqrt(RK_G/0.26), vrt.zeta );

  pdCZVrtSetRef( &vrt, 0.3 );
  pdCZVrtUpdate( &vrt );
  EXPECT_EQ( 0.3, vrt.z );
  EXPECT_EQ( 0.3, vrt.zd );
  EXPECT_EQ( sqrt(RK_G/0.3), vrt.zeta );
}
