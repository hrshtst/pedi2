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
  EXPECT_EQ( 0, vrt.m );
  EXPECT_EQ( 0, vrt.fz );
  EXPECT_EQ( 0, vrt.acc );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, Destroy)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  vrt.zd = 1;
  vrt.q1 = 2;
  vrt.q2 = 3;
  vrt.m  = 4;
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.q1 );
  EXPECT_EQ( 0, vrt.q2 );
  EXPECT_EQ( 0, vrt.m );
  EXPECT_EQ( 0, vrt.fz );
  EXPECT_EQ( 0, vrt.acc );
  EXPECT_EQ( 0, vrt.zeta );
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
