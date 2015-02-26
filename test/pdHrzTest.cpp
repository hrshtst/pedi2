#include "gtest/gtest.h"
#include <pedi2/pd_vrt.h>
#include <pedi2/pd_hrz.h>

class pdHrzTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdVrtInit( &vrt );
    pdHrzInit( &hrz, &vrt );
    pdVrtUpdateZeta( &vrt, 0.26 );
  };
  virtual void TearDown() {};

  pdVrt vrt;
  pdHrz hrz;
};

TEST_F(pdHrzTest, Init)
{
  pdVrt _vrt;
  pdHrz _hrz;

  pdVrtInit( &_vrt );
  pdHrzInit( &_hrz, &_vrt );
  EXPECT_EQ( 0, _hrz.q1 );
  EXPECT_EQ( 0, _hrz.q2 );
  EXPECT_EQ( 0, _hrz.kappa );
  EXPECT_EQ( 0, _hrz.rho );
  EXPECT_EQ( 0, _hrz.kr );
  EXPECT_EQ( &_vrt, _hrz.vrt );
}

TEST_F(pdHrzTest, ComputeK1)
{
  hrz.q1 = 1.0;
  hrz.q2 = 0.5;
  EXPECT_EQ( 0.5, pdHrzK1( &hrz ) );
  hrz.q1 = 0.8;
  hrz.q2 = 1.3;
  EXPECT_EQ( 0.8*1.3, pdHrzK1( &hrz ) );
}

TEST_F(pdHrzTest, ComputeK2)
{
  hrz.q1 = 1.0;
  hrz.q2 = 0.5;
  EXPECT_EQ( 1.5/vrt.zeta, pdHrzK2( &hrz ) );
  hrz.q1 = 0.8;
  hrz.q2 = 1.3;
  EXPECT_EQ( 2.1/vrt.zeta, pdHrzK2( &hrz ) );
}
