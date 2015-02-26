#include "gtest/gtest.h"
#include <pedi2/pd_vrt.h>

TEST(pdVrtTest, Init)
{
  pdVrt vrt;

  pdVrtInit( &vrt );
  EXPECT_EQ( 0, vrt.z );
}

TEST(pdVrtTest, UpdateZeta)
{
  pdVrt vrt;

  pdVrtInit( &vrt );
  pdVrtUpdateZeta( &vrt, 0.26 );
  EXPECT_EQ( 0.26, vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), vrt.zeta );

  pdVrtUpdateZeta( &vrt, 0.3 );
  EXPECT_EQ( 0.3, vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.3), vrt.zeta );
}
