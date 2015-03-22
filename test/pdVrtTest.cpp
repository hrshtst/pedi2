#include "gtest/gtest.h"
#include <pedi2/pd_cz_vrt.h>

TEST(pdCZVrtTest, Init)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.z );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST(pdCZVrtTest, Destroy)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.z );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST(pdCZVrtTest, SetPrm)
{
  pdCZVrt vrt;

  pdCZVrtInit( &vrt );
  pdCZVrtSetRef( &vrt, 0.26 );
  EXPECT_EQ( 0.26, vrt.zd );

  pdCZVrtSetRef( &vrt, 0.3 );
  EXPECT_EQ( 0.3, vrt.zd );
}

TEST(pdCZVrtTest, Update)
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
