#include "gtest/gtest.h"
#include <pedi2/pd_vrt.h>

TEST(pdVrtTest, Init)
{
  pdVrt vrt;

  pdVrtInit( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.z );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST(pdVrtTest, Destroy)
{
  pdVrt vrt;

  pdVrtInit( &vrt );
  pdVrtDestroy( &vrt );
  EXPECT_EQ( 0, vrt.zd );
  EXPECT_EQ( 0, vrt.z );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST(pdVrtTest, SetPrm)
{
  pdVrt vrt;

  pdVrtInit( &vrt );
  pdVrtSetRef( &vrt, 0.26 );
  EXPECT_EQ( 0.26, vrt.zd );

  pdVrtSetRef( &vrt, 0.3 );
  EXPECT_EQ( 0.3, vrt.zd );
}

TEST(pdVrtTest, Update)
{
  pdVrt vrt;

  pdVrtInit( &vrt );
  pdVrtSetRef( &vrt, 0.26 );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 0.26, vrt.z );
  EXPECT_EQ( 0.26, vrt.zd );
  EXPECT_EQ( sqrt(RK_G/0.26), vrt.zeta );

  pdVrtSetRef( &vrt, 0.3 );
  pdVrtUpdate( &vrt );
  EXPECT_EQ( 0.3, vrt.z );
  EXPECT_EQ( 0.3, vrt.zd );
  EXPECT_EQ( sqrt(RK_G/0.3), vrt.zeta );
}
