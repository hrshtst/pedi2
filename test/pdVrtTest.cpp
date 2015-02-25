#include "gtest/gtest.h"
#include <pedi2/pd_vert.h>

TEST(pdVertTest, Init)
{
  pdVert vert;

  pdVertInit( &vert );
  EXPECT_EQ( 0, vert.z );
}

TEST(pdVertTest, UpdateZeta)
{
  pdVert vert;

  pdVertUpdateZeta( &vert, 0.26 );
  EXPECT_EQ( 0.26, vert.z );
  EXPECT_EQ( sqrt(RK_G/0.26), vert.zeta );

  pdVertUpdateZeta( &vert, 0.3 );
  EXPECT_EQ( 0.3, vert.z );
  EXPECT_EQ( sqrt(RK_G/0.3), vert.zeta );
}
