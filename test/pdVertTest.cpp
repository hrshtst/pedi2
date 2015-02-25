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
  EXPECT_EQ( sqrt(RK_G/0.26), vert.zeta );
}



// TEST(pdVertTest, UpdateVert)
// {
//   pdVert vert;

//   vert.z = 0.26;
// }
