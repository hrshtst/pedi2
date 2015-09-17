#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

class pdEstZMPTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  void SetRandomValues() {
    e_zmp._dt = ri.rand();
  };

  RandomInitializer ri;
  pdEstZMP e_zmp;
};

TEST_F(pdEstZMPTest, Init)
{
  SetRandomValues();
  pdEstZMPInit( &e_zmp );
  EXPECT_EQ( 0, e_zmp._dt );
}

TEST_F(pdEstZMPTest, Destroy)
{
  SetRandomValues();
  pdEstZMPDestroy( &e_zmp );
  EXPECT_EQ( 0, e_zmp._dt );
}
