#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

class pdEstZMPTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdEstZMPInit( &e_zmp );
  };
  virtual void TearDown() {};

  void SetRandomValues() {
    e_zmp._t = ri.rand();
    e_zmp._dt = ri.rand();
  };

  RandomInitializer ri;
  pdEstZMP e_zmp;
};

TEST_F(pdEstZMPTest, Init)
{
  SetRandomValues();
  pdEstZMPInit( &e_zmp );
  EXPECT_EQ( 0, pdEstZMPTime(&e_zmp) );
  EXPECT_EQ( 0, pdEstZMPTimeStep(&e_zmp) );
}

TEST_F(pdEstZMPTest, Destroy)
{
  SetRandomValues();
  pdEstZMPDestroy( &e_zmp );
  EXPECT_EQ( 0, pdEstZMPTime(&e_zmp) );
  EXPECT_EQ( 0, pdEstZMPTimeStep(&e_zmp) );
}

TEST_F(pdEstZMPTest, SetTime)
{
  SetRandomValues();
  pdEstZMPSetTime( &e_zmp, 0.5 );
  EXPECT_EQ( 0.5, pdEstZMPTime(&e_zmp) );
}

TEST_F(pdEstZMPTest, SetTimeStep)
{
  SetRandomValues();
  pdEstZMPSetTimeStep( &e_zmp, 0.5 );
  EXPECT_EQ( 0.5, pdEstZMPTimeStep(&e_zmp) );
}

