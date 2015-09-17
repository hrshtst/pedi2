#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

const double TIME_STEP = 0.01;

class pdEstZMPTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdEstZMPInit( &e_zmp, TIME_STEP );
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
  pdEstZMPInit( &e_zmp, 0.02 );
  EXPECT_EQ( 0, pdEstZMPTime(&e_zmp) );
  EXPECT_EQ( 0.02, pdEstZMPTimeStep(&e_zmp) );
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
  pdEstZMPSetTimeStep( &e_zmp, 0.001 );
  EXPECT_EQ( 0.001, pdEstZMPTimeStep(&e_zmp) );
}

TEST_F(pdEstZMPTest, ConfReadFile)
{
  char config[] = "model/sensor.conf";

  zEchoOff();
  EXPECT_TRUE( pdEstZMPConfReadFile( &e_zmp, config ) );
  zEchoOn();
}

TEST_F(pdEstZMPTest, ConfReadFile_ReadValue)
{
  char config[] = "model/sensor.conf";

  zEchoOff();
  pdEstZMPConfReadFile( &e_zmp, config );
  EXPECT_STREQ( "bwf01", e_zmp.filter[0].name );
  EXPECT_STREQ( "lf_FT01", e_zmp.sensor[0].name );
  zEchoOn();
}

TEST_F(pdEstZMPTest, ConfReadFile_)
{}

// TEST_F(pdEstZMPTest, ConfReadFile_)
// {}

