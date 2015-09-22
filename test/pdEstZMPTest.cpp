#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

const double TIME_STEP = 0.01;

class pdEstZMPTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  void SetRandomValues() {
    ri.SetRandVec3D( &e.estforce );
    ri.SetRandVec3D( &e.estzmp );
  }

  RandomInitializer ri;
  pdEstZMP e;
};

TEST_F(pdEstZMPTest, Init)
{
  SetRandomValues();
  pdEstZMPInit( &e );
  EXPECT_EQ( 0, pdEstZMPEstForceX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceZ( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPZ( &e ) );
}
