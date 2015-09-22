#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

const double TIME_STEP = 0.01;

class pdEstZMPTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  void SetRandomValues() {
    e._lfsensor_num = ri.rand();
    e._rfsensor_num = ri.rand();
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
  EXPECT_EQ( NULL, zNamePtr( &e ) );
  EXPECT_EQ( NULL, e._lfsensor );
  EXPECT_EQ( NULL, e._rfsensor );
  EXPECT_EQ( 0, e._lfsensor_num );
  EXPECT_EQ( 0, e._rfsensor_num );
  EXPECT_EQ( 0, pdEstZMPEstForceX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceZ( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPZ( &e ) );
}

TEST_F(pdEstZMPTest, Destroy)
{
  pdEstZMPInit( &e );
  SetRandomValues();
  pdEstZMPDestroy( &e );
  EXPECT_EQ( NULL, zNamePtr( &e ) );
  EXPECT_EQ( NULL, e._lfsensor );
  EXPECT_EQ( NULL, e._rfsensor );
  EXPECT_EQ( 0, e._lfsensor_num );
  EXPECT_EQ( 0, e._rfsensor_num );
  EXPECT_EQ( 0, pdEstZMPEstForceX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceZ( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPZ( &e ) );
}
