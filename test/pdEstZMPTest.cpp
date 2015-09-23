#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

const double TIME_STEP = 0.01;

class pdEstZMPTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  void SetRandomValues() {
    zArraySetNum( &e._lfsensor, ri.rand() );
    zArraySetNum( &e._rfsensor, ri.rand() );
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
  EXPECT_EQ( NULL, zArrayBuf( &e._lfsensor ) );
  EXPECT_EQ( NULL, zArrayBuf( &e._rfsensor ) );
  EXPECT_EQ( 0, zArrayNum( &e._lfsensor ) );
  EXPECT_EQ( 0, zArrayNum( &e._rfsensor ) );
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
  EXPECT_EQ( NULL, zArrayBuf( &e._lfsensor ) );
  EXPECT_EQ( NULL, zArrayBuf( &e._rfsensor ) );
  EXPECT_EQ( 0, zArrayNum( &e._lfsensor ) );
  EXPECT_EQ( 0, zArrayNum( &e._rfsensor ) );
  EXPECT_EQ( 0, pdEstZMPEstForceX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstForceZ( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPX( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPY( &e ) );
  EXPECT_EQ( 0, pdEstZMPEstZMPZ( &e ) );
}

TEST_F(pdEstZMPTest, FRead)
{
  char filename[] = "model/zmpest.conf";
  FILE *fp;

  fp = fopen( filename, "r" );
  pdEstZMPConfFRead( fp, &e );
  // check filter array
  EXPECT_EQ( 2, zArrayNum( pdEstZMPFilterArray(&e) ) );
  EXPECT_STREQ( "bw01", zNamePtr(zArrayElem(pdEstZMPFilterArray(&e),0)));
  EXPECT_STREQ( "bw02", zNamePtr(zArrayElem(pdEstZMPFilterArray(&e),1)));
  // check sensor array
  EXPECT_EQ( 4, zArrayNum( pdEstZMPSensorArray(&e) ) );
  EXPECT_STREQ( "lf_FT01", zNamePtr(zArrayElem(pdEstZMPSensorArray(&e),0)));
  EXPECT_STREQ( "lf_FT02", zNamePtr(zArrayElem(pdEstZMPSensorArray(&e),1)));
  EXPECT_STREQ( "rf_FT01", zNamePtr(zArrayElem(pdEstZMPSensorArray(&e),2)));
  EXPECT_STREQ( "rf_FT02", zNamePtr(zArrayElem(pdEstZMPSensorArray(&e),3)));
  // check pdEstZMP class
  EXPECT_EQ( 2, zArrayNum( &e._lfsensor ) );
  EXPECT_EQ( 2, zArrayNum( &e._rfsensor ) );
  EXPECT_EQ( zArrayElem(pdEstZMPSensorArray(&e),0), zArrayBuf(&e._lfsensor)[0] );
  EXPECT_EQ( zArrayElem(pdEstZMPSensorArray(&e),1), zArrayBuf(&e._lfsensor)[1] );
  EXPECT_EQ( zArrayElem(pdEstZMPSensorArray(&e),2), zArrayBuf(&e._rfsensor)[0] );
  EXPECT_EQ( zArrayElem(pdEstZMPSensorArray(&e),3), zArrayBuf(&e._rfsensor)[1] );
  pdEstZMPDestroy( &e );
  fclose( fp );
}
