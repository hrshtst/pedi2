#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_sensor.h>

const double TIME_STEP = 0.01;

class pdSensorTest : public testing::Test {
protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdSensor sensor;
  RandomInitializer ri;
};

TEST_F(pdSensorTest, Init)
{
  pdSensorInit( &sensor );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 0, pdSensorSize( &sensor ) );
  EXPECT_EQ( NULL, pdSensorInput( &sensor ) );
  EXPECT_EQ( NULL, pdSensorOutput( &sensor ) );
  EXPECT_EQ( 0, zArrayNum( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( NULL, zArrayBuf( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( NULL, sensor._prm );
  EXPECT_EQ( NULL, sensor._met );
}

TEST_F(pdSensorTest, DestroyDefault)
{
  pdSensorInit( &sensor );
  zNameSet( &sensor, (char*)ZNONAME );
  pdSensorSize( &sensor ) = 3;
  pdSensorInput( &sensor ) = zVecAlloc( 3 );
  pdSensorOutput( &sensor ) = zVecAlloc( 3 );
  zArrayAlloc( &sensor.arr, pdFilter, 2 );
  pdFilterCreateNone( zArrayElem(&sensor.arr,0) );
  pdFilterCreateBW( zArrayElem(&sensor.arr,1), 0.5, 2 );
  zNameSet( zArrayElem(&sensor.arr,0), (char*)"none01" );
  zNameSet( zArrayElem(&sensor.arr,1), (char*)"bw01" );
  zNameSet( &sensor, (char*)ZNONAME );
  pdSensorDestroyDefault( &sensor );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 0, pdSensorSize( &sensor ) );
  EXPECT_EQ( NULL, pdSensorInput( &sensor ) );
  EXPECT_EQ( NULL, pdSensorOutput( &sensor ) );
  EXPECT_EQ( 0, zArrayNum( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( NULL, zArrayBuf( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( NULL, sensor._prm );
  EXPECT_EQ( NULL, sensor._met );
}


class pdSensor6AxisFTTest : public testing::Test {
protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdSensor sensor;
  RandomInitializer ri;
};

TEST_F(pdSensor6AxisFTTest, Create)
{
  zFrame3D frame;
  pdFilterArray arr;

  zFrame3DIdent( &frame );
  pdFilterArrayAlloc( &arr, 6 );
  EXPECT_TRUE( pdSensorCreate6AxisFT( &sensor, &frame, &arr ) );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 6, pdSensorSize( &sensor ) );
  EXPECT_EQ( 6, zVecSize( pdSensorInput( &sensor ) ) );
  EXPECT_EQ( 6, zVecSize( pdSensorOutput( &sensor ) ) );
  EXPECT_EQ( 6, zArrayNum( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( zArrayBuf(&arr), zArrayBuf(pdSensorFilterArray(&sensor)) );
  EXPECT_EQ( &pd_sensor_6axisft_met, sensor._met );
  pdFilterCreateNone( zArrayElem(&sensor.arr,0) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,1) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,2) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,3) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,4) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,5) );
  pdSensorDestroy( &sensor );
}
