#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_sensor.h>

bool zMat3DMatch(zMat3D *m1, zMat3D *m2){
  return m1->c[0] == m2->c[0] &&
         m1->c[1] == m2->c[1] &&
         m1->c[2] == m2->c[2] &&
         m1->c[3] == m2->c[3] &&
         m1->c[4] == m2->c[4] &&
         m1->c[5] == m2->c[5] &&
         m1->c[6] == m2->c[6] &&
         m1->c[7] == m2->c[7] &&
         m1->c[8] == m2->c[8];
};

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
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, zFrame3DPos( pdSensorFrame(&sensor) ) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, zFrame3DAtt( pdSensorFrame(&sensor) ) ) );
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
  virtual void SetUp() {
    pdFilterArrayAlloc( &orgflt, 2 );
    pdFilterCreateNone( zArrayElem(&orgflt,0) );
    pdFilterCreateBW( zArrayElem(&orgflt,1), 0.5, 2 );
    zNameSet( zArrayElem(&orgflt,0), (char*)"none01" );
    zNameSet( zArrayElem(&orgflt,1), (char*)"bw01" );
  };
  virtual void TearDown() {
    pdFilterArrayDestroy( &orgflt );
  };

  pdSensor sensor;
  pdFilterArray orgflt;
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

TEST_F(pdSensor6AxisFTTest, FRead)
{
  char filename[] = "model/sensor_6axisft.conf";
  zVec3D v;
  zMat3D m;
  FILE *fp;

  zVec3DCreate( &v, 1.0, 2.0, 3.0 );
  zMat3DCreate( &m,
                1.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 3.0 );
  fp = fopen( filename, "r" );
  pdSensorFRead( fp, &sensor, &orgflt );
  EXPECT_STREQ( "lf_FT01", zNamePtr( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( &v, zFrame3DPos( pdSensorFrame(&sensor) ) ) );
  EXPECT_TRUE( zMat3DMatch( &m, zFrame3DAtt( pdSensorFrame(&sensor) ) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,0) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,1) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,2) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,3) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,4) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,5) ) );
  pdSensorDestroy( &sensor );
  fclose( fp );
}
