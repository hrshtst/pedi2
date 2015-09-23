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
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, pdSensorPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorAtt(&sensor) ) );
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, pdSensorWldPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorWldAtt(&sensor) ) );
  EXPECT_EQ( 0, zArrayNum( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( NULL, zArrayBuf( pdSensorFilterArray( &sensor ) ) );
  EXPECT_STREQ( "", pdSensorLinkName( &sensor ) );
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

TEST_F(pdSensorTest, ProcessDefault)
{
  zVec input, output;

  input = zVecCreateList( 3, 1.0, 2.0, 3.0 );
  output = zVecAlloc( 3 );
  pdSensorInit( &sensor );
  pdSensorSize( &sensor ) = 3;
  pdSensorInput( &sensor ) = zVecAlloc( 3 );
  pdSensorOutput( &sensor ) = zVecAlloc( 3 );
  pdFilterArrayAlloc( &sensor.arr, 3 );
  pdFilterCreateNone( zArrayElem(&sensor.arr,0) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,1) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,2) );
  zNameSet( zArrayElem(&sensor.arr,0), (char*)"none00" );
  zNameSet( zArrayElem(&sensor.arr,1), (char*)"none01" );
  zNameSet( zArrayElem(&sensor.arr,2), (char*)"none02" );
  zNameSet( &sensor, (char*)ZNONAME );
  // methods to testify
  pdSensorSetInput( &sensor, input );
  pdSensorProcessDefault( &sensor, TIME_STEP );
  pdSensorGetOutput( &sensor, output );
  // test
  EXPECT_EQ( 1.0, pdSensorInputVal(&sensor,0) );
  EXPECT_EQ( 2.0, pdSensorInputVal(&sensor,1) );
  EXPECT_EQ( 3.0, pdSensorInputVal(&sensor,2) );
  EXPECT_EQ( 1.0, pdSensorOutputVal(&sensor,0) );
  EXPECT_EQ( 2.0, pdSensorOutputVal(&sensor,1) );
  EXPECT_EQ( 3.0, pdSensorOutputVal(&sensor,2) );
  EXPECT_EQ( 1.0, zVecElem(output,0) );
  EXPECT_EQ( 2.0, zVecElem(output,1) );
  EXPECT_EQ( 3.0, zVecElem(output,2) );
  pdSensorDestroyDefault( &sensor );
  zVecFree( input );
  zVecFree( output );
}

TEST_F(pdSensorTest, FrameUpdateDefault)
{
  zVec3D v;
  zMat3D m;
  zFrame3D frame;

  // prepare
  pdSensorInit( &sensor );
  zVec3DCreate( &v, 0, 0, 1 );
  zFrame3DCreate( pdSensorLinkFrame(&sensor), &v, ZMAT3DIDENT );
  zVec3DCreate( &v, 0, 0, -2 );
  zMat3DCreate( &m, 0, 1, 0, -1, 0, 0, 0, 0, 1 );
  zFrame3DCreate( &frame, &v, &m );
  // methods to testify
  pdSensorFrameUpdateDefault( &sensor, &frame );
  zVec3DCopy( pdSensorWldPos(&sensor), &v );
  zMat3DCopy( pdSensorWldAtt(&sensor), &m );
  EXPECT_EQ( 0,  zVec3DElem(&v,zX) );
  EXPECT_EQ( 0,  zVec3DElem(&v,zY) );
  EXPECT_EQ( -1, zVec3DElem(&v,zZ) );
  EXPECT_EQ( 0,  zMat3DElem9(&m,0) );
  EXPECT_EQ( -1, zMat3DElem9(&m,1) );
  EXPECT_EQ( 0,  zMat3DElem9(&m,2) );
  EXPECT_EQ( 1,  zMat3DElem9(&m,3) );
  EXPECT_EQ( 0,  zMat3DElem9(&m,4) );
  EXPECT_EQ( 0,  zMat3DElem9(&m,5) );
  EXPECT_EQ( 0,  zMat3DElem9(&m,6) );
  EXPECT_EQ( 0,  zMat3DElem9(&m,7) );
  EXPECT_EQ( 1,  zMat3DElem9(&m,8) );
  // test
  pdSensorDestroyDefault( &sensor );
}


class pdSensor6FTTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterArrayAlloc( &srcfarr, 2 );
    pdFilterCreateNone( zArrayElem(&srcfarr,0) );
    pdFilterCreateBW( zArrayElem(&srcfarr,1), 0.5, 2 );
    zNameSet( zArrayElem(&srcfarr,0), (char*)"none01" );
    zNameSet( zArrayElem(&srcfarr,1), (char*)"bw01" );
  };
  virtual void TearDown() {
    pdFilterArrayDestroy( &srcfarr );
  };

  pdSensor sensor;
  pdFilterArray srcfarr;
  RandomInitializer ri;
};

TEST_F(pdSensor6FTTest, Create)
{
  zFrame3D frame;
  pdFilterArray arr;

  zFrame3DIdent( &frame );
  pdFilterArrayAlloc( &arr, 6 );
  EXPECT_TRUE( pdSensorCreate6FT( &sensor, &frame, &arr ) );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 6, pdSensorSize( &sensor ) );
  EXPECT_EQ( 6, zVecSize( pdSensorInput( &sensor ) ) );
  EXPECT_EQ( 6, zVecSize( pdSensorOutput( &sensor ) ) );
  EXPECT_EQ( 6, zArrayNum( pdSensorFilterArray( &sensor ) ) );
  EXPECT_EQ( zArrayBuf(&arr), zArrayBuf(pdSensorFilterArray(&sensor)) );
  EXPECT_EQ( &pd_sensor_6ft_met, sensor._met );
  pdFilterCreateNone( zArrayElem(&sensor.arr,0) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,1) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,2) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,3) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,4) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,5) );
  pdSensorDestroy( &sensor );
}

TEST_F(pdSensor6FTTest, FRead)
{
  char filename[] = "model/sensor_6ft.conf";
  zVec3D v;
  zMat3D m;
  FILE *fp;

  zVec3DCreate( &v, 1.0, 2.0, 3.0 );
  zMat3DCreate( &m,
                1.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 3.0 );
  fp = fopen( filename, "r" );
  pdSensorFRead( fp, &sensor, &srcfarr );
  EXPECT_STREQ( "lf_FT01", zNamePtr( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( &m, pdSensorAtt(&sensor) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,0) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,1) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,2) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,3) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,4) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,5) ) );
  EXPECT_STREQ( "left_foot", pdSensorLinkName( &sensor ) );
  pdSensorDestroy( &sensor );
  fclose( fp );
}

TEST_F(pdSensor6FTTest, FRead2)
{
  char filename[] = "model/sensor_6ft2.conf";
  FILE *fp;

  fp = fopen( filename, "r" );
  pdSensorFRead( fp, &sensor, &srcfarr );
  EXPECT_STREQ( "lf_FT02", zNamePtr( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, pdSensorPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorAtt(&sensor) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,0) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,1) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,2) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,3) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,4) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,5) ) );
  EXPECT_STREQ( "left_foot", pdSensorLinkName( &sensor ) );
  pdSensorDestroy( &sensor );
  fclose( fp );
}

TEST_F(pdSensor6FTTest, GetFT)
{
  zFrame3D frame;
  pdFilterArray arr;
  zVec v;
  zVec3D f, tau;

  // prepare
  v = zVecCreateList( 6, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 );
  zFrame3DIdent( &frame );
  pdFilterArrayAlloc( &arr, 6 );
  pdSensorCreate6FT( &sensor, &frame, &arr );
  pdFilterCreateNone( zArrayElem(&sensor.arr,0) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,1) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,2) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,3) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,4) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,5) );
  // method to testify
  pdSensorSetInput( &sensor, v );
  pdSensorProcess( &sensor, TIME_STEP );
  pdSensor6FTGetF( &sensor, &f );
  pdSensor6FTGetT( &sensor, &tau );
  // check
  EXPECT_EQ( 1.0, zVec3DElem( &f, zX ) );
  EXPECT_EQ( 2.0, zVec3DElem( &f, zY ) );
  EXPECT_EQ( 3.0, zVec3DElem( &f, zZ ) );
  EXPECT_EQ( 4.0, zVec3DElem( &tau, zX ) );
  EXPECT_EQ( 5.0, zVec3DElem( &tau, zY ) );
  EXPECT_EQ( 6.0, zVec3DElem( &tau, zZ ) );
  pdSensorDestroy( &sensor );
}

TEST_F(pdSensor6FTTest, GetWldFT)
{
  zFrame3D sframe;
  zFrame3D lframe;
  pdFilterArray arr;
  zVec v;
  zVec3D f, tau;

  zVec3D p; zVec3DCreate( &p, 0, 0, -1 );
  zMat3D m; zMat3DCreate( &m, 0, 1, 0, -1, 0, 0, 0, 0, 1 );
  zFrame3DCreate( &lframe, &p, &m );
  // prepare
  v = zVecCreateList( 6, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0 );
  zFrame3DIdent( &sframe );
  pdFilterArrayAlloc( &arr, 6 );
  pdSensorCreate6FT( &sensor, &sframe, &arr );
  pdFilterCreateNone( zArrayElem(&sensor.arr,0) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,1) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,2) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,3) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,4) );
  pdFilterCreateNone( zArrayElem(&sensor.arr,5) );
  // method to testify
  pdSensorSetInput( &sensor, v );
  pdSensorProcess( &sensor, TIME_STEP );
  pdSensorFrameUpdate( &sensor, &lframe );
  pdSensor6FTGetWldF( &sensor, &f );
  pdSensor6FTGetWldT( &sensor, &tau );
  // check
  EXPECT_EQ( 1.0,  zVec3DElem( &f, zX ) );
  EXPECT_EQ( 0.0,  zVec3DElem( &f, zY ) );
  EXPECT_EQ( 1.0,  zVec3DElem( &f, zZ ) );
  EXPECT_EQ( 0.0,  zVec3DElem( &tau, zX ) );
  EXPECT_EQ( -1.0, zVec3DElem( &tau, zY ) );
  EXPECT_EQ( 0.0,  zVec3DElem( &tau, zZ ) );
  pdSensorDestroy( &sensor );
}


class pdSensorArrayTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterArrayAlloc( &srcfarr, 6 );
    pdFilterCreateNone( zArrayElem(&srcfarr,0) );
    pdFilterCreateBW(   zArrayElem(&srcfarr,1), 0.5, 2 );
    pdFilterCreateNone( zArrayElem(&srcfarr,2) );
    pdFilterCreateBW(   zArrayElem(&srcfarr,3), 1.0, 2 );
    pdFilterCreateNone( zArrayElem(&srcfarr,4) );
    pdFilterCreateBW(   zArrayElem(&srcfarr,5), 1.5, 3 );
    zNameSet( zArrayElem(&srcfarr,0), (char*)"none01" );
    zNameSet( zArrayElem(&srcfarr,1), (char*)"bw01" );
    zNameSet( zArrayElem(&srcfarr,2), (char*)"none02" );
    zNameSet( zArrayElem(&srcfarr,3), (char*)"bw02" );
    zNameSet( zArrayElem(&srcfarr,4), (char*)"none03" );
    zNameSet( zArrayElem(&srcfarr,5), (char*)"bw03" );
  };
  virtual void TearDown() {
    pdFilterArrayDestroy( &srcfarr );
  };

  pdSensorArray arr;
  pdFilterArray srcfarr;
  RandomInitializer ri;
};

TEST_F(pdSensorArrayTest, NameFind)
{
  pdSensorArrayAlloc( &arr, 2 );
  pdSensorCreate6FT( zArrayElem(&arr,0), ZFRAME3DIDENT, &srcfarr );
  pdSensorCreate6FT( zArrayElem(&arr,1), ZFRAME3DIDENT, &srcfarr );
  zNameSet( zArrayElem(&arr,0), (char*)"lf_FT01" );
  zNameSet( zArrayElem(&arr,1), (char*)"rf_FT01" );
  EXPECT_EQ( zArrayElem(&arr,0), pdSensorArrayNameFind(&arr,"lf_FT01") );
  EXPECT_EQ( zArrayElem(&arr,1), pdSensorArrayNameFind(&arr,"rf_FT01") );
  // pdSensorArrayDestroy( &arr );
}

TEST_F(pdSensorArrayTest, FRead)
{
  char filename[] = "model/sensor.conf";
  FILE *fp;

  fp = fopen( filename, "r" );
  pdSensorArrayFRead( fp, &arr, &srcfarr );
  EXPECT_STREQ( "lf_FT01", zNamePtr( zArrayElem(&arr,0) ) );
  EXPECT_STREQ( "rf_FT01", zNamePtr( zArrayElem(&arr,1) ) );
  pdSensorArrayDestroy( &arr );
  fclose( fp );
}
