#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_sensor.h>

const double TIME_STEP = 0.01;

// These definitions should be removed by using CPP version library
const zVec3D zVec3D::zvec3Dzero = { { 0, 0, 0 } };
const zMat3D zMat3D::zmat3Dident = { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } };
const zFrame3D zFrame3D::zframe3Dident = {
  { { 0, 0, 0 } },
  { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } }
};

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
  EXPECT_EQ( NULL, pdSensorRawData( &sensor ) );
  EXPECT_EQ( NULL, pdSensorData( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, pdSensorLinkPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorLinkAtt(&sensor) ) );
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, pdSensorWldPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorWldAtt(&sensor) ) );
  EXPECT_EQ( 0, zArraySize( pdSensorFilters( &sensor ) ) );
  EXPECT_EQ( NULL, zArrayBuf( pdSensorFilters( &sensor ) ) );
  EXPECT_STREQ( "", pdSensorLinkName( &sensor ) );
  EXPECT_EQ( NULL, sensor.prp );
  EXPECT_EQ( NULL, sensor.com );
}

TEST_F(pdSensorTest, DefaultDestroy)
{
  pdSensorInit( &sensor );
  zNameSet( &sensor, ZNONAME );
  pdSensorSize( &sensor ) = 3;
  pdSensorRawData( &sensor ) = zVecAlloc( 3 );
  pdSensorData( &sensor ) = zVecAlloc( 3 );
  zArrayAlloc( pdSensorFilters(&sensor), pdFilter, 2 );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),0) );
  pdFilterBWCreate( zArrayElem(pdSensorFilters(&sensor),1), 0.5, 2 );
  zNameSet( zArrayElem(pdSensorFilters(&sensor),0), "none01" );
  zNameSet( zArrayElem(pdSensorFilters(&sensor),1), "bw01" );
  zNameSet( &sensor, ZNONAME );
  pdSensorDefaultDestroy( &sensor );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 0, pdSensorSize( &sensor ) );
  EXPECT_EQ( NULL, pdSensorRawData( &sensor ) );
  EXPECT_EQ( NULL, pdSensorData( &sensor ) );
  EXPECT_EQ( 0, zArraySize( pdSensorFilters( &sensor ) ) );
  EXPECT_EQ( NULL, zArrayBuf( pdSensorFilters( &sensor ) ) );
  EXPECT_EQ( NULL, sensor.prp );
  EXPECT_EQ( NULL, sensor.com );
}

TEST_F(pdSensorTest, DefaultProcess)
{
  zVec raw, data;

  raw = zVecCreateList( 3, 1.0, 2.0, 3.0 );
  data = zVecAlloc( 3 );
  pdSensorInit( &sensor );
  pdSensorSize( &sensor ) = 3;
  pdSensorRawData( &sensor ) = zVecAlloc( 3 );
  pdSensorData( &sensor ) = zVecAlloc( 3 );
  pdFilterArrayAlloc( pdSensorFilters(&sensor), 3 );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),0) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),1) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),2) );
  zNameSet( zArrayElem(pdSensorFilters(&sensor),0), "none00" );
  zNameSet( zArrayElem(pdSensorFilters(&sensor),1), "none01" );
  zNameSet( zArrayElem(pdSensorFilters(&sensor),2), "none02" );
  zNameSet( &sensor, ZNONAME );
  // methods to testify
  pdSensorSetRawData( &sensor, raw );
  pdSensorDefaultProcess( &sensor, TIME_STEP );
  pdSensorGetData( &sensor, data );
  // test
  EXPECT_EQ( 1.0, pdSensorRawDataVal(&sensor,0) );
  EXPECT_EQ( 2.0, pdSensorRawDataVal(&sensor,1) );
  EXPECT_EQ( 3.0, pdSensorRawDataVal(&sensor,2) );
  EXPECT_EQ( 1.0, pdSensorDataVal(&sensor,0) );
  EXPECT_EQ( 2.0, pdSensorDataVal(&sensor,1) );
  EXPECT_EQ( 3.0, pdSensorDataVal(&sensor,2) );
  EXPECT_EQ( 1.0, zVecElem(data,0) );
  EXPECT_EQ( 2.0, zVecElem(data,1) );
  EXPECT_EQ( 3.0, zVecElem(data,2) );
  pdSensorDefaultDestroy( &sensor );
  zVecFree( raw );
  zVecFree( data );
}

TEST_F(pdSensorTest, DefaultFrameUpdate)
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
  pdSensorDefaultFrameUpdate( &sensor, &frame );
  zVec3DCopy( pdSensorWldPos(&sensor), &v );
  zMat3DCopy( pdSensorWldAtt(&sensor), &m );
  // test
  EXPECT_EQ( 0,  v.c.x );
  EXPECT_EQ( 0,  v.c.y );
  EXPECT_EQ( -1, v.c.z );
  EXPECT_EQ( 0,  m.e[0][0] );
  EXPECT_EQ( -1, m.e[0][1] );
  EXPECT_EQ( 0,  m.e[0][2] );
  EXPECT_EQ( 1,  m.e[1][0] );
  EXPECT_EQ( 0,  m.e[1][1] );
  EXPECT_EQ( 0,  m.e[1][2] );
  EXPECT_EQ( 0,  m.e[2][0] );
  EXPECT_EQ( 0,  m.e[2][1] );
  EXPECT_EQ( 1,  m.e[2][2] );
  pdSensorDefaultDestroy( &sensor );
}


class pdSensorDummyTest : public testing::Test {
protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdSensor sensor;
  RandomInitializer ri;
};

TEST_F(pdSensorDummyTest, Create)
{
  zFrame3D frame;

  zFrame3DIdent( &frame );
  pdSensorDummyCreate( &sensor, "foot", &frame );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 0, pdSensorSize( &sensor ) );
  EXPECT_EQ( 0, zVecSize( pdSensorRawData( &sensor ) ) );
  EXPECT_EQ( 0, zVecSize( pdSensorData( &sensor ) ) );
  EXPECT_EQ( &pd_sensor_dummy_com, sensor.com );
  EXPECT_STREQ( "dummy", sensor.com->typestr );
  pdSensorDestroy( &sensor );
}

TEST_F(pdSensorDummyTest, FromZTK)
{
  char filename[] = "model/sensor_dummy.ztk";
  ZTK ztk;
  zVec3D v;
  zMat3D m;

  zVec3DCreate( &v, 1.0, 2.0, 3.0 );
  zMat3DCreate( &m,
                1.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 3.0 );
  ZTKParse( &ztk, filename );
  pdSensorFromZTK( &sensor, NULL, &ztk );
  EXPECT_STREQ( "dummy01", zNamePtr( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorLinkPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( &m, pdSensorLinkAtt(&sensor) ) );
  EXPECT_STREQ( "dummy", sensor.com->typestr );
  pdSensorDestroy( &sensor );
  ZTKDestroy( &ztk );
}

TEST_F(pdSensorDummyTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  zFrame3D frame;
  pdFilterArray arr;
  char linkname[] = "foot";
  char name[] = "dummy01";
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  zFrame3DIdent( &frame );
  pdFilterArrayAlloc( &arr, 6 );
  pdSensorDummyCreate( &sensor, linkname, &frame );
  zNameSet( &sensor, name );
  sprintf( expected,
           "name: %s\ntype: dummy\nlink: %s\nframe: {\n 1, 0, 0, 0\n 0, 1, 0, 0\n 0, 0, 1, 0\n}\n",
           name, linkname );

  pdSensorFPrintZTK( fp, &sensor );
  fflush( fp );
  EXPECT_STREQ( expected, buf );

  pdSensorDestroy( &sensor );
  fclose( fp );
}


class pdSensor6FTTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterArrayAlloc( &filterarray, 2 );
    pdFilterNoneCreate( zArrayElem(&filterarray,0) );
    pdFilterBWCreate( zArrayElem(&filterarray,1), 0.5, 2 );
    zNameSet( zArrayElem(&filterarray,0), "none01" );
    zNameSet( zArrayElem(&filterarray,1), "bw01" );
  };
  virtual void TearDown() {
    pdFilterArrayDestroy( &filterarray );
  };

  pdSensor sensor;
  pdFilterArray filterarray;
  RandomInitializer ri;
};

TEST_F(pdSensor6FTTest, Create)
{
  zFrame3D frame;
  pdFilterArray arr;

  zFrame3DIdent( &frame );
  pdFilterArrayAlloc( &arr, 6 );
  pdSensor6FTCreate( &sensor, "foot", &frame, &arr );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( 6, pdSensorSize( &sensor ) );
  EXPECT_EQ( 6, zVecSize( pdSensorRawData( &sensor ) ) );
  EXPECT_EQ( 6, zVecSize( pdSensorData( &sensor ) ) );
  EXPECT_EQ( 6, zArraySize( pdSensorFilters( &sensor ) ) );
  EXPECT_EQ( zArrayBuf(&arr), zArrayBuf(pdSensorFilters(&sensor)) );
  EXPECT_EQ( &pd_sensor_6ft_com, sensor.com );
  EXPECT_STREQ( "6ft", sensor.com->typestr );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),0) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),1) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),2) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),3) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),4) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),5) );
  pdSensorDestroy( &sensor );
}

TEST_F(pdSensor6FTTest, FromZTK)
{
  char filename[] = "model/sensor_6ft.ztk";
  ZTK ztk;
  zVec3D v;
  zMat3D m;

  zVec3DCreate( &v, 1.0, 2.0, 3.0 );
  zMat3DCreate( &m,
                1.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 3.0 );
  ZTKParse( &ztk, filename );
  pdSensorFromZTK( &sensor, &filterarray, &ztk );
  EXPECT_STREQ( "lf_FT01", zNamePtr( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorLinkPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( &m, pdSensorLinkAtt(&sensor) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,0) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,1) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,2) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,3) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,4) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,5) ) );
  EXPECT_STREQ( "left_foot", pdSensorLinkName( &sensor ) );
  EXPECT_STREQ( "6ft", sensor.com->typestr );
  pdSensorDestroy( &sensor );
  ZTKDestroy( &ztk );
}

TEST_F(pdSensor6FTTest, FromZTK2)
{
  char filename[] = "model/sensor_6ft2.ztk";
  ZTK ztk;

  ZTKParse( &ztk, filename );
  pdSensorFromZTK( &sensor, &filterarray, &ztk );
  EXPECT_STREQ( "lf_FT02", zNamePtr( &sensor ) );
  EXPECT_TRUE( zVec3DMatch( ZVEC3DZERO, pdSensorLinkPos(&sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorLinkAtt(&sensor) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,0) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,1) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,2) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,3) ) );
  EXPECT_STREQ( "none01", zNamePtr( pdSensorFilterElem(&sensor,4) ) );
  EXPECT_STREQ( "bw01",   zNamePtr( pdSensorFilterElem(&sensor,5) ) );
  EXPECT_STREQ( "left_foot", pdSensorLinkName( &sensor ) );
  EXPECT_STREQ( "6ft", sensor.com->typestr );
  pdSensorDestroy( &sensor );
  ZTKDestroy( &ztk );
}

TEST_F(pdSensor6FTTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  zFrame3D frame;
  pdFilterArray arr;
  char linkname[] = "left_foot";
  char name[] = "lf_FT01";
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  zFrame3DIdent( &frame );
  pdFilterArrayAlloc( &arr, 6 );
  pdSensor6FTCreate( &sensor, linkname, &frame, &arr );
  zNameSet( &sensor, name );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(pdSensorFilters(&sensor),0) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(pdSensorFilters(&sensor),1) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(pdSensorFilters(&sensor),2) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(pdSensorFilters(&sensor),3) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(pdSensorFilters(&sensor),4) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(pdSensorFilters(&sensor),5) );
  sprintf( expected,
           "name: %s\ntype: 6ft\nlink: %s\nframe: {\n 1, 0, 0, 0\n 0, 1, 0, 0\n 0, 0, 1, 0\n}\n"
           "filterfx: bw01\nfilterfy: none01\nfilterfz: bw01\n"
           "filtertx: none01\nfilterty: bw01\nfiltertz: none01\n",
           name, linkname );

  pdSensorFPrintZTK( fp, &sensor );
  fflush( fp );
  EXPECT_STREQ( expected, buf );

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
  pdSensor6FTCreate( &sensor, "foot", &frame, &arr );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),0) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),1) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),2) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),3) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),4) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),5) );
  // method to testify
  pdSensorSetRawData( &sensor, v );
  pdSensorProcess( &sensor, TIME_STEP );
  pdSensor6FTGetF( &sensor, &f );
  pdSensor6FTGetT( &sensor, &tau );
  // check
  EXPECT_EQ( 1.0, f.c.x );
  EXPECT_EQ( 2.0, f.c.y );
  EXPECT_EQ( 3.0, f.c.z );
  EXPECT_EQ( 4.0, tau.c.x );
  EXPECT_EQ( 5.0, tau.c.y );
  EXPECT_EQ( 6.0, tau.c.z );
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
  pdSensor6FTCreate( &sensor, "foot", &sframe, &arr );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),0) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),1) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),2) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),3) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),4) );
  pdFilterNoneCreate( zArrayElem(pdSensorFilters(&sensor),5) );
  // method to testify
  pdSensorSetRawData( &sensor, v );
  pdSensorProcess( &sensor, TIME_STEP );
  pdSensorFrameUpdate( &sensor, &lframe );
  pdSensor6FTGetWldF( &sensor, &f );
  pdSensor6FTGetWldT( &sensor, &tau );
  // check
  EXPECT_EQ( 1.0,  f.c.x );
  EXPECT_EQ( 0.0,  f.c.y );
  EXPECT_EQ( 1.0,  f.c.z );
  EXPECT_EQ( 0.0,  tau.c.x );
  EXPECT_EQ( -1.0, tau.c.y );
  EXPECT_EQ( 0.0,  tau.c.z );
  pdSensorDestroy( &sensor );
}


class pdSensorArrayTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterArrayAlloc( &filterarray, 2 );
    pdFilterNoneCreate( zArrayElem(&filterarray,0) );
    pdFilterBWCreate(   zArrayElem(&filterarray,1), 0.5, 2 );
    zNameSet( zArrayElem(&filterarray,0), "none01" );
    zNameSet( zArrayElem(&filterarray,1), "bw01" );
  };
  virtual void TearDown() {
    pdFilterArrayDestroy( &filterarray );
  };

  pdSensorArray arr;
  pdFilterArray filterarray;
  RandomInitializer ri;
};

TEST_F(pdSensorArrayTest, NameFind)
{
  pdFilterArray lf_filters, rf_filters;

  pdFilterArrayAlloc( &lf_filters, 6 );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&lf_filters,0) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&lf_filters,1) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&lf_filters,2) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&lf_filters,3) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&lf_filters,4) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&lf_filters,5) );

  pdSensorArrayAlloc( &arr, 2 );
  pdSensor6FTCreate( zArrayElem(&arr,0), "left_foot", ZFRAME3DIDENT, &lf_filters );
  zNameSet( zArrayElem(&arr,0), "lf_FT01" );
  EXPECT_EQ( zArrayElem(&arr,0), pdSensorArrayNameFind(&arr,"lf_FT01") );

  pdFilterArrayAlloc( &rf_filters, 6 );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&rf_filters,0) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&rf_filters,1) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&rf_filters,2) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&rf_filters,3) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&rf_filters,4) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&rf_filters,5) );

  pdSensor6FTCreate( zArrayElem(&arr,1), "right_foot", ZFRAME3DIDENT, &rf_filters );
  zNameSet( zArrayElem(&arr,1), "rf_FT01" );
  EXPECT_EQ( zArrayElem(&arr,1), pdSensorArrayNameFind(&arr,"rf_FT01") );

  pdSensorArrayDestroy( &arr );
}

TEST_F(pdSensorArrayTest, FromZTK)
{
  char filename[] = "model/sensor.ztk";
  pdFilterArray filterarray;
  ZTK ztk;
  pdSensor *sensor;
  zVec3D v;

  ZTKParse( &ztk, filename );
  pdFilterArrayFromZTK( &filterarray, &ztk );
  pdSensorArrayFromZTK( &arr, &filterarray, &ztk );

  // sensor 1
  sensor = zArrayElem( &arr, 0 );
  EXPECT_STREQ( "lf_FT01", zNamePtr( sensor ) );
  zVec3DCreate( &v, 1, 0, 0 );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorLinkPos(sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorLinkAtt(sensor) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,0) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,1) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,2) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,3) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,4) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,5) ) );
  EXPECT_STREQ( "left_foot", pdSensorLinkName( sensor ) );
  EXPECT_STREQ( "6ft", sensor->com->typestr );

  // sensor 2
  sensor = zArrayElem( &arr, 1 );
  EXPECT_STREQ( "rf_FT01", zNamePtr( zArrayElem(&arr,1) ) );
  zVec3DCreate( &v, -1, 0, 0 );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorLinkPos(sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorLinkAtt(sensor) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,0) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,1) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,2) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,3) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,4) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,5) ) );
  EXPECT_STREQ( "right_foot", pdSensorLinkName( sensor ) );
  EXPECT_STREQ( "6ft", sensor->com->typestr );

  pdSensorArrayDestroy( &arr );
  pdFilterArrayDestroy( &filterarray );
  ZTKDestroy( &ztk );
}

TEST_F(pdSensorArrayTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;
  pdSensor *sensor;
  pdFilterArray lf_filters ,rf_filters;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  zArrayAlloc( &arr, pdSensor, 2 );

  // sensor 1
  pdFilterArrayAlloc( &lf_filters, 6 );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&lf_filters,0) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&lf_filters,1) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&lf_filters,2) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&lf_filters,3) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&lf_filters,4) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&lf_filters,5) );

  sensor = zArrayElem( &arr, 0 );
  pdSensor6FTCreate( sensor, "left_foot", ZFRAME3DIDENT, &lf_filters );
  zNameSet( sensor, "lf_FT01" );

  // sensor 2
  pdFilterArrayAlloc( &rf_filters, 6 );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&rf_filters,0) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&rf_filters,1) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&rf_filters,2) );
  pdFilterClone( zArrayElem(&filterarray,0), zArrayElem(&rf_filters,3) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&rf_filters,4) );
  pdFilterClone( zArrayElem(&filterarray,1), zArrayElem(&rf_filters,5) );

  sensor = zArrayElem( &arr, 1 );
  pdSensor6FTCreate( zArrayElem(&arr,1), "right_foot", ZFRAME3DIDENT, &rf_filters  );
  zNameSet( zArrayElem(&arr,1), "rf_FT01" );

  sprintf( expected,
           "[pedi2::sensor]\n"
           "name: lf_FT01\ntype: 6ft\nlink: left_foot\nframe: {\n 1, 0, 0, 0\n 0, 1, 0, 0\n 0, 0, 1, 0\n}\n"
           "filterfx: none01\nfilterfy: none01\nfilterfz: none01\n"
           "filtertx: bw01\nfilterty: bw01\nfiltertz: bw01\n\n"
           "[pedi2::sensor]\n"
           "name: rf_FT01\ntype: 6ft\nlink: right_foot\nframe: {\n 1, 0, 0, 0\n 0, 1, 0, 0\n 0, 0, 1, 0\n}\n"
           "filterfx: bw01\nfilterfy: none01\nfilterfz: none01\n"
           "filtertx: none01\nfilterty: bw01\nfiltertz: bw01\n\n"
           );

  pdSensorArrayFPrintZTK( fp, &arr );
  fflush( fp );
  EXPECT_STREQ( expected, buf );

  pdSensorArrayDestroy( &arr );
  fclose( fp );
}

TEST_F(pdSensorArrayTest, ReadZTK)
{
  char filename[] = "model/sensor.ztk";
  pdFilterArray filterarray;
  pdSensor *sensor;
  zVec3D v;

  pdFilterArrayReadZTK( &filterarray, filename );
  pdSensorArrayReadZTK( &arr, &filterarray, filename );
  ASSERT_TRUE( zArrayBuf(&arr) );

  // sensor 1
  sensor = zArrayElem( &arr, 0 );
  EXPECT_STREQ( "lf_FT01", zNamePtr( sensor ) );
  zVec3DCreate( &v, 1, 0, 0 );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorLinkPos(sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorLinkAtt(sensor) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,0) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,1) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,2) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,3) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,4) ) );
  EXPECT_STREQ( "bw01", zNamePtr( pdSensorFilterElem(sensor,5) ) );
  EXPECT_STREQ( "left_foot", pdSensorLinkName( sensor ) );
  EXPECT_STREQ( "6ft", sensor->com->typestr );

  // sensor 2
  sensor = zArrayElem( &arr, 1 );
  EXPECT_STREQ( "rf_FT01", zNamePtr( zArrayElem(&arr,1) ) );
  zVec3DCreate( &v, -1, 0, 0 );
  EXPECT_TRUE( zVec3DMatch( &v, pdSensorLinkPos(sensor) ) );
  EXPECT_TRUE( zMat3DMatch( ZMAT3DIDENT, pdSensorLinkAtt(sensor) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,0) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,1) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,2) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,3) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,4) ) );
  EXPECT_STREQ( "bw02", zNamePtr( pdSensorFilterElem(sensor,5) ) );
  EXPECT_STREQ( "right_foot", pdSensorLinkName( sensor ) );
  EXPECT_STREQ( "6ft", sensor->com->typestr );

  pdSensorArrayDestroy( &arr );
  pdFilterArrayDestroy( &filterarray );
}

TEST_F(pdSensorArrayTest, WriteZTK)
{
  // Same as FPrintZTK
}
