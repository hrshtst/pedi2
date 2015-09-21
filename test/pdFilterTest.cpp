#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_filter.h>

const double TIME_STEP = 0.01;

class pdFilterTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterTest, Init)
{
  pdFilterInit( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}

TEST_F(pdFilterTest, DestroyDefault)
{
  pdFilterInit( &filter );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroyDefault( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}


class pdFilterNoneTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterNoneTest, Create)
{
  EXPECT_TRUE( pdFilterCreateNone( &filter ) );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( &pd_filter_none_met, filter._met );
  pdFilterDestroy( &filter );
}

TEST_F(pdFilterNoneTest, Destroy)
{
  pdFilterCreateNone( &filter );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroy( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}

TEST_F(pdFilterNoneTest, Update)
{
  double input, output;

  pdFilterCreateNone( &filter );
  input = 0.1;
  pdFilterInput( &filter ) = input;
  output = pdFilterUpdate( &filter, TIME_STEP );
  EXPECT_EQ( input, output );
  input = ri.rand();
  pdFilterInput( &filter ) = input;
  output = pdFilterUpdate( &filter, TIME_STEP );
  EXPECT_EQ( input, output );
  pdFilterDestroy( &filter );
}

TEST_F(pdFilterNoneTest, FRead)
{
  char filename[] = "model/filter_none.conf";
  FILE *fp;

  fp = fopen( filename, "r" );
  pdFilterFRead( fp, &filter );
  EXPECT_STREQ( "none01", zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( &pd_filter_none_met, filter._met );
  pdFilterDestroy( &filter );
  fclose( fp );
}


const double BWF_CF = 50;
const int BWF_DIM = 2;
class pdFilterBWTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterBWTest, Create)
{
  EXPECT_TRUE( pdFilterCreateBW( &filter, BWF_CF, BWF_DIM ) );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_TRUE( filter._prm );
  EXPECT_EQ( &pd_filter_bw_met, filter._met );
  pdFilterDestroy( &filter );
}

TEST_F(pdFilterBWTest, Destroy)
{
  pdFilterCreateBW( &filter, BWF_CF, BWF_DIM );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroy( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}

TEST_F(pdFilterBWTest, FRead)
{
  char filename[] = "model/filter_bw.conf";
  FILE *fp;

  fp = fopen( filename, "r" );
  pdFilterFRead( fp, &filter );
  EXPECT_STREQ( "bw01", zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_TRUE( filter._prm );
  EXPECT_EQ( &pd_filter_bw_met, filter._met );
  pdFilterDestroy( &filter );
  fclose( fp );
}


class pdFilterArrayTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilterArray arr;
  RandomInitializer ri;
};

TEST_F(pdFilterArrayTest, NameFind)
{
  zArrayAlloc( &arr, pdFilter, 2 );
  pdFilterCreateNone( zArrayElem(&arr,0) );
  pdFilterCreateBW( zArrayElem(&arr,1), BWF_CF, BWF_DIM );
  zNameSet( zArrayElem(&arr,0), (char*)"none01" );
  zNameSet( zArrayElem(&arr,1), (char*)"bw01" );
  EXPECT_EQ( zArrayElem(&arr,0), pdFilterArrayNameFind( &arr, "none01" ) );
  EXPECT_EQ( zArrayElem(&arr,1), pdFilterArrayNameFind( &arr, "bw01" ) );
  pdFilterArrayDestroy( &arr );
}

TEST_F(pdFilterArrayTest, FRead)
{
  char filename[] = "model/sensor.conf";
  FILE *fp;

  fp = fopen( filename, "r" );
  pdFilterArrayFRead( fp, &arr );
  EXPECT_STREQ( "bw01", zNamePtr( zArrayElem(&arr,0) ) );
  EXPECT_STREQ( "bw02", zNamePtr( zArrayElem(&arr,1) ) );
  EXPECT_STREQ( "none01", zNamePtr( zArrayElem(&arr,2) ) );
  pdFilterArrayDestroy( &arr );
  fclose( fp );
}
