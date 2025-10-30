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
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( NULL, filter.com );
}

TEST_F(pdFilterTest, DefaultDestroy)
{
  pdFilterInit( &filter );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );

  pdFilterDefaultDestroy( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( NULL, filter.com );
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
  pdFilterNoneCreate( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( &pd_filter_none_com, filter.com );
  pdFilterDestroy( &filter );
}

TEST_F(pdFilterNoneTest, Destroy)
{
  pdFilterNoneCreate( &filter );
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
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( NULL, filter.com );
}

TEST_F(pdFilterNoneTest, Update)
{
  double input, output;

  pdFilterNoneCreate( &filter );
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

TEST_F(pdFilterNoneTest, Clone)
{
  pdFilter org, cln;

  pdFilterNoneCreate( &org );
  zNameSet( &org, (char*)"none01" );
  pdFilterInput( &org ) = ri.rand();
  pdFilterOutput( &org ) = ri.rand();
  pdFilterClone( &org, &cln );

  EXPECT_STREQ( "none01", zNamePtr(&cln) );
  EXPECT_EQ( pdFilterInput( &org ), pdFilterInput( &cln ) );
  EXPECT_EQ( pdFilterOutput( &org ), pdFilterOutput( &cln ) );
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( &pd_filter_none_com, cln.com );
  EXPECT_STREQ( "none", cln.com->typestr );
  pdFilterDestroy( &org );
  pdFilterDestroy( &cln );
}

TEST_F(pdFilterNoneTest, FromZTK)
{
  char filename[] = "model/filter_none.ztk";
  ZTK ztk;

  ZTKParse( &ztk, filename );
  pdFilterFromZTK( &filter, &ztk );
  EXPECT_STREQ( "none01", zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( &pd_filter_none_com, filter.com );
  EXPECT_STREQ( "none", filter.com->typestr );
  pdFilterDestroy( &filter );
  ZTKDestroy( &ztk );
}

TEST_F(pdFilterNoneTest, FPrintZTK)
{
  char buf[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdFilterNoneCreate( &filter );
  zNameSet( &filter, (char*)"none01" );

  pdFilterFPrintZTK( fp, &filter );
  fflush( fp );
  EXPECT_STREQ( "name: none01\ntype: none\n", buf );
  pdFilterDestroy( &filter );
  fclose( fp );
}


typedef struct{
  dzSys bwf;
  double cf;
  uint dim;
} _pdBW;

const double BWF_CF = 50;
const uint BWF_DIM = 2;
class pdFilterBWTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterBWTest, Create)
{
  pdFilterBWCreate( &filter, BWF_CF, BWF_DIM );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( BWF_CF, ((_pdBW*)filter.prp)->cf );
  EXPECT_EQ( BWF_DIM, ((_pdBW*)filter.prp)->dim );
  EXPECT_TRUE( filter.prp );
  EXPECT_EQ( &pd_filter_bw_com, filter.com );
  pdFilterDestroy( &filter );
}

TEST_F(pdFilterBWTest, Destroy)
{
  pdFilterBWCreate( &filter, BWF_CF, BWF_DIM );
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
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( NULL, filter.com );
}

TEST_F(pdFilterBWTest, Refresh)
{
  double output;

  output = ri.rand();
  pdFilterBWCreate( &filter, BWF_CF, BWF_DIM );
  pdFilterOutput( &filter ) = output;
  pdFilterRefresh( &filter );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
}

TEST_F(pdFilterBWTest, Update)
{
  pdFilterBWCreate( &filter, BWF_CF, BWF_DIM );
  pdFilterInput( &filter ) = 0;
  pdFilterUpdate( &filter, 0.01 );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  pdFilterInput( &filter ) = 1;
  pdFilterUpdate( &filter, 0.01 );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroy( &filter );
}

TEST_F(pdFilterBWTest, Clone)
{
  pdFilter org, cln;

  pdFilterBWCreate( &org, 0.8, 3 );
  zNameSet( &org, (char*)"bw01" );
  pdFilterInput( &org ) = ri.rand();
  pdFilterOutput( &org ) = ri.rand();
  pdFilterClone( &org, &cln );
  EXPECT_STREQ( "bw01", zNamePtr(&cln) );
  EXPECT_EQ( pdFilterInput( &org ), pdFilterInput( &cln ) );
  EXPECT_EQ( pdFilterOutput( &org ), pdFilterOutput( &cln ) );
  EXPECT_EQ( 0.8, ((_pdBW*)cln.prp)->cf );
  EXPECT_EQ( 3, ((_pdBW*)cln.prp)->dim );
  EXPECT_TRUE( cln.prp );
  EXPECT_EQ( &pd_filter_bw_com, cln.com );
  EXPECT_STREQ( "bw", cln.com->typestr );
  pdFilterDestroy( &org );
  pdFilterDestroy( &cln );
}

TEST_F(pdFilterBWTest, FromZTK)
{
  char filename[] = "model/filter_bw.ztk";
  ZTK ztk;

  ZTKParse( &ztk, filename );
  pdFilterFromZTK( &filter, &ztk );
  EXPECT_STREQ( "bw01", zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( 0.5, ((_pdBW*)filter.prp)->cf );
  EXPECT_EQ( 2, ((_pdBW*)filter.prp)->dim );
  EXPECT_TRUE( filter.prp );
  EXPECT_EQ( &pd_filter_bw_com, filter.com );
  EXPECT_STREQ( "bw", filter.com->typestr );
  pdFilterDestroy( &filter );
  ZTKDestroy( &ztk );
}

TEST_F(pdFilterBWTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdFilterBWCreate( &filter, BWF_CF, BWF_DIM );
  zNameSet( &filter, (char*)"bw01" );
  sprintf( expected, "name: %s\ntype: bw\ncf: %.10g\ndim: %d\n",
           zName(&filter), BWF_CF, BWF_DIM );

  pdFilterFPrintZTK( fp, &filter );
  fflush( fp );
  EXPECT_STREQ( expected, buf );
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

TEST_F(pdFilterArrayTest, Destroy)
{
  zArrayAlloc( &arr, pdFilter, 2 );
  pdFilterNoneCreate( zArrayElem(&arr,0) );
  pdFilterBWCreate( zArrayElem(&arr,1), BWF_CF, BWF_DIM );
  pdFilterArrayDestroy( &arr );
  fprintf( stderr, "arr: %p\n", &arr );
  fprintf( stderr, "arr.buf: %p\n", arr.buf );
  EXPECT_EQ( NULL, zArrayBuf( &arr ) );
}

TEST_F(pdFilterArrayTest, NameFind)
{
  zArrayAlloc( &arr, pdFilter, 2 );
  pdFilterNoneCreate( zArrayElem(&arr,0) );
  pdFilterBWCreate( zArrayElem(&arr,1), BWF_CF, BWF_DIM );
  zNameSet( zArrayElem(&arr,0), (char*)"none01" );
  zNameSet( zArrayElem(&arr,1), (char*)"bw01" );
  EXPECT_EQ( zArrayElem(&arr,0), pdFilterArrayNameFind( &arr, "none01" ) );
  EXPECT_EQ( zArrayElem(&arr,1), pdFilterArrayNameFind( &arr, "bw01" ) );
  pdFilterArrayDestroy( &arr );
}

TEST_F(pdFilterArrayTest, FromZTK)
{
  char filename[] = "model/sensor.ztk";
  ZTK ztk;
  pdFilter *filter;

  ZTKParse( &ztk, filename );
  pdFilterArrayFromZTK( &arr, &ztk );

  // filter 1
  filter = zArrayElem( &arr, 0 );
  EXPECT_STREQ( "bw01", zNamePtr( filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( filter ) );
  EXPECT_EQ( 2, ((_pdBW*)filter->prp)->dim );
  EXPECT_EQ( 50, ((_pdBW*)filter->prp)->cf );
  EXPECT_TRUE( filter->prp );
  EXPECT_EQ( &pd_filter_bw_com, filter->com );
  EXPECT_STREQ( "bw", filter->com->typestr );
  // filter 2
  filter = zArrayElem( &arr, 1 );
  EXPECT_STREQ( "bw02", zNamePtr( filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( filter ) );
  EXPECT_EQ( 3, ((_pdBW*)filter->prp)->dim );
  EXPECT_EQ( 100, ((_pdBW*)filter->prp)->cf );
  EXPECT_TRUE( filter->prp );
  EXPECT_EQ( &pd_filter_bw_com, filter->com );
  EXPECT_STREQ( "bw", filter->com->typestr );
  // filter 3
  filter = zArrayElem( &arr, 2 );
  EXPECT_STREQ( "none01", zNamePtr( filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( filter ) );
  EXPECT_EQ( NULL, filter->prp );
  EXPECT_EQ( &pd_filter_none_com, filter->com );
  EXPECT_STREQ( "none", filter->com->typestr );

  pdFilterArrayDestroy( &arr );
  ZTKDestroy( &ztk );
}

TEST_F(pdFilterArrayTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  zArrayAlloc( &arr, pdFilter, 2 );
  pdFilterNoneCreate( zArrayElem(&arr,0) );
  pdFilterBWCreate( zArrayElem(&arr,1), BWF_CF, BWF_DIM );
  zNameSet( zArrayElem(&arr,0), (char*)"none01" );
  zNameSet( zArrayElem(&arr,1), (char*)"bw01" );
  sprintf( expected,
           "[pedi2::filter]\nname: %s\ntype: none\n\n"\
           "[pedi2::filter]\nname: %s\ntype: bw\ncf: %.10g\ndim: %d\n\n",
           zName(zArrayElem(&arr,0)),
           zName(zArrayElem(&arr,1)), BWF_CF, BWF_DIM );

  pdFilterArrayFPrintZTK( fp, &arr );
  fflush( fp );
  EXPECT_STREQ( expected, buf );

  pdFilterArrayDestroy( &arr );
  fclose( fp );
}

TEST_F(pdFilterArrayTest, ReadZTK)
{
  char filename[] = "model/sensor.ztk";
  pdFilter *filter;

  pdFilterArrayReadZTK( &arr, filename );
  ASSERT_TRUE( zArrayBuf(&arr) );

  // filter 1
  filter = zArrayElem( &arr, 0 );
  EXPECT_STREQ( "bw01", zNamePtr( filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( filter ) );
  EXPECT_EQ( 2, ((_pdBW*)filter->prp)->dim );
  EXPECT_EQ( 50, ((_pdBW*)filter->prp)->cf );
  EXPECT_TRUE( filter->prp );
  EXPECT_EQ( &pd_filter_bw_com, filter->com );
  EXPECT_STREQ( "bw", filter->com->typestr );
  // filter 2
  filter = zArrayElem( &arr, 1 );
  EXPECT_STREQ( "bw02", zNamePtr( filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( filter ) );
  EXPECT_EQ( 3, ((_pdBW*)filter->prp)->dim );
  EXPECT_EQ( 100, ((_pdBW*)filter->prp)->cf );
  EXPECT_TRUE( filter->prp );
  EXPECT_EQ( &pd_filter_bw_com, filter->com );
  EXPECT_STREQ( "bw", filter->com->typestr );
  // filter 3
  filter = zArrayElem( &arr, 2 );
  EXPECT_STREQ( "none01", zNamePtr( filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( filter ) );
  EXPECT_EQ( NULL, filter->prp );
  EXPECT_EQ( &pd_filter_none_com, filter->com );
  EXPECT_STREQ( "none", filter->com->typestr );

  pdFilterArrayDestroy( &arr );
}

TEST_F(pdFilterArrayTest, WriteZTK)
{
  // Same as FPrintZTK
}
