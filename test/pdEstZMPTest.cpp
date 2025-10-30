#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_est_zmp.h>

// These definitions should be removed by using CPP version library
const zVec3D zVec3D::zvec3Dzero = { { 0, 0, 0 } };
const zMat3D zMat3D::zmat3Dident = { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } };
const zFrame3D zFrame3D::zframe3Dident = {
  { { 0, 0, 0 } },
  { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } }
};

TEST(pdSensorListTest, CellInit)
{
  pdSensorListCell cell;

  pdSensorListCellInit( &cell );

  EXPECT_EQ( &cell, zListCellNext(&cell));
  EXPECT_EQ( &cell, zListCellPrev(&cell));
  EXPECT_EQ( NULL, pdSensorListCellSensor(&cell) );
}

TEST(pdSensorListTest, Push)
{
  pdSensorList list;
  pdSensorListCell *cp;
  pdSensor s1;

  pdSensorDummyCreate( &s1, "foot", ZFRAME3DIDENT );
  zListInit( &list );
  EXPECT_EQ( 0, zListSize( &list ) );

  pdSensorListPush( &list, &s1 );
  EXPECT_EQ( 1, zListSize( &list ) );
  cp = zListCellNext( zListRoot(&list) );
  EXPECT_EQ( &s1, cp->data );
  cp = zListCellPrev( zListRoot(&list) );
  EXPECT_EQ( &s1, cp->data );

  pdSensorListDestroy( &list );
  EXPECT_EQ( 0, zListSize( &list ) );
}

TEST(pdSensorListTest, Push2)
{
  pdSensorList list;
  pdSensorListCell *cp;
  pdSensor s1, s2;

  pdSensorDummyCreate( &s1, "left_foot", ZFRAME3DIDENT );
  pdSensorDummyCreate( &s2, "right_foot", ZFRAME3DIDENT );
  zListInit( &list );
  EXPECT_EQ( 0, zListSize( &list ) );

  pdSensorListPush( &list, &s1 );
  pdSensorListPush( &list, &s2 );
  EXPECT_EQ( 2, zListSize( &list ) );
  cp = zListCellNext( zListRoot(&list) );
  EXPECT_EQ( &s2, cp->data );
  cp = zListCellNext( cp );
  EXPECT_EQ( &s1, cp->data );

  pdSensorListDestroy( &list );
  EXPECT_EQ( 0, zListSize( &list ) );
}

TEST(pdSensorListTest, Pop)
{
  pdSensorList list;
  pdSensor s1, s2;
  pdSensor *sp;

  pdSensorDummyCreate( &s1, "left_foot", ZFRAME3DIDENT );
  pdSensorDummyCreate( &s2, "right_foot", ZFRAME3DIDENT );
  zListInit( &list );
  EXPECT_EQ( 0, zListSize( &list ) );

  pdSensorListPush( &list, &s1 );
  pdSensorListPush( &list, &s2 );
  EXPECT_EQ( 2, zListSize( &list ) );
  sp = pdSensorListPop( &list );
  EXPECT_EQ( &s2, sp );
  sp = pdSensorListPop( &list );
  EXPECT_EQ( &s1, sp );
  sp = pdSensorListPop( &list );
  EXPECT_EQ( NULL, sp );

  pdSensorListDestroy( &list );
  EXPECT_EQ( 0, zListSize( &list ) );
}

class pdEstZmpTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  void SetRandomValues() {
    e._lfsensor.size = ri.rand();
    e._rfsensor.size = ri.rand();
    ri.SetRandVec3D( &e.estforce );
    ri.SetRandVec3D( &e.estzmp );
  }

  RandomInitializer ri;
  pdEstZmp e;
};

TEST_F(pdEstZmpTest, Init)
{
  SetRandomValues();
  pdEstZmpInit( &e );
  EXPECT_EQ( 0, zListSize( &e._lfsensor ) );
  EXPECT_EQ( 0, zListSize( &e._rfsensor ) );
  EXPECT_EQ( 0, pdEstZmpEstForceX( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstForceY( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstForceZ( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstZMPX( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstZMPY( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstZMPZ( &e ) );
}

TEST_F(pdEstZmpTest, Destroy)
{
  pdEstZmpInit( &e );
  SetRandomValues();
  pdEstZmpDestroy( &e );
  EXPECT_EQ( 0, zListSize( &e._lfsensor ) );
  EXPECT_EQ( 0, zListSize( &e._rfsensor ) );
  EXPECT_EQ( 0, pdEstZmpEstForceX( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstForceY( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstForceZ( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstZMPX( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstZMPY( &e ) );
  EXPECT_EQ( 0, pdEstZmpEstZMPZ( &e ) );
}

TEST_F(pdEstZmpTest, FromZTK)
{
  char filename[] = "model/zmpest.ztk";
  ZTK ztk;
  pdSensorListCell *cp;

  ZTKParse( &ztk, filename );
  ASSERT_TRUE( pdEstZmpFromZTK( &e, &ztk ) );
  // check filter list
  EXPECT_EQ( 2, zArraySize( pdEstZmpFilterArray(&e) ) );
  EXPECT_STREQ( "bw01", zNamePtr(zArrayElem(pdEstZmpFilterArray(&e),0)));
  EXPECT_STREQ( "bw02", zNamePtr(zArrayElem(pdEstZmpFilterArray(&e),1)));
  // check sensor array
  EXPECT_EQ( 4, zArraySize( pdEstZmpSensorArray(&e) ) );
  EXPECT_STREQ( "lf_FT01", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),0)));
  EXPECT_STREQ( "lf_FT02", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),1)));
  EXPECT_STREQ( "rf_FT01", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),2)));
  EXPECT_STREQ( "rf_FT02", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),3)));
  // check pdEstZmp class
  EXPECT_EQ( 2, zListSize( &e._lfsensor ) );
  cp = zListRoot(&e._lfsensor);
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),0), pdSensorListCellSensor(cp) );
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),1), pdSensorListCellSensor(cp) );
  EXPECT_EQ( 2, zListSize( &e._rfsensor ) );
  cp = zListRoot(&e._rfsensor);
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),2), pdSensorListCellSensor(cp) );
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),3), pdSensorListCellSensor(cp) );
  pdEstZmpDestroy( &e );
  ZTKDestroy( &ztk );
}

TEST_F(pdEstZmpTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;
  FILE *fp_expected;
  char filename[] = "model/zmpest.ztk";
  ZTK ztk;

  ZTKParse( &ztk, filename );
  ASSERT_TRUE( pdEstZmpFromZTK( &e, &ztk ) );

  fp_expected = fmemopen( expected, sizeof(expected), "r+" );
  pdFilterArrayFPrintZTK( fp_expected, pdEstZmpFilterArray(&e) );
  pdSensorArrayFPrintZTK( fp_expected, pdEstZmpSensorArray(&e) );
  fprintf( fp_expected,
           "[pedi2::estimator]\n"
           "type: zmp\n"
           "leftfoot: lf_FT01 lf_FT02\n"
           "rightfoot: rf_FT01 rf_FT02\n");
  fflush( fp_expected );

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdEstZmpFPrintZTK( fp, &e );
  fflush( fp );
  EXPECT_STREQ( expected, buf );

  pdEstZmpDestroy( &e );
  ZTKDestroy( &ztk );
}

TEST_F(pdEstZmpTest, ReadZTK)
{
  char filename[] = "model/zmpest.ztk";
  pdSensorListCell *cp;

  ASSERT_TRUE( pdEstZmpReadZTK( &e, filename ) );
  // check filter list
  EXPECT_EQ( 2, zArraySize( pdEstZmpFilterArray(&e) ) );
  EXPECT_STREQ( "bw01", zNamePtr(zArrayElem(pdEstZmpFilterArray(&e),0)));
  EXPECT_STREQ( "bw02", zNamePtr(zArrayElem(pdEstZmpFilterArray(&e),1)));
  // check sensor array
  EXPECT_EQ( 4, zArraySize( pdEstZmpSensorArray(&e) ) );
  EXPECT_STREQ( "lf_FT01", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),0)));
  EXPECT_STREQ( "lf_FT02", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),1)));
  EXPECT_STREQ( "rf_FT01", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),2)));
  EXPECT_STREQ( "rf_FT02", zNamePtr(zArrayElem(pdEstZmpSensorArray(&e),3)));
  // check pdEstZmp class
  EXPECT_EQ( 2, zListSize( &e._lfsensor ) );
  cp = zListRoot(&e._lfsensor);
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),0), pdSensorListCellSensor(cp) );
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),1), pdSensorListCellSensor(cp) );
  EXPECT_EQ( 2, zListSize( &e._rfsensor ) );
  cp = zListRoot(&e._rfsensor);
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),2), pdSensorListCellSensor(cp) );
  cp = zListCellPrev( cp );
  EXPECT_EQ( zArrayElem(pdEstZmpSensorArray(&e),3), pdSensorListCellSensor(cp) );
  pdEstZmpDestroy( &e );
}
