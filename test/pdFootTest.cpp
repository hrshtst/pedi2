#include "gtest/gtest.h"
#include <pedi2/pd_foot.h>

const double TIME_STEP = 0.01;

class pdFootTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzInit( &cz, &vrt );
    pdFootInit( &lf, &cz, PD_FOOT_LEFT, TIME_STEP );
    pdFootInit( &rf, &cz, PD_FOOT_RIGHT, TIME_STEP );
  };
  virtual void TearDown() {
    pdFootDestroy( &lf );
    pdFootDestroy( &rf );
    pdCZHrzDestroy( &cz );
  };

  void SetVacuousPrm(pdFoot *f) {
    pdFootSetPosX( f, 1 );
    pdFootSetPosY( f, 2 );
    pdFootSetPosZ( f, 3 );
    pdFootSetDesPosX( f, 4 );
    pdFootSetDesPosY( f, 5 );
    pdFootSetDesPosZ( f, 6 );
    pdFootSetAttX( f, 7 );
    pdFootSetAttY( f, 8 );
    pdFootSetAttZ( f, 9 );
    pdFootSetDesAttX( f, 10 );
    pdFootSetDesAttY( f, 11 );
    pdFootSetDesAttZ( f, 12 );
    pdFootSetSR( f, &dummy );
    pdFootSetTrXK( f, 13 );
    pdFootSetTrXC( f, 14 );
    pdFootSetTrXOld( f, 15 );
    pdFootSetTrYK( f, 16 );
    pdFootSetTrYC( f, 17 );
    pdFootSetTrYOld( f, 18 );
    pdFootSetTrZK( f, 19 );
    pdFootSetTrZC( f, 20 );
    pdFootSetTrZOld( f, 21 );
    pdFootSetMaxHeight( f, 22 );
    pdFootRefPosX( f ) = 23;
    pdFootRefPosY( f ) = 24;
    pdFootRefPosZ( f ) = 25;
    pdFootRefAttX( f ) = 26;
    pdFootRefAttY( f ) = 27;
    pdFootRefAttZ( f ) = 28;
    pdFootUWSign( pdFootUWPtr( f ) ) = 100;
    pdFootZSign( pdFootZPtr( f ) ) = -100;
  };

  zVec3DList sr, dummy;
  pdCZVrt vrt;
  pdCZHrz cz;
  pdFoot lf, rf;
};

TEST_F(pdFootTest, Init)
{
  SetVacuousPrm( &lf );
  pdFootInit( &lf, &cz, PD_FOOT_LEFT, TIME_STEP );
  EXPECT_EQ( 0, pdFootPosX( &lf ) );
  EXPECT_EQ( 0, pdFootPosY( &lf ) );
  EXPECT_EQ( 0, pdFootPosZ( &lf ) );
  EXPECT_EQ( 0, pdFootDesPosX( &lf ) );
  EXPECT_EQ( 0, pdFootDesPosY( &lf ) );
  EXPECT_EQ( 0, pdFootDesPosZ( &lf ) );
  EXPECT_EQ( 0, pdFootAttX( &lf ) );
  EXPECT_EQ( 0, pdFootAttY( &lf ) );
  EXPECT_EQ( 0, pdFootAttZ( &lf ) );
  EXPECT_EQ( 0, pdFootDesAttX( &lf ) );
  EXPECT_EQ( 0, pdFootDesAttY( &lf ) );
  EXPECT_EQ( 0, pdFootDesAttZ( &lf ) );
  EXPECT_EQ( NULL, pdFootSR( &lf ) );
  EXPECT_EQ( &cz, pdFootCZPtr( &lf ) );
  EXPECT_EQ( 0, lf._sol._k[0] );
  EXPECT_EQ( 0, lf._sol._c[0] );
  EXPECT_EQ( 0, lf._sol._old[0] );
  EXPECT_EQ( 0, lf._sol._k[1] );
  EXPECT_EQ( 0, lf._sol._c[1] );
  EXPECT_EQ( 0, lf._sol._old[1] );
  EXPECT_EQ( 0, lf._sol._k[2] );
  EXPECT_EQ( 0, lf._sol._c[2] );
  EXPECT_EQ( 0, lf._sol._old[2] );
  EXPECT_EQ( 0, pdFootRefPosX( &lf ) );
  EXPECT_EQ( 0, pdFootRefPosY( &lf ) );
  EXPECT_EQ( 0, pdFootRefPosZ( &lf ) );
  EXPECT_EQ( 0, pdFootRefAttX( &lf ) );
  EXPECT_EQ( 0, pdFootRefAttY( &lf ) );
  EXPECT_EQ( 0, pdFootRefAttZ( &lf ) );
  EXPECT_EQ( 1, pdFootUWSign( pdFootUWPtr( &lf ) ) );
  EXPECT_EQ( 1, pdFootZSign( pdFootZPtr( &lf ) ) );

  SetVacuousPrm( &rf );
  pdFootInit( &rf, &cz, PD_FOOT_RIGHT, TIME_STEP );
  EXPECT_EQ( -1, pdFootUWSign( pdFootUWPtr( &rf ) ) );
  EXPECT_EQ( -1, pdFootZSign( pdFootZPtr( &rf ) ) );
}

TEST_F(pdFootTest, Destroy)
{
  SetVacuousPrm( &lf );
  pdFootInit( &lf, &cz, PD_FOOT_LEFT, TIME_STEP );
  pdFootDestroy( &lf );
  EXPECT_EQ( 0, pdFootPosX( &lf ) );
  EXPECT_EQ( 0, pdFootPosY( &lf ) );
  EXPECT_EQ( 0, pdFootPosZ( &lf ) );
  EXPECT_EQ( 0, pdFootDesPosX( &lf ) );
  EXPECT_EQ( 0, pdFootDesPosY( &lf ) );
  EXPECT_EQ( 0, pdFootDesPosZ( &lf ) );
  EXPECT_EQ( 0, pdFootAttX( &lf ) );
  EXPECT_EQ( 0, pdFootAttY( &lf ) );
  EXPECT_EQ( 0, pdFootAttZ( &lf ) );
  EXPECT_EQ( 0, pdFootDesAttX( &lf ) );
  EXPECT_EQ( 0, pdFootDesAttY( &lf ) );
  EXPECT_EQ( 0, pdFootDesAttZ( &lf ) );
  EXPECT_EQ( NULL, pdFootSR( &lf ) );
  EXPECT_EQ( NULL, pdFootCZPtr( &lf ) );
  EXPECT_EQ( 0, lf._sol._k[0] );
  EXPECT_EQ( 0, lf._sol._c[0] );
  EXPECT_EQ( 0, lf._sol._old[0] );
  EXPECT_EQ( 0, lf._sol._k[1] );
  EXPECT_EQ( 0, lf._sol._c[1] );
  EXPECT_EQ( 0, lf._sol._old[1] );
  EXPECT_EQ( 0, lf._sol._k[2] );
  EXPECT_EQ( 0, lf._sol._c[2] );
  EXPECT_EQ( 0, lf._sol._old[2] );
  EXPECT_EQ( 0, pdFootRefPosX( &lf ) );
  EXPECT_EQ( 0, pdFootRefPosY( &lf ) );
  EXPECT_EQ( 0, pdFootRefPosZ( &lf ) );
  EXPECT_EQ( 0, pdFootRefAttX( &lf ) );
  EXPECT_EQ( 0, pdFootRefAttY( &lf ) );
  EXPECT_EQ( 0, pdFootRefAttZ( &lf ) );
  EXPECT_EQ( 0, pdFootUWSign( pdFootUWPtr( &lf ) ) );
  EXPECT_EQ( 0, pdFootZSign( pdFootZPtr( &lf ) ) );
}

TEST_F(pdFootTest, SetPos)
{
  SetVacuousPrm( &lf );
  pdFootSetPos( &lf, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootPosX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootPosY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootPosZ( &lf ) );
}

TEST_F(pdFootTest, SetPosVec)
{
  zVec3D v;

  SetVacuousPrm( &lf );
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdFootSetPosVec( &lf, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdFootPosX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootPosY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootPosZ( &lf ) );
}

TEST_F(pdFootTest, SetPosXYZ)
{
  SetVacuousPrm( &lf );
  pdFootSetPosX( &lf, 0.1 );
  pdFootSetPosY( &lf, 0.2 );
  pdFootSetPosZ( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootPosX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootPosY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootPosZ( &lf ) );
}

TEST_F(pdFootTest, SetDesPos)
{
  SetVacuousPrm( &lf );
  pdFootSetDesPos( &lf, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootDesPosX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootDesPosY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootDesPosZ( &lf ) );
}

TEST_F(pdFootTest, SetDesPosVec)
{
  zVec3D v;

  SetVacuousPrm( &lf );
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdFootSetDesPosVec( &lf, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdFootDesPosX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootDesPosY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootDesPosZ( &lf ) );
}

TEST_F(pdFootTest, SetDesPosXYZ)
{
  SetVacuousPrm( &lf );
  pdFootSetDesPosX( &lf, 0.1 );
  pdFootSetDesPosY( &lf, 0.2 );
  pdFootSetDesPosZ( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootDesPosX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootDesPosY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootDesPosZ( &lf ) );
}

TEST_F(pdFootTest, SetAtt)
{
  SetVacuousPrm( &lf );
  pdFootSetAtt( &lf, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootAttX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootAttY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootAttZ( &lf ) );
}

TEST_F(pdFootTest, SetAttVec)
{
  zVec3D v;

  SetVacuousPrm( &lf );
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdFootSetAttVec( &lf, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdFootAttX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootAttY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootAttZ( &lf ) );
}

TEST_F(pdFootTest, SetAttXYZ)
{
  SetVacuousPrm( &lf );
  pdFootSetAttX( &lf, 0.1 );
  pdFootSetAttY( &lf, 0.2 );
  pdFootSetAttZ( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootAttX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootAttY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootAttZ( &lf ) );
}

TEST_F(pdFootTest, SetDesAtt)
{
  SetVacuousPrm( &lf );
  pdFootSetDesAtt( &lf, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootDesAttX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootDesAttY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootDesAttZ( &lf ) );
}

TEST_F(pdFootTest, SetDesAttVec)
{
  zVec3D v;

  SetVacuousPrm( &lf );
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdFootSetDesAttVec( &lf, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdFootDesAttX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootDesAttY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootDesAttZ( &lf ) );
}

TEST_F(pdFootTest, SetDesAttXYZ)
{
  SetVacuousPrm( &lf );
  pdFootSetDesAttX( &lf, 0.1 );
  pdFootSetDesAttY( &lf, 0.2 );
  pdFootSetDesAttZ( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdFootDesAttX( &lf ) );
  EXPECT_DOUBLE_EQ( 0.2, pdFootDesAttY( &lf ) );
  EXPECT_DOUBLE_EQ( 0.3, pdFootDesAttZ( &lf ) );
}

TEST_F(pdFootTest, SetTrX)
{
  SetVacuousPrm( &lf );
  pdFootSetTrXK( &lf, 0.1 );
  pdFootSetTrXC( &lf, 0.2 );
  pdFootSetTrXOld( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, lf._sol._k[0] );
  EXPECT_DOUBLE_EQ( 0.2, lf._sol._c[0] );
  EXPECT_DOUBLE_EQ( 0.3, lf._sol._old[0] );
}

TEST_F(pdFootTest, SetTrY)
{
  SetVacuousPrm( &lf );
  pdFootSetTrYK( &lf, 0.1 );
  pdFootSetTrYC( &lf, 0.2 );
  pdFootSetTrYOld( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, lf._sol._k[1] );
  EXPECT_DOUBLE_EQ( 0.2, lf._sol._c[1] );
  EXPECT_DOUBLE_EQ( 0.3, lf._sol._old[1] );
}

TEST_F(pdFootTest, SetTrZ)
{
  SetVacuousPrm( &lf );
  pdFootSetTrZK( &lf, 0.1 );
  pdFootSetTrZC( &lf, 0.2 );
  pdFootSetTrZOld( &lf, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, lf._sol._k[2] );
  EXPECT_DOUBLE_EQ( 0.2, lf._sol._c[2] );
  EXPECT_DOUBLE_EQ( 0.3, lf._sol._old[2] );
}

TEST_F(pdFootTest, ReferMaxHeight)
{
  EXPECT_EQ( 0, pdFootMaxHeight( &lf ) );
  pdFootZSetMaxHeight( pdFootZPtr( &lf ), 0.2 );
  EXPECT_EQ( 0.2, pdFootMaxHeight( &lf ) );
  pdFootZSetMaxHeight( pdFootZPtr( &lf ), 1.0 );
  EXPECT_EQ( 1.0, pdFootMaxHeight( &lf ) );
}

TEST_F(pdFootTest, SetMaxHeight)
{
  pdFootSetMaxHeight( &lf, 0.2 );
  EXPECT_EQ( 0.2, pdFootZMaxHeight( pdFootZPtr( &lf ) ) );
  pdFootSetMaxHeight( &lf, 1.0 );
  EXPECT_EQ( 1.0, pdFootZMaxHeight( pdFootZPtr( &lf ) ) );
}

TEST_F(pdFootTest, SetTime)
{
  SetVacuousPrm( &lf );
  lf._sol._t = 1000;

  pdFootInit( &lf, &cz, PD_FOOT_LEFT, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0.0, pdFootTime(&lf) );

  pdFootSetTime( &lf, 10 );
  EXPECT_DOUBLE_EQ( 10.0, pdFootTime(&lf) );
}

TEST_F(pdFootTest, SetTimeStep)
{
  SetVacuousPrm( &lf );
  lf._sol._dt = 1000;

  pdFootInit( &lf, &cz, PD_FOOT_LEFT, TIME_STEP );
  EXPECT_DOUBLE_EQ( TIME_STEP, pdFootTimeStep(&lf) );

  pdFootSetTimeStep( &lf, 0.001 );
  EXPECT_DOUBLE_EQ( 0.001, pdFootTimeStep(&lf) );
}

TEST_F(pdFootTest, TimeIncr)
{
  EXPECT_DOUBLE_EQ( 0, pdFootTime( &lf ) );
  pdFootIncrTime( &lf );
  EXPECT_DOUBLE_EQ( TIME_STEP, pdFootTime( &lf ) );
  pdFootIncrTime( &lf );
  EXPECT_DOUBLE_EQ( 2*TIME_STEP, pdFootTime( &lf ) );
  pdFootIncrTime( &lf );
  EXPECT_DOUBLE_EQ( 3*TIME_STEP, pdFootTime( &lf ) );
}
