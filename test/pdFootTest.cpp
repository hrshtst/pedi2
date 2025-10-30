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

  zLoop3D sr, dummy;
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

TEST_F(pdFootTest, IsOn_IsOff)
{
  pdFootPosZ( &lf ) = 0.0;
  pdFootSR( &lf ) = &sr;
  EXPECT_TRUE( pdFootIsOn( &lf ) );
  EXPECT_FALSE( pdFootIsOff( &lf ) );

  pdFootPosZ( &lf ) = 0.1;
  pdFootSR( &lf ) = NULL;
  EXPECT_FALSE( pdFootIsOn( &lf ) );
  EXPECT_TRUE( pdFootIsOff( &lf ) );

  pdFootPosZ( &lf ) = 0.1;
  pdFootSR( &lf ) = &sr;
  EXPECT_FALSE( pdFootIsOn( &lf ) );
  EXPECT_TRUE( pdFootIsOff( &lf ) );

  pdFootPosZ( &lf ) = 0.0;
  pdFootSR( &lf ) = NULL;
  EXPECT_FALSE( pdFootIsOn( &lf ) );
  EXPECT_TRUE( pdFootIsOff( &lf ) );
}

TEST_F(pdFootTest, IsOnNext_IsOffNext)
{
  pdFootRefPosZ( &lf ) = 0.0;
  EXPECT_TRUE( pdFootIsOnNext( &lf ) );
  EXPECT_FALSE( pdFootIsOffNext( &lf ) );

  pdFootRefPosZ( &lf ) = 0.1;
  EXPECT_FALSE( pdFootIsOnNext( &lf ) );
  EXPECT_TRUE( pdFootIsOffNext( &lf ) );
}

TEST_F(pdFootTest, IsOnAttempt_IsOffAttempt)
{
  pdFootDesPosZ( &lf ) = 0.0;
  EXPECT_TRUE( pdFootIsOnAttempt( &lf ) );
  EXPECT_FALSE( pdFootIsOffAttempt( &lf ) );

  pdFootDesPosZ( &lf ) = 0.1;
  EXPECT_FALSE( pdFootIsOnAttempt( &lf ) );
  EXPECT_TRUE( pdFootIsOffAttempt( &lf ) );
}

TEST_F(pdFootTest, IsBothOn)
{
  pdFootPosZ( &lf ) = 0.0;
  pdFootPosZ( &rf ) = 0.0;
  pdFootSR( &lf ) = &sr;
  pdFootSR( &rf ) = &sr;
  EXPECT_TRUE( pdFootIsBothOn( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOn( &lf, &rf ) );
  EXPECT_FALSE( pdFootIsEitherOff( &lf, &rf ) );

  pdFootPosZ( &lf ) = 0.1;
  pdFootPosZ( &rf ) = 0.0;
  pdFootSR( &lf ) = NULL;
  pdFootSR( &rf ) = &sr;
  EXPECT_FALSE( pdFootIsBothOn( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOn( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOff( &lf, &rf ) );

  pdFootPosZ( &lf ) = 0.0;
  pdFootPosZ( &rf ) = 0.1;
  pdFootSR( &lf ) = &sr;
  pdFootSR( &rf ) = NULL;
  EXPECT_FALSE( pdFootIsBothOn( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOn( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOff( &lf, &rf ) );
}

TEST_F(pdFootTest, IsBothOnNext)
{
  pdFootRefPosZ( &lf ) = 0.0;
  pdFootRefPosZ( &rf ) = 0.0;
  EXPECT_TRUE( pdFootIsBothOnNext( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOnNext( &lf, &rf ) );
  EXPECT_FALSE( pdFootIsEitherOffNext( &lf, &rf ) );

  pdFootRefPosZ( &lf ) = 0.1;
  pdFootRefPosZ( &rf ) = 0.0;
  EXPECT_FALSE( pdFootIsBothOnNext( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOnNext( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOffNext( &lf, &rf ) );

  pdFootRefPosZ( &lf ) = 0.0;
  pdFootRefPosZ( &rf ) = 0.1;
  EXPECT_FALSE( pdFootIsBothOnNext( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOnNext( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOffNext( &lf, &rf ) );
}

TEST_F(pdFootTest, IsBothOnAttempt)
{
  pdFootDesPosZ( &lf ) = 0.0;
  pdFootDesPosZ( &rf ) = 0.0;
  EXPECT_TRUE( pdFootIsBothOnAttempt( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOnAttempt( &lf, &rf ) );
  EXPECT_FALSE( pdFootIsEitherOffAttempt( &lf, &rf ) );

  pdFootDesPosZ( &lf ) = 0.1;
  pdFootDesPosZ( &rf ) = 0.0;
  EXPECT_FALSE( pdFootIsBothOnAttempt( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOnAttempt( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOffAttempt( &lf, &rf ) );

  pdFootDesPosZ( &lf ) = 0.0;
  pdFootDesPosZ( &rf ) = 0.1;
  EXPECT_FALSE( pdFootIsBothOnAttempt( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOnAttempt( &lf, &rf ) );
  EXPECT_TRUE( pdFootIsEitherOffAttempt( &lf, &rf ) );
}


TEST_F(pdFootTest, RotUWtoXY_1)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/4 );
  zVec2DCreate( &vuw, 2, 2 );
  pdFootRotUWtoXY( &lf, &vuw, &vxy );
  EXPECT_NEAR( -2*sqrt(2), vxy.c.x, 1e-12 );
  EXPECT_NEAR( 0,          vxy.c.y, 1e-12 );
}

TEST_F(pdFootTest, RotUWtoXY_2)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/6 );
  zVec2DCreate( &vuw, 4, 2 );
  pdFootRotUWtoXY( &lf, &vuw, &vxy );
  EXPECT_NEAR( -sqrt(3)-2,  vxy.c.x, 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)-1, vxy.c.y, 1e-12 );
}

TEST_F(pdFootTest, RotXYtoUW_1)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/4 );
  zVec2DCreate( &vxy, 2, 2 );
  pdFootRotXYtoUW( &lf, &vxy, &vuw );
  EXPECT_NEAR( 0,          vuw.e[pdU], 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), vuw.e[pdW], 1e-12 );
}

TEST_F(pdFootTest, RotXYtoUW_2)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/6 );
  zVec2DCreate( &vxy, -2, 2 );
  pdFootRotXYtoUW( &lf, &vxy, &vuw );
  EXPECT_NEAR( sqrt(3)+1, vuw.e[pdU], 1e-12 );
  EXPECT_NEAR( sqrt(3)-1, vuw.e[pdW], 1e-12 );
}

TEST_F(pdFootTest, XformUWtoXY_1)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( pdFootCZPtr( &lf ), 0, 0 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/4 );
  zVec2DCreate( &uw, 2, -2 );
  pdFootXformUWtoXY( &lf, &uw, &xy );
  EXPECT_NEAR( 0,         xy.c.x, 1e-12 );
  EXPECT_NEAR( 2*sqrt(2), xy.c.y, 1e-12 );
}

TEST_F(pdFootTest, XformUWtoXY_2)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( pdFootCZPtr( &lf ), 2, 1 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/6 );
  zVec2DCreate( &uw, 2, 4 );
  pdFootXformUWtoXY( &lf, &uw, &xy );
  EXPECT_NEAR( -2*sqrt(3)+1, xy.c.x, 1e-12 );
  EXPECT_NEAR( sqrt(3)-1,    xy.c.y, 1e-12 );
}

TEST_F(pdFootTest, XformXYtoUW_1)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( pdFootCZPtr( &lf ), 0, 0 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/4 );
  zVec2DCreate( &xy, 2, -2 );
  pdFootXformXYtoUW( &lf, &xy, &uw );
  EXPECT_NEAR( -2*sqrt(2), uw.e[pdU], 1e-12 );
  EXPECT_NEAR( 0,          uw.e[pdW], 1e-12 );
}

TEST_F(pdFootTest, XformXYtoUW_2)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( pdFootCZPtr( &lf ), 2, 1 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/6 );
  zVec2DCreate( &xy, 4, 5 );
  pdFootXformXYtoUW( &lf, &xy, &uw);
  EXPECT_NEAR( -1+2*sqrt(3), uw.e[pdU], 1e-12 );
  EXPECT_NEAR( -sqrt(3)-2,   uw.e[pdW], 1e-12 );
}

TEST_F(pdFootTest, XformSRXYtoUW_1)
{
  zLoop3D sr_w;
  zVec3D sr_w_vert[4];
  zVec3DData sr_w_data;
  zVec3DData *sr_m_data;

  // make a convex hull
  zVec3DCreate( &sr_w_vert[0], 0, 2, 0 );
  zVec3DCreate( &sr_w_vert[1], 0, 4, 0 );
  zVec3DCreate( &sr_w_vert[2], 2, 4, 0 );
  zVec3DCreate( &sr_w_vert[3], 2, 2, 0 );
  zVec3DDataAssignArrayDirect( &sr_w_data, sr_w_vert, 4 );
  zVec3DDataConvexHull2D( &sr_w_data, &sr_w );
  pdCZHrzSetPos( pdFootCZPtr( &lf ), 0, 0 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/4 );

  // method to be tested
  pdFootXformSRXYtoUW( &lf, &sr_w );

  sr_m_data = pdFootZSRVert( pdFootZPtr( &lf ) );
  EXPECT_NEAR( sqrt(2),    zArrayElem( sr_m_data->data.array, 0 )->c.x, 1e-12 );
  EXPECT_NEAR( -sqrt(2),   zArrayElem( sr_m_data->data.array, 0 )->c.y, 1e-12 );
  EXPECT_NEAR( 0,          zArrayElem( sr_m_data->data.array, 1 )->c.x, 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), zArrayElem( sr_m_data->data.array, 1 )->c.y, 1e-12 );
  EXPECT_NEAR( sqrt(2),    zArrayElem( sr_m_data->data.array, 2 )->c.x, 1e-12 );
  EXPECT_NEAR( -3*sqrt(2), zArrayElem( sr_m_data->data.array, 2 )->c.y, 1e-12 );
  EXPECT_NEAR( 2*sqrt(2),  zArrayElem( sr_m_data->data.array, 3 )->c.x, 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), zArrayElem( sr_m_data->data.array, 3 )->c.y, 1e-12 );
}

TEST_F(pdFootTest, XformSRXYtoUW_2)
{
  zLoop3D sr_w;
  zVec3D sr_w_vert[3];
  zVec3DData sr_w_data;
  zVec3DData *sr_m_data;

  // make a convex hull
  zVec3DCreate( &sr_w_vert[0], 0, 2, 0 );
  zVec3DCreate( &sr_w_vert[1], 4, 5, 0 );
  zVec3DCreate( &sr_w_vert[2], 5, 1, 0 );
  zVec3DDataAssignArrayDirect( &sr_w_data, sr_w_vert, 3 );
  zVec3DDataConvexHull2D( &sr_w_data, &sr_w );
  pdCZHrzSetPos( pdFootCZPtr( &lf ), 4, 3 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/6 );

  // method to be tested
  pdFootXformSRXYtoUW( &lf, &sr_w );

  sr_m_data = pdFootZSRVert( pdFootZPtr( &lf ) );
  EXPECT_NEAR( -0.5*sqrt(3)+2, zArrayElem( sr_m_data->data.array, 0 )->c.x, 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)+0.5,  zArrayElem( sr_m_data->data.array, 0 )->c.y, 1e-12 );
  EXPECT_NEAR( -sqrt(3)-0.5,   zArrayElem( sr_m_data->data.array, 1 )->c.x, 1e-12 );
  EXPECT_NEAR( -0.5*sqrt(3)+1, zArrayElem( sr_m_data->data.array, 1 )->c.y, 1e-12 );
  EXPECT_NEAR( sqrt(3),        zArrayElem( sr_m_data->data.array, 2 )->c.x, 1e-12 );
  EXPECT_NEAR( -1,             zArrayElem( sr_m_data->data.array, 2 )->c.y, 1e-12 );
}

TEST_F(pdFootTest, XformSRXYtoUW_NULL)
{
  zLoop3D sr_w, sr_dummy;
  zVec3D sr_w_vert[3];
  zVec3DData sr_w_data;

  // make a convex hull
  zVec3DCreate( &sr_w_vert[0], 0, 2, 0 );
  zVec3DCreate( &sr_w_vert[1], 4, 5, 0 );
  zVec3DCreate( &sr_w_vert[2], 5, 1, 0 );
  zVec3DDataAssignArrayDirect( &sr_w_data, sr_w_vert, 3 );
  zVec3DDataConvexHull2D( &sr_w_data, &sr_w );
  pdCZHrzSetPos( pdFootCZPtr( &lf ), 4, 3 );
  pdCZHrzSetTheta( pdFootCZPtr( &lf ), zPI/6 );

  pdFootXformSRXYtoUW( &lf, &sr_w );
  EXPECT_EQ( 3, pdFootZPtr(&lf)->_vert_num );

  zListInit( &sr_dummy );
  pdFootXformSRXYtoUW( &lf, &sr_dummy );
  EXPECT_EQ( 0, pdFootZPtr(&lf)->_vert_num );

  pdFootXformSRXYtoUW( &lf, NULL );
  EXPECT_EQ( 0, pdFootZPtr(&lf)->_vert_num );
}

TEST_F(pdFootTest, pdFootCalcRefPos)
{
  zVec3D p, pd, refp;
  int i;

  pdFootSetTimeStep( &lf, 0.01 );
  zVec3DCreate( &p, 0, 0, 0 );
  zVec3DCreate( &pd, 0.5, 0.5, 0.5 );
  pdFootSetTrXK( &lf, 3000 );
  pdFootSetTrXC( &lf, 50 );
  pdFootSetTrXOld( &lf, 0 );
  pdFootSetTrYK( &lf, 3000 );
  pdFootSetTrYC( &lf, 50 );
  pdFootSetTrYOld( &lf, 0 );
  pdFootSetTrZK( &lf, 3000 );
  pdFootSetTrZC( &lf, 50 );
  pdFootSetTrZOld( &lf, 0 );
  for( i=0; i<100; i++ ){
    pdFootCalcRefPos( &lf, &p, &pd, &refp );
    zVec3DCopy( &refp, &p );
  }
  EXPECT_NEAR( 0.5, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.5, p.c.y, 1e-12 );
  EXPECT_NEAR( 0.5, p.c.y, 1e-12 );
}

TEST_F(pdFootTest, pdFootCalcRefPos_2)
{
  zVec3D p, pd, refp;
  int i;

  pdFootSetTimeStep( &lf, 0.01 );
  zVec3DCreate( &p, 0.1, 0.1, 0.1 );
  zVec3DCreate( &pd, 0.1, 0.1, 0.1 );
  pdFootSetTrXK( &lf, 7000 );
  pdFootSetTrXC( &lf, 200 );
  pdFootSetTrXOld( &lf, 0 );
  pdFootSetTrYK( &lf, 7000 );
  pdFootSetTrYC( &lf, 200 );
  pdFootSetTrYOld( &lf, 0 );
  pdFootSetTrZK( &lf, 7000 );
  pdFootSetTrZC( &lf, 200 );
  pdFootSetTrZOld( &lf, 0 );
  for( i=0; i<100; i++ ){
    pdFootCalcRefPos( &lf, &p, &pd, &refp );
    zVec3DCopy( &refp, &p );
  }
  EXPECT_NEAR( 0.1, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.1, p.c.y, 1e-12 );
  EXPECT_NEAR( 0.1, p.c.y, 1e-12 );
}

TEST_F(pdFootTest, pdFootCalcRefAtt)
{
  zVec3D pd, refa;
  double offset;

  offset = zPI_2;
  zVec3DZero( &pd );
  pdFootCalcRefAtt( &lf, &pd, &refa );
  // EXPECT_NEAR( 0, refa.c.x, 1e-12 );
  EXPECT_NEAR( offset, refa.c.x, 1e-12 );
  EXPECT_NEAR( 0, refa.c.y, 1e-12 );
  EXPECT_NEAR( 0, refa.c.z, 1e-12 );

  pdCZHrzSetTheta( &cz, zPI_2 );
  pdFootCalcRefAtt( &lf, &pd, &refa );
  // EXPECT_NEAR( zPI_2, refa.c.x, 1e-12 );
  EXPECT_NEAR( zPI_2+offset, refa.c.x, 1e-12 );
  EXPECT_NEAR( 0,     refa.c.y, 1e-12 );
  EXPECT_NEAR( 0,     refa.c.z, 1e-12 );

  pdCZHrzSetTheta( &cz, zPI_2 );
  pdFootUWPhi( pdFootUWPtr( &lf ) ) = zPI / 6.0;
  pdFootCalcRefAtt( &lf, &pd, &refa );
  // EXPECT_NEAR( 2.0*zPI/3.0, refa.c.x, 1e-12 );
  EXPECT_NEAR( 2.0*zPI/3.0+offset, refa.c.x, 1e-12 );
  EXPECT_NEAR( 0,     refa.c.y, 1e-12 );
  EXPECT_NEAR( 0,     refa.c.z, 1e-12 );
}

TEST_F(pdFootTest, UpdateState)
{
  zVec3D p, a;
  zVec3DCreate( &p, 0.1, 0.2, 0.0 );
  zVec3DCreate( &a, 0.3, 0.4, 0.5 );

  // on the ground
  pdFootUpdateState( &lf, &p, &a, &sr );
  EXPECT_EQ( 0.1, pdFootPosX(&lf) );
  EXPECT_EQ( 0.2, pdFootPosY(&lf) );
  EXPECT_EQ( 0.0, pdFootPosZ(&lf) );
  EXPECT_EQ( 0.3, pdFootAttX(&lf) );
  EXPECT_EQ( 0.4, pdFootAttY(&lf) );
  EXPECT_EQ( 0.5, pdFootAttZ(&lf) );
  EXPECT_EQ( &sr, pdFootSR(&lf) );
  EXPECT_EQ( 0.1, pdFootPivotPosX(&lf) );
  EXPECT_EQ( 0.2, pdFootPivotPosY(&lf) );
  EXPECT_EQ( 0.0, pdFootPivotPosZ(&lf) );
  EXPECT_EQ( 0.3, pdFootPivotAttX(&lf) );
  EXPECT_EQ( 0.4, pdFootPivotAttY(&lf) );
  EXPECT_EQ( 0.5, pdFootPivotAttZ(&lf) );

  // off the ground
  zVec3DCreate( &p, 0.1, 0.2, 0.01 );
  zVec3DCreate( &a, 0.6, 0.0, 0.0 );
  pdFootUpdateState( &lf, &p, &a, NULL );
  EXPECT_EQ( 0.1, pdFootPosX(&lf) );
  EXPECT_EQ( 0.2, pdFootPosY(&lf) );
  EXPECT_EQ( 0.01, pdFootPosZ(&lf) );
  EXPECT_EQ( 0.6, pdFootAttX(&lf) );
  EXPECT_EQ( 0.0, pdFootAttY(&lf) );
  EXPECT_EQ( 0.0, pdFootAttZ(&lf) );
  EXPECT_EQ( NULL, pdFootSR(&lf) );
  EXPECT_EQ( 0.1, pdFootPivotPosX(&lf) );
  EXPECT_EQ( 0.2, pdFootPivotPosY(&lf) );
  EXPECT_EQ( 0.0, pdFootPivotPosZ(&lf) );
  EXPECT_EQ( 0.3, pdFootPivotAttX(&lf) );
  EXPECT_EQ( 0.4, pdFootPivotAttY(&lf) );
  EXPECT_EQ( 0.5, pdFootPivotAttZ(&lf) );
}
