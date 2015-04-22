#include "gtest/gtest.h"
#include <pedi2/pd_cz_hrz.h>

class pdCZHrzTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzInit( &hrz, &vrt );
  };
  virtual void TearDown() {
    pdCZHrzDestroy( &hrz );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdCZVrtSetRef( &vrt, 1 );
    pdCZHrzSetRefPosX( &hrz, 2 );
    pdCZHrzSetRefPosY( &hrz, 3 );
    pdCZHrzSetRefTheta( &hrz, 4 );
    pdCZHrzSetPosX( &hrz, 5 );
    pdCZHrzSetPosY( &hrz, 6 );
    pdCZHrzSetVelX( &hrz, 7 );
    pdCZHrzSetVelY( &hrz, 8 );
    pdCZHrzSetTheta( &hrz, 9 );
    pdCZHrzSetRefVelU( &hrz, 10 );
    pdCZHrzSetQ1U( &hrz, 11 );
    pdCZHrzSetQ2U( &hrz, 12 );
    pdCZHrzSetRefVelW( &hrz, 13 );
    pdCZHrzSetQ1W( &hrz, 14 );
    pdCZHrzSetQ2W( &hrz, 15 );
    pdCZHrzSetRho( &hrz, 16 );
    pdCZHrzSetKr( &hrz, 17 );
    pdCZHrzSetDist( &hrz, 18 );
    pdCZHrzSetKappa( &hrz, 19 );
    hrz._sr = &sr;
    hrz.refxz = 20;
    hrz.refyz = 21;
    hrz.refddx = 22;
    hrz.refddy = 23;
  };

  zVec3DList sr;
  pdCZVrt vrt;
  pdCZHrz hrz;
};

TEST_F(pdCZHrzTest, Init)
{
  SetVacuousPrm();
  pdCZVrtInit( &vrt );
  pdCZHrzInit( &hrz, &vrt );
  EXPECT_EQ( 0, pdCZHrzRefVelU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ1U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ2U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefVelW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ1W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ2W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRho( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzKr( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzDist( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzKappa( &hrz ) );
  EXPECT_EQ( NULL, hrz._sr );
  EXPECT_EQ( 0, pdCZHrzRefPosX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefPosY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefTheta( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzPosX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzPosY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzVelX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzVelY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzTheta( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzZMPX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzZMPY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzAccX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzAccY( &hrz ) );
}

TEST_F(pdCZHrzTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzDestroy( &hrz );
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, pdCZHrzRefVelU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ1U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ2U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefVelW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ1W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzQ2W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRho( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzKr( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzDist( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzKappa( &hrz ) );
  EXPECT_EQ( NULL, hrz._sr );
  EXPECT_EQ( 0, pdCZHrzRefPosX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefPosY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefTheta( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzPosX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzPosY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzVelX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzVelY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzTheta( &hrz ) );
}
