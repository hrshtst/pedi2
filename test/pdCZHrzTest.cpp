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

TEST_F(pdCZHrzTest, XformMtoW)
{
  double X, Y;

  pdCZHrzSetPosX( &hrz, 2 );
  pdCZHrzSetPosY( &hrz, 1 );
  pdCZHrzSetTheta( &hrz, zPI/6 );
  pdCZHrzXformMtoW( &hrz, 2, 4, &X, &Y );
  EXPECT_DOUBLE_EQ( sqrt(3), X );
  EXPECT_DOUBLE_EQ( 2*sqrt(3)+2, Y );
}

TEST_F(pdCZHrzTest, XformWtoM)
{
  double u, w;

  pdCZHrzSetPosX( &hrz, 2 );
  pdCZHrzSetPosY( &hrz, 1 );
  pdCZHrzSetTheta( &hrz, zPI/6 );
  pdCZHrzXformWtoM( &hrz, 4, 5, &u, &w );
  EXPECT_DOUBLE_EQ( sqrt(3)+2, u );
  EXPECT_DOUBLE_EQ( 2*sqrt(3)-1, w );
}

TEST_F(pdCZHrzTest, RotMtoW)
{
  double vx, vy;

  pdCZHrzSetTheta( &hrz, zPI/6 );
  pdCZHrzRotMtoW( &hrz, 4, 2, &vx, &vy );
  EXPECT_DOUBLE_EQ( 2*sqrt(3)-1, vx );
  EXPECT_DOUBLE_EQ( sqrt(3)+2, vy );
}

TEST_F(pdCZHrzTest, RotWtoM)
{
  double vu, vw;

  pdCZHrzSetTheta( &hrz, zPI/6 );
  pdCZHrzRotWtoM( &hrz, -2, 2, &vu, &vw );
  EXPECT_DOUBLE_EQ( -sqrt(3)+1, vu );
  EXPECT_DOUBLE_EQ( sqrt(3)+1, vw );
}

TEST_F(pdCZHrzTest, pdCZHrzXformSRWtoM_1)
{
  zVec3DList sr_w;
  zVec3D sr_w_vert[3];
  zVec3D *sr_m_vert;

  // make a convex hull
  zVec3DCreate( &sr_w_vert[0], 0, 2, 0 );
  zVec3DCreate( &sr_w_vert[1], 4, 5, 0 );
  zVec3DCreate( &sr_w_vert[2], 5, 1, 0 );
  zCH2D( &sr_w, sr_w_vert, 3 );
  pdCZHrzSetPosX( &hrz, 4 );
  pdCZHrzSetPosY( &hrz, 3 );
  pdCZHrzSetTheta( &hrz, zPI/6 );

  // method to be tested
  pdCZHrzXformSRWtoM( &hrz, &sr_w );

  sr_m_vert = pdCZHrzSRVertM( &hrz );
  EXPECT_DOUBLE_EQ( -2*sqrt(3)-0.5, zVec3DElem(&sr_m_vert[0], zX) );
  EXPECT_DOUBLE_EQ( -0.5*sqrt(3)+2, zVec3DElem(&sr_m_vert[0], zY) );
  // EXPECT_DOUBLE_EQ( 0.5*sqrt(3)-1,  zVec3DElem(&sr_m_vert[1], zX) );
  EXPECT_NEAR( 0.5*sqrt(3)-1,  zVec3DElem(&sr_m_vert[1], zX), 1e-04 );
  EXPECT_DOUBLE_EQ( -sqrt(3)-0.5,   zVec3DElem(&sr_m_vert[1], zY) );
  EXPECT_DOUBLE_EQ( 1,              zVec3DElem(&sr_m_vert[2], zX) );
  EXPECT_DOUBLE_EQ( sqrt(3),        zVec3DElem(&sr_m_vert[2], zY) );
}

TEST_F(pdCZHrzTest, pdCZHrzXformSRWtoM_2)
{
  zVec3DList sr_w;
  zVec3D sr_w_vert[4];
  zVec3D *sr_m_vert;

  // make a convex hull
  zVec3DCreate( &sr_w_vert[0], 0, 2, 0 );
  zVec3DCreate( &sr_w_vert[1], 0, 4, 0 );
  zVec3DCreate( &sr_w_vert[2], 2, 4, 0 );
  zVec3DCreate( &sr_w_vert[3], 2, 2, 0 );
  zCH2D( &sr_w, sr_w_vert, 4 );
  pdCZHrzSetPosX( &hrz, 0 );
  pdCZHrzSetPosY( &hrz, 0 );
  pdCZHrzSetTheta( &hrz, zPI/4 );

  // method to be tested
  pdCZHrzXformSRWtoM( &hrz, &sr_w );

  sr_m_vert = pdCZHrzSRVertM( &hrz );
  EXPECT_DOUBLE_EQ( sqrt(2),   zVec3DElem(&sr_m_vert[0], zX) );
  EXPECT_DOUBLE_EQ( sqrt(2),   zVec3DElem(&sr_m_vert[0], zY) );
  EXPECT_DOUBLE_EQ( 2*sqrt(2), zVec3DElem(&sr_m_vert[1], zX) );
  // EXPECT_DOUBLE_EQ( 0,         zVec3DElem(&sr_m_vert[1], zY) );
  EXPECT_NEAR( 0,         zVec3DElem(&sr_m_vert[1], zY), 1e-04 );
  EXPECT_DOUBLE_EQ( 3*sqrt(2), zVec3DElem(&sr_m_vert[2], zX) );
  EXPECT_DOUBLE_EQ( sqrt(2),   zVec3DElem(&sr_m_vert[2], zY) );
  EXPECT_DOUBLE_EQ( 2*sqrt(2), zVec3DElem(&sr_m_vert[3], zX) );
  EXPECT_DOUBLE_EQ( 2*sqrt(2), zVec3DElem(&sr_m_vert[3], zY) );
}

TEST_F(pdCZHrzTest, CalcDiffToRefPos)
{
  double deltau, deltaw;

  pdCZHrzSetKappa( &hrz, 0 );
  pdCZHrzCalcDiffToRefPos( &hrz, 3, 2, &deltau, &deltaw );
  EXPECT_DOUBLE_EQ( 3, deltau );
  EXPECT_DOUBLE_EQ( 2, deltaw );

  pdCZHrzSetKappa( &hrz, 1.0/6.0 );
  pdCZHrzCalcDiffToRefPos( &hrz, 3, 2, &deltau, &deltaw );
  EXPECT_DOUBLE_EQ( zPI, deltau );
  EXPECT_DOUBLE_EQ( 3*sqrt(3)-4, deltaw );
}
