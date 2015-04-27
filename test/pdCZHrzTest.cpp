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
    pdCZHrzSetSR( &hrz, &sr );
    pdCZHrzZMPX( &hrz ) = 20;
    pdCZHrzZMPY( &hrz ) = 21;
    pdCZHrzAccX( &hrz ) = 22;
    pdCZHrzAccY( &hrz ) = 23;
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
  EXPECT_EQ( NULL, pdCZHrzSR( &hrz ) );
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
  EXPECT_EQ( NULL, pdCZHrzSR( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefPosX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefPosY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzRefTheta( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzPosX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzPosY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzVelX( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzVelY( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzTheta( &hrz ) );
}

TEST_F(pdCZHrzTest, SetRefPos)
{
  SetVacuousPrm();
  pdCZHrzSetRefPos( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzRefPosX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzRefPosY(&hrz) );
}
TEST_F(pdCZHrzTest, SetRefPosVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetRefPosVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzRefPosX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzRefPosY(&hrz) );
}

TEST_F(pdCZHrzTest, SetRefPosXYTheta)
{
  SetVacuousPrm();
  pdCZHrzSetRefPosX( &hrz, 0.5 );
  pdCZHrzSetRefPosY( &hrz, 0.6 );
  pdCZHrzSetRefTheta( &hrz, 0.7 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzRefPosX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzRefPosY(&hrz) );
  EXPECT_DOUBLE_EQ( 0.7, pdCZHrzRefTheta(&hrz) );
}

TEST_F(pdCZHrzTest, SetPos)
{
  SetVacuousPrm();
  pdCZHrzSetPos( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzPosX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzPosY(&hrz) );
}

TEST_F(pdCZHrzTest, SetPosVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetPosVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzPosX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzPosY(&hrz) );
}

TEST_F(pdCZHrzTest, SetPosXYTheta)
{
  SetVacuousPrm();
  pdCZHrzSetPosX( &hrz, 0.5 );
  pdCZHrzSetPosY( &hrz, 0.6 );
  pdCZHrzSetTheta( &hrz, 0.7 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzPosX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzPosY(&hrz) );
  EXPECT_DOUBLE_EQ( 0.7, pdCZHrzTheta(&hrz) );
}

TEST_F(pdCZHrzTest, SetVel)
{
  SetVacuousPrm();
  pdCZHrzSetVel( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzVelX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzVelY(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetVelVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzVelX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzVelY(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelXY)
{
  SetVacuousPrm();
  pdCZHrzSetVelX( &hrz, 0.5 );
  pdCZHrzSetVelY( &hrz, 0.6 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzVelX(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzVelY(&hrz) );
}

TEST_F(pdCZHrzTest, SetRefPosUW)
{
  SetVacuousPrm();
  pdCZHrzSetRefPosUW( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzRefPosU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzRefPosW(&hrz) );
}

TEST_F(pdCZHrzTest, SetRefPosUWVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetRefPosUWVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzRefPosU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzRefPosW(&hrz) );
}

TEST_F(pdCZHrzTest, SetRefPosUPosW)
{
  SetVacuousPrm();
  pdCZHrzSetRefPosU( &hrz, 0.5 );
  pdCZHrzSetRefPosW( &hrz, 0.6 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzRefPosU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzRefPosW(&hrz) );
}

TEST_F(pdCZHrzTest, SetDeltaM)
{
  SetVacuousPrm();
  pdCZHrzSetDelta( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzDeltaU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzDeltaW(&hrz) );
}

TEST_F(pdCZHrzTest, SetDeltaMVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetDeltaVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzDeltaU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzDeltaW(&hrz) );
}

TEST_F(pdCZHrzTest, SetDeltaUW)
{
  SetVacuousPrm();
  pdCZHrzSetDeltaU( &hrz, 0.5 );
  pdCZHrzSetDeltaW( &hrz, 0.6 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzDeltaU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzDeltaW(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelUW)
{
  SetVacuousPrm();
  pdCZHrzSetVelUW( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzVelU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzVelW(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelUWVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetVelUWVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzVelU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzVelW(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelUVelW)
{
  SetVacuousPrm();
  pdCZHrzSetVelU( &hrz, 0.5 );
  pdCZHrzSetVelW( &hrz, 0.6 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzVelU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzVelW(&hrz) );
}

TEST_F(pdCZHrzTest, RotUWtoXY_1)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( vuw, 2, 2 );
  pdCZHrzRotUWtoXY( &hrz, vuw, vxy );
  EXPECT_NEAR( -2*sqrt(2), vxy[zX], 1e-12 );
  EXPECT_NEAR( 0,          vxy[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, RotUWtoXY_2)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( vuw, 4, 2 );
  pdCZHrzRotUWtoXY( &hrz, vuw, vxy );
  EXPECT_NEAR( -sqrt(3)-2,  vxy[zX], 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)-1, vxy[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, RotXYtoUW_1)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( vxy, 2, 2 );
  pdCZHrzRotXYtoUW( &hrz, vxy, vuw );
  EXPECT_NEAR( 0,          vuw[pdU], 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), vuw[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, RotXYtoUW_2)
{
  zVec2D vuw, vxy;

  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( vxy, -2, 2 );
  pdCZHrzRotXYtoUW( &hrz, vxy, vuw );
  EXPECT_NEAR( sqrt(3)+1, vuw[pdU], 1e-12 );
  EXPECT_NEAR( sqrt(3)-1, vuw[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, XformUWtoXY_1)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( &hrz, 0, 0 );
  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( uw, 2, -2 );
  pdCZHrzXformUWtoXY( &hrz, uw, xy );
  EXPECT_NEAR( 0,         xy[zX], 1e-12 );
  EXPECT_NEAR( 2*sqrt(2), xy[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, XformUWtoXY_2)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( &hrz, 2, 1 );
  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( uw, 2, 4 );
  pdCZHrzXformUWtoXY( &hrz, uw, xy );
  EXPECT_NEAR( -2*sqrt(3)+1, xy[zX], 1e-12 );
  EXPECT_NEAR( sqrt(3)-1,    xy[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, XformXYtoUW_1)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( &hrz, 0, 0 );
  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( xy, 2, -2 );
  pdCZHrzXformXYtoUW( &hrz, xy, uw );
  EXPECT_NEAR( -2*sqrt(2), uw[pdU], 1e-12 );
  EXPECT_NEAR( 0,          uw[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, XformXYtoUW_2)
{
  zVec2D uw, xy;

  pdCZHrzSetPos( &hrz, 2, 1 );
  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( xy, 4, 5 );
  pdCZHrzXformXYtoUW( &hrz, xy, uw);
  EXPECT_NEAR( -1+2*sqrt(3), uw[pdU], 1e-12 );
  EXPECT_NEAR( -sqrt(3)-2,   uw[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, pdCZHrzXformSRXYtoUW_1)
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
  pdCZHrzSetPos( &hrz, 0, 0 );
  pdCZHrzSetTheta( &hrz, zPI/4 );

  // method to be tested
  pdCZHrzXformSRXYtoUW( &hrz, &sr_w );

  sr_m_vert = pdCZHrzSRVertUW( &hrz );
  EXPECT_NEAR( sqrt(2),    zVec3DElem(&sr_m_vert[0], zX), 1e-12 );
  EXPECT_NEAR( -sqrt(2),   zVec3DElem(&sr_m_vert[0], zY), 1e-12 );
  EXPECT_NEAR( 0,          zVec3DElem(&sr_m_vert[1], zX), 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), zVec3DElem(&sr_m_vert[1], zY), 1e-12 );
  EXPECT_NEAR( sqrt(2),    zVec3DElem(&sr_m_vert[2], zX), 1e-12 );
  EXPECT_NEAR( -3*sqrt(2), zVec3DElem(&sr_m_vert[2], zY), 1e-12 );
  EXPECT_NEAR( 2*sqrt(2),  zVec3DElem(&sr_m_vert[3], zX), 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), zVec3DElem(&sr_m_vert[3], zY), 1e-12 );
}

TEST_F(pdCZHrzTest, pdCZHrzXformSRXYtoUW_2)
{
  zVec3DList sr_w;
  zVec3D sr_w_vert[3];
  zVec3D *sr_m_vert;

  // make a convex hull
  zVec3DCreate( &sr_w_vert[0], 0, 2, 0 );
  zVec3DCreate( &sr_w_vert[1], 4, 5, 0 );
  zVec3DCreate( &sr_w_vert[2], 5, 1, 0 );
  zCH2D( &sr_w, sr_w_vert, 3 );
  pdCZHrzSetPos( &hrz, 4, 3 );
  pdCZHrzSetTheta( &hrz, zPI/6 );

  // method to be tested
  pdCZHrzXformSRXYtoUW( &hrz, &sr_w );

  sr_m_vert = pdCZHrzSRVertUW( &hrz );
  EXPECT_NEAR( -0.5*sqrt(3)+2, zVec3DElem(&sr_m_vert[0], zX), 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)+0.5,  zVec3DElem(&sr_m_vert[0], zY), 1e-12 );
  EXPECT_NEAR( -sqrt(3)-0.5,   zVec3DElem(&sr_m_vert[1], zX), 1e-12 );
  EXPECT_NEAR( -0.5*sqrt(3)+1, zVec3DElem(&sr_m_vert[1], zY), 1e-12 );
  EXPECT_NEAR( sqrt(3),        zVec3DElem(&sr_m_vert[2], zX), 1e-12 );
  EXPECT_NEAR( -1,             zVec3DElem(&sr_m_vert[2], zY), 1e-12 );
}

TEST_F(pdCZHrzTest, CalcDiffToRefPos_1)
{
  zVec2D uwd, delta;

  pdCZHrzSetKappa( &hrz, 0 );
  zVec2DCreate( uwd, 3, 2 );
  pdCZHrzCalcDiffToRefPos( &hrz, uwd, delta );
  EXPECT_NEAR( 3, delta[pdU], 1e-12 );
  EXPECT_NEAR( 2, delta[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, CalcDiffToRefPos_2)
{
  zVec2D uwd, delta;

  pdCZHrzSetKappa( &hrz, 1.0/6.0 );
  zVec2DCreate( uwd, 3, 2 );
  pdCZHrzCalcDiffToRefPos( &hrz, uwd, delta );
  EXPECT_NEAR( zPI,         delta[pdU], 1e-12 );
  EXPECT_NEAR( 3*sqrt(3)-4, delta[pdW], 1e-12 );
}
