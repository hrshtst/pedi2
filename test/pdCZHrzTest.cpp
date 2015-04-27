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

TEST_F(pdCZHrzTest, SetRefPosM)
{
  SetVacuousPrm();
  pdCZHrzSetRefPosM( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzRefPosU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzRefPosW(&hrz) );
}

TEST_F(pdCZHrzTest, SetRefPosMVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetRefPosMVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzRefPosU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzRefPosW(&hrz) );
}

TEST_F(pdCZHrzTest, SetRefPosUW)
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
  pdCZHrzSetDeltaM( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzDeltaU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzDeltaW(&hrz) );
}

TEST_F(pdCZHrzTest, SetDeltaMVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetDeltaMVec( &hrz, v );
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

TEST_F(pdCZHrzTest, SetVelM)
{
  SetVacuousPrm();
  pdCZHrzSetVelM( &hrz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZHrzVelU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZHrzVelW(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelMVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.3, 0.4 );
  pdCZHrzSetVelMVec( &hrz, v );
  EXPECT_DOUBLE_EQ( 0.3, pdCZHrzVelU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.4, pdCZHrzVelW(&hrz) );
}

TEST_F(pdCZHrzTest, SetVelUW)
{
  SetVacuousPrm();
  pdCZHrzSetVelU( &hrz, 0.5 );
  pdCZHrzSetVelW( &hrz, 0.6 );
  EXPECT_DOUBLE_EQ( 0.5, pdCZHrzVelU(&hrz) );
  EXPECT_DOUBLE_EQ( 0.6, pdCZHrzVelW(&hrz) );
}

TEST_F(pdCZHrzTest, RotMtoW_1)
{
  zVec2D vm, vw;

  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( vm, 2, 2 );
  pdCZHrzRotMtoW( &hrz, vm, vw );
  EXPECT_NEAR( -2*sqrt(2), vw[zX], 1e-12 );
  EXPECT_NEAR( 0,          vw[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, RotMtoW_2)
{
  zVec2D vm, vw;

  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( vm, 4, 2 );
  pdCZHrzRotMtoW( &hrz, vm, vw );
  EXPECT_NEAR( -sqrt(3)-2,  vw[zX], 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)-1, vw[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, RotWtoM_1)
{
  zVec2D vm, vw;

  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( vw, 2, 2 );
  pdCZHrzRotWtoM( &hrz, vw, vm );
  EXPECT_NEAR( 0,          vm[pdU], 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), vm[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, RotWtoM_2)
{
  zVec2D vm, vw;

  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( vw, -2, 2 );
  pdCZHrzRotWtoM( &hrz, vw, vm );
  EXPECT_NEAR( sqrt(3)+1, vm[pdU], 1e-12 );
  EXPECT_NEAR( sqrt(3)-1, vm[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, XformMtoW_1)
{
  zVec2D pm, pw;

  pdCZHrzSetPos( &hrz, 0, 0 );
  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( pm, 2, -2 );
  pdCZHrzXformMtoW( &hrz, pm, pw );
  EXPECT_NEAR( 0,         pw[zX], 1e-12 );
  EXPECT_NEAR( 2*sqrt(2), pw[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, XformMtoW_2)
{
  zVec2D pm, pw;

  pdCZHrzSetPos( &hrz, 2, 1 );
  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( pm, 2, 4 );
  pdCZHrzXformMtoW( &hrz, pm, pw );
  EXPECT_NEAR( -2*sqrt(3)+1, pw[zX], 1e-12 );
  EXPECT_NEAR( sqrt(3)-1,    pw[zY], 1e-12 );
}

TEST_F(pdCZHrzTest, XformWtoM_1)
{
  zVec2D pm, pw;

  pdCZHrzSetPos( &hrz, 0, 0 );
  pdCZHrzSetTheta( &hrz, zPI/4 );
  zVec2DCreate( pw, 2, -2 );
  pdCZHrzXformWtoM( &hrz, pw, pm );
  EXPECT_NEAR( -2*sqrt(2), pm[pdU], 1e-12 );
  EXPECT_NEAR( 0,          pm[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, XformWtoM_2)
{
  zVec2D pm, pw;

  pdCZHrzSetPos( &hrz, 2, 1 );
  pdCZHrzSetTheta( &hrz, zPI/6 );
  zVec2DCreate( pw, 4, 5 );
  pdCZHrzXformWtoM( &hrz, pw, pm);
  EXPECT_NEAR( -1+2*sqrt(3), pm[pdU], 1e-12 );
  EXPECT_NEAR( -sqrt(3)-2,   pm[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, pdCZHrzXformSRWtoM_1)
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
  pdCZHrzXformSRWtoM( &hrz, &sr_w );

  sr_m_vert = pdCZHrzSRVertM( &hrz );
  EXPECT_NEAR( sqrt(2),    zVec3DElem(&sr_m_vert[0], zX), 1e-12 );
  EXPECT_NEAR( -sqrt(2),   zVec3DElem(&sr_m_vert[0], zY), 1e-12 );
  EXPECT_NEAR( 0,          zVec3DElem(&sr_m_vert[1], zX), 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), zVec3DElem(&sr_m_vert[1], zY), 1e-12 );
  EXPECT_NEAR( sqrt(2),    zVec3DElem(&sr_m_vert[2], zX), 1e-12 );
  EXPECT_NEAR( -3*sqrt(2), zVec3DElem(&sr_m_vert[2], zY), 1e-12 );
  EXPECT_NEAR( 2*sqrt(2),  zVec3DElem(&sr_m_vert[3], zX), 1e-12 );
  EXPECT_NEAR( -2*sqrt(2), zVec3DElem(&sr_m_vert[3], zY), 1e-12 );
}

TEST_F(pdCZHrzTest, pdCZHrzXformSRWtoM_2)
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
  pdCZHrzXformSRWtoM( &hrz, &sr_w );

  sr_m_vert = pdCZHrzSRVertM( &hrz );
  EXPECT_NEAR( -0.5*sqrt(3)+2, zVec3DElem(&sr_m_vert[0], zX), 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)+0.5,  zVec3DElem(&sr_m_vert[0], zY), 1e-12 );
  EXPECT_NEAR( -sqrt(3)-0.5,   zVec3DElem(&sr_m_vert[1], zX), 1e-12 );
  EXPECT_NEAR( -0.5*sqrt(3)+1, zVec3DElem(&sr_m_vert[1], zY), 1e-12 );
  EXPECT_NEAR( sqrt(3),        zVec3DElem(&sr_m_vert[2], zX), 1e-12 );
  EXPECT_NEAR( -1,             zVec3DElem(&sr_m_vert[2], zY), 1e-12 );
}

TEST_F(pdCZHrzTest, CalcDiffToRefPos_1)
{
  zVec2D pd, delta;

  pdCZHrzSetKappa( &hrz, 0 );
  zVec2DCreate( pd, 3, 2 );
  pdCZHrzCalcDiffToRefPos( &hrz, pd, delta );
  EXPECT_NEAR( 3, delta[pdU], 1e-12 );
  EXPECT_NEAR( 2, delta[pdW], 1e-12 );
}

TEST_F(pdCZHrzTest, CalcDiffToRefPos_2)
{
  zVec2D pd, delta;

  pdCZHrzSetKappa( &hrz, 1.0/6.0 );
  zVec2DCreate( pd, 3, 2 );
  pdCZHrzCalcDiffToRefPos( &hrz, pd, delta );
  EXPECT_NEAR( zPI,         delta[pdU], 1e-12 );
  EXPECT_NEAR( 3*sqrt(3)-4, delta[pdW], 1e-12 );
}
