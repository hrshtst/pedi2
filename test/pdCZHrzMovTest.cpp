#include "gtest/gtest.h"
#include <pedi2/pd_cz_hrz_mov.h>

class pdCZHrzMovTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzMovInit( &hrz, &vrt );
  };
  virtual void TearDown() {
    pdCZHrzMovDestroy( &hrz );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdCZVrtSetRef( &vrt, 1 );
    pdCZHrzMovSetRefVelU( &hrz, 2 );
    pdCZHrzMovSetQ1U( &hrz, 3 );
    pdCZHrzMovSetQ2U( &hrz, 4 );
    pdCZHrzMovSetRefVelW( &hrz, 5 );
    pdCZHrzMovSetQ1W( &hrz, 6 );
    pdCZHrzMovSetQ2W( &hrz, 7 );
    pdCZHrzMovSetRho( &hrz, 8 );
    pdCZHrzMovSetKr( &hrz, 9 );
    pdCZHrzMovSetDist( &hrz, 10 );
    pdCZHrzMovSetKappa( &hrz, 11 );
    hrz._vert_num = 12;
    hrz._vrt = NULL;
    // hrz.refuz = 13;
    // hrz.refwz = 14;
    // hrz.refddu = 15;
    // hrz.refddw = 16;
    pdCZHrzMovZMPU( &hrz ) = 13;
    pdCZHrzMovZMPW( &hrz ) = 14;
    pdCZHrzMovAccU( &hrz ) = 15;
    pdCZHrzMovAccW( &hrz ) = 16;
  };

  void SetDefaultPrmVelocityFollow() {
    pdCZVrtSetRef( &vrt, 0.26 );
    pdCZHrzMovSetRefVelU( &hrz, 0.25 );
    pdCZHrzMovSetQ1U( &hrz, 1.0 );
    pdCZHrzMovSetQ2U( &hrz, 0.0 );
    pdCZHrzMovSetRefVelW( &hrz, 0.0 );
    pdCZHrzMovSetQ1W( &hrz, 1.0 );
    pdCZHrzMovSetQ2W( &hrz, 1.5 );
    pdCZHrzMovSetRho( &hrz, 1.0 );
    pdCZHrzMovSetKr( &hrz, 1.0 );
    pdCZHrzMovSetDist( &hrz, 0.1 );
    pdCZHrzMovSetKappa( &hrz, 0.0 );
  };

  void SetDefaultPrmVelocityFollowCurve() {
    SetDefaultPrmVelocityFollow();
    pdCZHrzMovSetKappa( &hrz, 2.0 );
  };

  zVec3DList sr;
  pdCZVrt vrt;
  pdCZHrzMov hrz;
};

TEST_F(pdCZHrzMovTest, Init)
{
  SetVacuousPrm();
  pdCZVrtInit( &vrt );
  pdCZHrzMovInit( &hrz, &vrt );
  EXPECT_EQ( 0, pdCZHrzMovRefVelU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRefVelW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRho( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKr( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovDist( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKappa( &hrz ) );
  EXPECT_EQ( &vrt, hrz._vrt );
  EXPECT_EQ( &vrt.zeta, hrz._u._zeta );
  EXPECT_EQ( &vrt.zeta, hrz._w._zeta );
  EXPECT_EQ( &hrz._kappa, hrz._u._kappa );
  EXPECT_EQ( &hrz._kappa, hrz._w._kappa );
  EXPECT_FALSE( pdCZHrzMovIsSRSet( &hrz ) );
  EXPECT_EQ( NULL, pdCZHrzMovSRVert( &hrz ) );
  EXPECT_EQ( 0, hrz._vert_num );
  EXPECT_EQ( 0, pdCZHrzMovZMPU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovZMPW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovAccU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovAccW( &hrz ) );
}

TEST_F(pdCZHrzMovTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzMovDestroy( &hrz );
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, pdCZHrzMovRefVelU( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2U( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRefVelW( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ1W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovQ2W( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovRho( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKr( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovDist( &hrz ) );
  EXPECT_EQ( 0, pdCZHrzMovKappa( &hrz ) );
  EXPECT_EQ( NULL, hrz._vrt );
  EXPECT_EQ( NULL, hrz._u._zeta );
  EXPECT_EQ( NULL, hrz._w._zeta );
  EXPECT_EQ( NULL, hrz._u._kappa );
  EXPECT_EQ( NULL, hrz._w._kappa );
  EXPECT_FALSE( pdCZHrzMovIsSRSet( &hrz ) );
  EXPECT_EQ( NULL, pdCZHrzMovSRVert( &hrz ) );
  EXPECT_EQ( 0, hrz._vert_num );
}

TEST_F(pdCZHrzMovTest, SetSR)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  pdCZHrzMovSetSR( &hrz, v, 3 );
  EXPECT_EQ( 0, zVec3DElem( &pdCZHrzMovSRVert(&hrz)[0], zX ) );
  EXPECT_EQ( 0, zVec3DElem( &pdCZHrzMovSRVert(&hrz)[0], zY ) );
  EXPECT_EQ( 0, zVec3DElem( &pdCZHrzMovSRVert(&hrz)[1], zX ) );
  EXPECT_EQ( 1, zVec3DElem( &pdCZHrzMovSRVert(&hrz)[1], zY ) );
  EXPECT_EQ( 1, zVec3DElem( &pdCZHrzMovSRVert(&hrz)[2], zX ) );
  EXPECT_EQ( 1, zVec3DElem( &pdCZHrzMovSRVert(&hrz)[2], zY ) );
}

TEST_F(pdCZHrzMovTest, SetSR_chk_memory)
{
  zVec3D v1[3];
  zVec3D v2[4];
  zVec3D v3[4];
  zVec3D *p1;
  zVec3D *p2;
  zVec3D *p3;

  zVec3DCreate( &v1[0], 0, 0, 0 );
  zVec3DCreate( &v1[1], 0, 1, 0 );
  zVec3DCreate( &v1[2], 1, 1, 0 );
  zVec3DCreate( &v2[0], 0, 0, 0 );
  zVec3DCreate( &v2[1], 0, 1, 0 );
  zVec3DCreate( &v2[2], 1, 1, 0 );
  zVec3DCreate( &v2[3], 1, 0, 0 );
  zVec3DCreate( &v3[0], 0, 0, 0 );
  zVec3DCreate( &v3[1], 0, 1, 0 );
  zVec3DCreate( &v3[2], 1, 1, 0 );
  zVec3DCreate( &v3[3], 1, 0, 0 );

  EXPECT_EQ( NULL, pdCZHrzMovSRVert(&hrz) );
  pdCZHrzMovSetSR( &hrz, v1, 3 );
  p1 = pdCZHrzMovSRVert(&hrz);
  EXPECT_TRUE( NULL != p1 );

  pdCZHrzMovSetSR( &hrz, v2, 4 );
  p2 = pdCZHrzMovSRVert(&hrz);
  EXPECT_NE( p1, p2 );

  pdCZHrzMovSetSR( &hrz, v3, 4 );
  p3 = pdCZHrzMovSRVert(&hrz);
  EXPECT_EQ( p2, p3 );

  pdCZHrzMovSetSR( &hrz, v1, 3 );
  p1 = pdCZHrzMovSRVert(&hrz);
  EXPECT_NE( p3, p1 );
}

TEST_F(pdCZHrzMovTest, IsSRSet)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  EXPECT_FALSE( pdCZHrzMovIsSRSet( &hrz ) );
  zCH2D( &pdCZHrzMovSR( &hrz ), v, 3 );
  EXPECT_TRUE( pdCZHrzMovIsSRSet( &hrz ) );
}

TEST_F(pdCZHrzMovTest, CheckSimZMPAllStateZero)
{
  zVec2D delta, vel, zmp;

  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzMovSetPrm( &hrz, 0.25, 1.0, 0, 0, 1.0, 1.5, 1.0, 1.0, 0.1, 0 );
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0 ); zVec2DCreate( vel, 0, 0 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, zmp[pdU] );
  EXPECT_DOUBLE_EQ( 0.0, zmp[pdW] );
}

TEST_F(pdCZHrzMovTest, CheckSimZMPVelocityFollow)
{
  zVec2D delta, vel, zmp;

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, zmp[pdU] );
  EXPECT_DOUBLE_EQ( -0.029689777531362912532664, zmp[pdW] );
}

TEST_F(pdCZHrzMovTest, CheckSimZMPVelocityFollowCurve)
{
  zVec2D delta, vel, zmp;

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( -0.006876061458783272808959, zmp[pdU] );
  EXPECT_DOUBLE_EQ( -0.031769198404332107954495, zmp[pdW] );
  // EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZHrzMovZMPU( &hrz ) );
  // EXPECT_DOUBLE_EQ( -0.001854072725677788271298,  pdCZHrzMovZMPW( &hrz ) );
}

TEST_F(pdCZHrzMovTest, CheckAccVelocityFollow)
{
  zVec2D delta, vel, zmp, acc;

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  pdCZHrzMovCalcAcc( &hrz, zmp, acc );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542, acc[pdU] );
  EXPECT_DOUBLE_EQ( 1.119835831567288941812421, acc[pdW] );
}

TEST_F(pdCZHrzMovTest, CheckAccVelocityFollowCurve)
{
  zVec2D delta, vel, zmp, acc;

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  pdCZHrzMovCalcAcc( &hrz, zmp, acc );
  EXPECT_DOUBLE_EQ( 0.259350545603461102306397, acc[pdU] );
  EXPECT_DOUBLE_EQ( 1.198267204116308493055953, acc[pdW] );
}

TEST_F(pdCZHrzMovTest, SaturationOfZMP)
{
  zVec3D v[8];
  // double uz, wz;
  zVec2D delta, vel, zmp;

  // make convex hull
  zVec3DCreate( &v[0], -0.2, -0.1, 0 );
  zVec3DCreate( &v[1],  0.2, -0.1, 0 );
  zVec3DCreate( &v[2],  0.2, -0.3, 0 );
  zVec3DCreate( &v[3], -0.2, -0.3, 0 );
  zVec3DCreate( &v[4], -0.2,  0.1, 0 );
  zVec3DCreate( &v[5], -0.2,  0.3, 0 );
  zVec3DCreate( &v[6],  0.2,  0.3, 0 );
  zVec3DCreate( &v[7],  0.2,  0.1, 0 );
  pdCZHrzMovSetSR( &hrz, v, 8 );

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0 ); zVec2DCreate( vel, 1, 1 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( 0.122120213614806391588807, zmp[pdU] );
  EXPECT_DOUBLE_EQ( 0.3, zmp[pdW] );

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0 ); zVec2DCreate( vel, 1, 1 );
  pdCZHrzMovCalcZMP( &hrz, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( 0.2, zmp[pdU] );
  EXPECT_DOUBLE_EQ( 0.3, zmp[pdW] );
}

TEST_F(pdCZHrzMovTest, UpdateVelocityFollow)
{
  zVec2D delta, vel;

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzMovUpdate( &hrz, delta, vel );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZHrzMovZMPU(&hrz) );
  EXPECT_DOUBLE_EQ( -0.029689777531362912532664, pdCZHrzMovZMPW(&hrz) );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542,  pdCZHrzMovAccU(&hrz) );
  EXPECT_DOUBLE_EQ( 1.119835831567288941812421,  pdCZHrzMovAccW(&hrz) );
}

TEST_F(pdCZHrzMovTest, UpdateVelocityFollowCurve)
{
  zVec2D delta, vel;

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdate( &vrt, 0.26, 0, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzMovUpdate( &hrz, delta, vel );
  EXPECT_DOUBLE_EQ( -0.006876061458783272808959, pdCZHrzMovZMPU(&hrz) );
  EXPECT_DOUBLE_EQ( -0.031769198404332107954495, pdCZHrzMovZMPW(&hrz) );
  EXPECT_DOUBLE_EQ(  0.259350545603461102306397, pdCZHrzMovAccU(&hrz) );
  EXPECT_DOUBLE_EQ(  1.198267204116308493055953, pdCZHrzMovAccW(&hrz) );
}
