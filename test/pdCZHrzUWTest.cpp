#include "gtest/gtest.h"
#include <pedi2/pd_cz_hrz_uw.h>

class pdCZHrzUWTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzUWInit( &uw, &vrt );
  };
  virtual void TearDown() {
    pdCZHrzUWDestroy( &uw );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdCZVrtSetRef( &vrt, 1 );
    pdCZHrzUWSetRefVelU( &uw, 2 );
    pdCZHrzUWSetQ1U( &uw, 3 );
    pdCZHrzUWSetQ2U( &uw, 4 );
    pdCZHrzUWSetRefVelW( &uw, 5 );
    pdCZHrzUWSetQ1W( &uw, 6 );
    pdCZHrzUWSetQ2W( &uw, 7 );
    pdCZHrzUWSetRho( &uw, 8 );
    pdCZHrzUWSetKr( &uw, 9 );
    pdCZHrzUWSetDist( &uw, 10 );
    pdCZHrzUWSetKappa( &uw, 11 );
    uw._vert_num = 12;
    uw._vrt = NULL;
    pdCZHrzUWZMPU( &uw ) = 13;
    pdCZHrzUWZMPW( &uw ) = 14;
    pdCZHrzUWAccU( &uw ) = 15;
    pdCZHrzUWAccW( &uw ) = 16;
  };

  void SetDefaultPrmVelocityFollow() {
    pdCZVrtSetRef( &vrt, 0.26 );
    pdCZHrzUWSetRefVelU( &uw, 0.25 );
    pdCZHrzUWSetQ1U( &uw, 1.0 );
    pdCZHrzUWSetQ2U( &uw, 0.0 );
    pdCZHrzUWSetRefVelW( &uw, 0.0 );
    pdCZHrzUWSetQ1W( &uw, 1.0 );
    pdCZHrzUWSetQ2W( &uw, 1.5 );
    pdCZHrzUWSetRho( &uw, 1.0 );
    pdCZHrzUWSetKr( &uw, 1.0 );
    pdCZHrzUWSetDist( &uw, 0.1 );
    pdCZHrzUWSetKappa( &uw, 0.0 );
  };

  void SetDefaultPrmVelocityFollowCurve() {
    SetDefaultPrmVelocityFollow();
    pdCZHrzUWSetKappa( &uw, 2.0 );
  };

  zVec3DList sr;
  pdCZVrt vrt;
  pdCZHrzUW uw;
};

TEST_F(pdCZHrzUWTest, Init)
{
  SetVacuousPrm();
  pdCZVrtInit( &vrt );
  pdCZHrzUWInit( &uw, &vrt );
  EXPECT_EQ( 0, pdCZHrzUWRefVelU( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ1U( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ2U( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWRefVelW( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ1W( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ2W( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWRho( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWKr( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWDist( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWKappa( &uw ) );
  EXPECT_EQ( &vrt, uw._vrt );
  EXPECT_EQ( &vrt.zeta, uw._u._zeta );
  EXPECT_EQ( &vrt.zeta, uw._w._zeta );
  EXPECT_EQ( &uw._kappa, uw._u._kappa );
  EXPECT_EQ( &uw._kappa, uw._w._kappa );
  EXPECT_FALSE( pdCZHrzUWIsSRSet( &uw ) );
  EXPECT_EQ( NULL, pdCZHrzUWSRVert( &uw ) );
  EXPECT_EQ( 0, uw._vert_num );
  EXPECT_EQ( 0, pdCZHrzUWZMPU( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWZMPW( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWAccU( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWAccW( &uw ) );
}

TEST_F(pdCZHrzUWTest, Destroy)
{
  SetVacuousPrm();
  pdCZHrzUWDestroy( &uw );
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, pdCZHrzUWRefVelU( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ1U( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ2U( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWRefVelW( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ1W( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWQ2W( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWRho( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWKr( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWDist( &uw ) );
  EXPECT_EQ( 0, pdCZHrzUWKappa( &uw ) );
  EXPECT_EQ( NULL, uw._vrt );
  EXPECT_EQ( NULL, uw._u._zeta );
  EXPECT_EQ( NULL, uw._w._zeta );
  EXPECT_EQ( NULL, uw._u._kappa );
  EXPECT_EQ( NULL, uw._w._kappa );
  EXPECT_FALSE( pdCZHrzUWIsSRSet( &uw ) );
  EXPECT_EQ( NULL, pdCZHrzUWSRVert( &uw ) );
  EXPECT_EQ( 0, uw._vert_num );
}

TEST_F(pdCZHrzUWTest, SetSR)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  pdCZHrzUWSetSR( &uw, v, 3 );
  EXPECT_EQ( 0, zVec3DElem( &pdCZHrzUWSRVert(&uw)[0], zX ) );
  EXPECT_EQ( 0, zVec3DElem( &pdCZHrzUWSRVert(&uw)[0], zY ) );
  EXPECT_EQ( 0, zVec3DElem( &pdCZHrzUWSRVert(&uw)[1], zX ) );
  EXPECT_EQ( 1, zVec3DElem( &pdCZHrzUWSRVert(&uw)[1], zY ) );
  EXPECT_EQ( 1, zVec3DElem( &pdCZHrzUWSRVert(&uw)[2], zX ) );
  EXPECT_EQ( 1, zVec3DElem( &pdCZHrzUWSRVert(&uw)[2], zY ) );
}

TEST_F(pdCZHrzUWTest, SetSR_chk_memory)
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

  EXPECT_EQ( NULL, pdCZHrzUWSRVert(&uw) );
  pdCZHrzUWSetSR( &uw, v1, 3 );
  p1 = pdCZHrzUWSRVert(&uw);
  EXPECT_TRUE( NULL != p1 );

  pdCZHrzUWSetSR( &uw, v2, 4 );
  p2 = pdCZHrzUWSRVert(&uw);
  EXPECT_NE( p1, p2 );

  pdCZHrzUWSetSR( &uw, v3, 4 );
  p3 = pdCZHrzUWSRVert(&uw);
  EXPECT_EQ( p2, p3 );

  pdCZHrzUWSetSR( &uw, v1, 3 );
  p1 = pdCZHrzUWSRVert(&uw);
  EXPECT_NE( p3, p1 );
}

TEST_F(pdCZHrzUWTest, IsSRSet)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  EXPECT_FALSE( pdCZHrzUWIsSRSet( &uw ) );
  zCH2D( pdCZHrzUWSR( &uw ), v, 3 );
  EXPECT_TRUE( pdCZHrzUWIsSRSet( &uw ) );
}

TEST_F(pdCZHrzUWTest, CheckSimZMPAllStateZero)
{
  zVec2D delta, vel, zmp;

  pdCZVrtSetRef( &vrt, 0.26 );
  pdCZHrzUWSetPrm( &uw, 0.25, 1.0, 0, 0, 1.0, 1.5, 1.0, 1.0, 0.1, 0 );
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0 ); zVec2DCreate( vel, 0, 0 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, zmp[pdU] );
  EXPECT_DOUBLE_EQ( 0.0, zmp[pdW] );
}

TEST_F(pdCZHrzUWTest, CheckSimZMPVelocityFollow)
{
  zVec2D delta, vel, zmp;

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, zmp[pdU] );
  EXPECT_DOUBLE_EQ( -0.029689777531362912532664, zmp[pdW] );
}

TEST_F(pdCZHrzUWTest, CheckSimZMPVelocityFollowCurve)
{
  zVec2D delta, vel, zmp;

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( -0.006876061458783272808959, zmp[pdU] );
  EXPECT_DOUBLE_EQ( -0.031769198404332107954495, zmp[pdW] );
  // EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZHrzUWZMPU( &uw ) );
  // EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCZHrzUWZMPW( &uw ) );
}

TEST_F(pdCZHrzUWTest, CheckAccVelocityFollow)
{
  zVec2D delta, vel, zmp, acc;

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  pdCZHrzUWCalcAcc( &uw, zmp, acc );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542, acc[pdU] );
  EXPECT_DOUBLE_EQ( 1.119835831567288941812421, acc[pdW] );
}

TEST_F(pdCZHrzUWTest, CheckAccVelocityFollowCurve)
{
  zVec2D delta, vel, zmp, acc;

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  pdCZHrzUWCalcAcc( &uw, zmp, acc );
  EXPECT_DOUBLE_EQ( 0.259350545603461102306397, acc[pdU] );
  EXPECT_DOUBLE_EQ( 1.198267204116308493055953, acc[pdW] );
}

TEST_F(pdCZHrzUWTest, SaturationOfZMP)
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
  pdCZHrzUWSetSR( &uw, v, 8 );

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0 ); zVec2DCreate( vel, 1, 1 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( 0.122120213614806391588807, zmp[pdU] );
  EXPECT_DOUBLE_EQ( 0.3, zmp[pdW] );

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0 ); zVec2DCreate( vel, 1, 1 );
  pdCZHrzUWCalcZMP( &uw, delta, vel, zmp );
  EXPECT_DOUBLE_EQ( 0.2, zmp[pdU] );
  EXPECT_DOUBLE_EQ( 0.3, zmp[pdW] );
}

TEST_F(pdCZHrzUWTest, CalcRegZMP)
{
  zVec2D delta, vel, regzmp;

  pdCZVrtZeta( &vrt ) = 1;
  pdCZHrzUWSetPrm( &uw, 0.2, 1, 1, 0.2, 1, 1, 1, 1, 1, 1 );
  zVec2DCreate( delta, 0.2, -0.2 ); zVec2DCreate( vel, 0.2, -0.2 );
  pdCZHrzUWCalcRegZMP( &uw, delta, vel, regzmp );
  EXPECT_NEAR( 0.14,  regzmp[pdU], 1e-12 );
  EXPECT_NEAR( -0.25, regzmp[pdW], 1e-12 );
  zVec2DCreate( delta, -0.2, -0.2 ); zVec2DCreate( vel, 0.2, 0.2 );
  pdCZHrzUWCalcRegZMP( &uw, delta, vel, regzmp );
  EXPECT_NEAR( 0.66, regzmp[pdU], 1e-12 );
  EXPECT_NEAR( 0.55, regzmp[pdW], 1e-12 );
}

TEST_F(pdCZHrzUWTest, CalcZMPPhase_1)
{
  zVec2D delta, vel, zmp;
  zComplex pz;

  pdCZVrtZeta( &vrt ) = 1;
  pdCZHrzUWSetQ1W( &uw, 1 );
  pdCZHrzUWSetQ2W( &uw, 1 );
  zVec2DCreate( delta, 0, 0 );
  zVec2DCreate( vel, 0, 0 );
  zVec2DCreate( zmp, 0, 0 );
  pdCZHrzUWCalcZMPPhase( &uw, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pz.re, 1e-12 );
  EXPECT_NEAR( 0, pz.im, 1e-12 );
}

TEST_F(pdCZHrzUWTest, CalcZMPPhase_2)
{
  zVec2D delta, vel, zmp;
  zComplex pz;

  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &uw, 1 );
  pdCZHrzUWSetQ2W( &uw, 1 );
  zVec2DCreate( delta, 0, 0.2 );
  zVec2DCreate( vel, 0, 0.5 );
  zVec2DCreate( zmp, 0, 0.1 );
  pdCZHrzUWCalcZMPPhase( &uw, delta, vel, zmp, &pz );
  EXPECT_NEAR( -0.1, pz.re, 1e-12 );
  EXPECT_NEAR( -0.5, pz.im, 1e-12 );
}

TEST_F(pdCZHrzUWTest, UpdateVelocityFollow)
{
  zVec2D delta, vel;

  SetDefaultPrmVelocityFollow();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzUWUpdate( &uw, delta, vel );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZHrzUWZMPU(&uw) );
  EXPECT_DOUBLE_EQ( -0.029689777531362912532664, pdCZHrzUWZMPW(&uw) );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542,  pdCZHrzUWAccU(&uw) );
  EXPECT_DOUBLE_EQ( 1.119835831567288941812421,  pdCZHrzUWAccW(&uw) );
}

TEST_F(pdCZHrzUWTest, UpdateVelocityFollowCurve)
{
  zVec2D delta, vel;

  SetDefaultPrmVelocityFollowCurve();
  pdCZVrtUpdateZeta( &vrt, 0.26, 0, 0 );
  zVec2DCreate( delta, 0, 0.01 ); zVec2DCreate( vel, 0.2, 0.1 );
  pdCZHrzUWUpdate( &uw, delta, vel );
  EXPECT_DOUBLE_EQ( -0.006876061458783272808959, pdCZHrzUWZMPU(&uw) );
  EXPECT_DOUBLE_EQ( -0.031769198404332107954495, pdCZHrzUWZMPW(&uw) );
  EXPECT_DOUBLE_EQ(  0.259350545603461102306397, pdCZHrzUWAccU(&uw) );
  EXPECT_DOUBLE_EQ(  1.198267204116308493055953, pdCZHrzUWAccW(&uw) );
}
