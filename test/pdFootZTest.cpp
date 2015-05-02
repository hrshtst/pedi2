#include "gtest/gtest.h"
#include <pedi2/pd_foot_z.h>

class pdFootZTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzUWInit( &czuw, &vrt );
    pdFootZInit( &lf, &czuw );
    pdFootZInit( &rf, &czuw );
    pdFootZSign( &lf ) = 1;
    pdFootZSign( &rf ) = -1;
  };
  virtual void TearDown() {
    pdFootZDestroy( &lf );
    pdFootZDestroy( &rf );
    pdCZHrzUWDestroy( &czuw );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdFootZCZPtr( &lf ) = NULL;
    pdFootZSetMaxHeight( &lf, 100 );
    pdFootZSign( &lf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &lf ), 10, 20 );
    pdFootZFootPhase( &lf ) = 30;
    pdFootZRefZ( &lf ) = 40;
    lf._sr_vert = dummy1;
    lf._vert_num = 50;

    pdFootZCZPtr( &rf ) = NULL;
    pdFootZSetMaxHeight( &rf, 100 );
    pdFootZSign( &rf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &rf ), 10, 20 );
    pdFootZFootPhase( &rf ) = 30;
    pdFootZRefZ( &rf ) = 40;
    rf._sr_vert = dummy2;
    rf._vert_num = 50;
  }

  // zVec3DList lsr, rsr;
  zVec3D *dummy1, *dummy2;
  pdCZVrt vrt;
  pdCZHrzUW czuw;
  pdFootZ lf, rf;
};

TEST_F(pdFootZTest, Init)
{
  SetVacuousPrm();
  pdFootZInit( &lf, &czuw );
  EXPECT_EQ( &czuw, pdFootZCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &lf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( NULL, pdFootZSRVert( &lf ) );
  EXPECT_EQ( 0, lf._vert_num );
  EXPECT_EQ( 0, pdFootZSign( &lf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &lf ) );

  pdFootZInit( &rf, &czuw );
  EXPECT_EQ( &czuw, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &rf ) );
  EXPECT_EQ( NULL, pdFootZSRVert( &rf ) );
  EXPECT_EQ( 0, rf._vert_num );
  EXPECT_EQ( 0, pdFootZSign( &rf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &rf ) );
}

TEST_F(pdFootZTest, Destroy)
{
  SetVacuousPrm();
  pdFootZInit( &lf, &czuw );
  pdFootZDestroy( &lf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &lf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( NULL, pdFootZSRVert( &lf ) );
  EXPECT_EQ( 0, lf._vert_num );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &lf ) );

  pdFootZInit( &rf, &czuw );
  pdFootZDestroy( &rf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &rf ) );
  EXPECT_EQ( NULL, pdFootZSRVert( &rf ) );
  EXPECT_EQ( 0, rf._vert_num );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &rf ) );
}

TEST_F(pdFootZTest, ReferRho)
{
  EXPECT_EQ( 0, pdFootZRho( &lf ) );
  pdCZHrzUWSetRho( &czuw, 1 );
  EXPECT_EQ( 1, pdFootZRho( &lf ) );
  pdCZHrzUWSetRho( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootZRho( &lf ) );
}

TEST_F(pdFootZTest, ReferDist)
{
  EXPECT_EQ( 0, pdFootZDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 1 );
  EXPECT_EQ( 1, pdFootZDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootZDist( &lf ) );
}

TEST_F(pdFootZTest, SetSR)
{
  zVec3D v[3];
  zVec3D vv[4];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_EQ( 0, zVec3DElem( &pdFootZSRVert(&lf)[0], zX ) );
  EXPECT_EQ( 0, zVec3DElem( &pdFootZSRVert(&lf)[0], zY ) );
  EXPECT_EQ( 0, zVec3DElem( &pdFootZSRVert(&lf)[1], zX ) );
  EXPECT_EQ( 1, zVec3DElem( &pdFootZSRVert(&lf)[1], zY ) );
  EXPECT_EQ( 1, zVec3DElem( &pdFootZSRVert(&lf)[2], zX ) );
  EXPECT_EQ( 1, zVec3DElem( &pdFootZSRVert(&lf)[2], zY ) );

  zVec3DCreate( &v[0], 0.1, 0.1, 0 );
  zVec3DCreate( &v[1], 0.1, 0.2, 0 );
  zVec3DCreate( &v[2], 0.2, 0.2, 0 );
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[0], zX ) );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[0], zY ) );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[1], zX ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[1], zY ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[2], zX ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[2], zY ) );

  zVec3DCreate( &vv[0], 0.1, 0.1, 0 );
  zVec3DCreate( &vv[1], 0.1, 0.2, 0 );
  zVec3DCreate( &vv[2], 0.2, 0.2, 0 );
  zVec3DCreate( &vv[3], 0.2, 0.1, 0 );
  pdFootZSetSR( &lf, vv, 4 );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[0], zX ) );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[0], zY ) );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[1], zX ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[1], zY ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[2], zX ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[2], zY ) );
  EXPECT_EQ( 0.2, zVec3DElem( &pdFootZSRVert(&lf)[3], zX ) );
  EXPECT_EQ( 0.1, zVec3DElem( &pdFootZSRVert(&lf)[3], zY ) );
}

TEST_F(pdFootZTest, SetSR_chk_memory)
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

  EXPECT_EQ( NULL, pdFootZSRVert(&lf) );
  pdFootZSetSR( &lf, v1, 3 );
  p1 = pdFootZSRVert(&lf);
  EXPECT_TRUE( NULL != p1 );

  pdFootZSetSR( &lf, v2, 4 );
  p2 = pdFootZSRVert(&lf);
  EXPECT_NE( p1, p2 );

  pdFootZSetSR( &lf, v3, 4 );
  p3 = pdFootZSRVert(&lf);
  EXPECT_EQ( p2, p3 );

  pdFootZSetSR( &lf, v1, 3 );
  p1 = pdFootZSRVert(&lf);
  EXPECT_NE( p3, p1 );
}

TEST_F(pdFootZTest, IsSRSet)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_TRUE( pdFootZIsSRSet( &lf ) );
}

TEST_F(pdFootZTest, SetSR_None)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  // set supporting region
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_TRUE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 3, lf._vert_num );
  // set None
  pdFootZSetSR( &lf, NULL, 0 );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 0, lf._vert_num );
  // set supporting region again
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_TRUE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 3, lf._vert_num );
}

TEST_F(pdFootZTest, CalcFootPhase_SingleSupportLeft)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;
  zComplex pz;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );

  // Now ZMP is out of the supporting region,
  zVec2DCreate( delta, 0, 0.05 );
  zVec2DCreate( vel,   0, 0.05 );
  zVec2DCreate( zmp,   0, 0.07 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // comes into the supporting region,
  zVec2DCreate( delta, 0, 0.02 );
  zVec2DCreate( vel,   0, 0.1 );
  zVec2DCreate( zmp,   0, 0.1 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // comes at the most leftward,
  zVec2DCreate( delta, 0, -0.02 );
  zVec2DCreate( vel,   0, 0.0 );
  zVec2DCreate( zmp,   0, 0.125 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0.5, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // get back to the edge again,
  zVec2DCreate( delta, 0, 0.02 );
  zVec2DCreate( vel,   0, -0.1 );
  zVec2DCreate( zmp,   0, 0.1 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // and go far away
  zVec2DCreate( delta, 0, 0.05 );
  zVec2DCreate( vel,   0, -0.05 );
  zVec2DCreate( zmp,   0, 0.07 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcFootPhase_OrthogonalSRRight)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;
  zComplex pz;

  // make convex hull of right foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &lf, NULL, 0 );
  pdFootZSetSR( &rf, v, 4 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );

  // Now ZMP is out of the supporting region,
  zVec2DCreate( delta, 0, -0.05 );
  zVec2DCreate( vel,   0, -0.05 );
  zVec2DCreate( zmp,   0, -0.07 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // comes into the supporting region,
  zVec2DCreate( delta, 0, -0.02 );
  zVec2DCreate( vel,   0, -0.1 );
  zVec2DCreate( zmp,   0, -0.1 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // comes at the most righttward,
  zVec2DCreate( delta, 0, 0.02 );
  zVec2DCreate( vel,   0, 0.0 );
  zVec2DCreate( zmp,   0, -0.125 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0.5, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // get back to the edge again,
  zVec2DCreate( delta, 0, -0.02 );
  zVec2DCreate( vel,   0, 0.1 );
  zVec2DCreate( zmp,   0, -0.1 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  // and go far away
  zVec2DCreate( delta, 0, -0.05 );
  zVec2DCreate( vel,   0, 0.05 );
  zVec2DCreate( zmp,   0, -0.07 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcFootPhase_DoubleSupport)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;
  zComplex pz;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &rf, v, 4 );

  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );

  // always 0 or 1 when double support phase
  zVec2DCreate( delta, 0, -0.05 );
  zVec2DCreate( vel,   0, -0.02 );
  zVec2DCreate( zmp,   0, 0.07 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
  //
  zVec2DCreate( delta, 0, 0.05 );
  zVec2DCreate( vel,   0, 0.02 );
  zVec2DCreate( zmp,   0, -0.07 );
  pdFootZCalcZMPPhase( &lf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, delta, &pz ), 1e-12 );
  pdFootZCalcZMPPhase( &rf, delta, vel, zmp, &pz );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, delta, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcRefZ_RhoIsOne)
{
  double phase;
  zComplex pz;

  pdCZHrzUWSetRho( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 1 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  zComplexCreate( &pz, 0.5, 0 );

  phase = 0;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 0.5;
  EXPECT_NEAR( 0.8, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 1;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcRefZ_RhoIsLowerThanOne)
{
  double phase;
  zComplex pz;

  // e^-1 = 0.36787944117144233
  pdCZHrzUWSetRho( &czuw, 0.8 );
  pdCZHrzUWSetDist( &czuw, 1 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  zComplexCreate( &pz, 0.5, 0 );

  phase = 0;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 0.5;
  EXPECT_GT( pdFootZCalcRefZ( &lf, phase, &pz ), 0 );
  EXPECT_LT( pdFootZCalcRefZ( &lf, phase, &pz ), 0.8 );
  phase = 1;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcRefZ_RhoIsNearlyZero)
{
  double phase;
  zComplex pz;

  // e^-1 = 0.36787944117144233
  pdCZHrzUWSetRho( &czuw, 0.2 );
  pdCZHrzUWSetDist( &czuw, 1 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  zComplexCreate( &pz, 0.5, 0 );

  phase = 0;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 0.5;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 1;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, Update_SingleSupportLeftIsOn)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );
  pdCZHrzUWSetRho( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 0.25 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  pdFootZSetMaxHeight( &rf, 0.8 );

  // Now ZMP is out of the left foot
  zVec2DCreate( delta, 0, 0.05 );
  zVec2DCreate( vel,   0, 0.05 );
  zVec2DCreate( zmp,   0, 0.07 );
  pdFootZUpdate( &lf, &rf, delta, vel, zmp );
  pdFootZUpdate( &rf, &lf, delta, vel, zmp );
  EXPECT_NEAR( 0, pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &rf ), 1e-12 );

  // right foot can be lifted
  zVec2DCreate( delta, 0, 0.0 );
  zVec2DCreate( vel,   0, 0.0 );
  zVec2DCreate( zmp,   0, 0.125 );
  pdFootZUpdate( &lf, &rf, delta, vel, zmp );
  pdFootZUpdate( &rf, &lf, delta, vel, zmp );
  EXPECT_NEAR( 0,   pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0,   pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0.5, pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0.8, pdFootZRefZ( &rf ), 1e-12 );
}

TEST_F(pdFootZTest, Update_SingleSupportRightIsOn)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of right foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &lf, NULL, 0 );
  pdFootZSetSR( &rf, v, 4 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );
  pdCZHrzUWSetRho( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 0.25 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  pdFootZSetMaxHeight( &rf, 0.8 );

  // Now ZMP is out of the right foot
  zVec2DCreate( delta, 0, -0.05 );
  zVec2DCreate( vel,   0, -0.05 );
  zVec2DCreate( zmp,   0, -0.07 );
  pdFootZUpdate( &lf, &rf, delta, vel, zmp );
  pdFootZUpdate( &rf, &lf, delta, vel, zmp );
  EXPECT_NEAR( 0, pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &rf ), 1e-12 );
  // left foot can be lifted
  zVec2DCreate( delta, 0, 0.0 );
  zVec2DCreate( vel,   0, 0.0 );
  zVec2DCreate( zmp,   0, -0.125 );
  pdFootZUpdate( &lf, &rf, delta, vel, zmp );
  pdFootZUpdate( &rf, &lf, delta, vel, zmp );
  EXPECT_NEAR( 0.5, pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0.8, pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0,   pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0,   pdFootZRefZ( &rf ), 1e-12 );
}
