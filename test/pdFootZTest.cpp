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
    pdFootZSetSR( &lf, &rsr ); // purposely set pointer to rsr
    pdFootZSign( &lf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &lf ), 10, 20 );
    pdFootZFootPhase( &lf ) = 30;
    pdFootZRefHeight( &lf ) = 40;

    pdFootZCZPtr( &rf ) = NULL;
    pdFootZSetMaxHeight( &rf, 100 );
    pdFootZSetSR( &rf, &lsr ); // purposely set pointer to lsr
    pdFootZSign( &rf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &rf ), 10, 20 );
    pdFootZFootPhase( &rf ) = 30;
    pdFootZRefHeight( &rf ) = 40;
}

  zVec3DList lsr, rsr;
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
  EXPECT_EQ( NULL, pdFootZSR( &lf ) );
  EXPECT_EQ( 0, pdFootZSign( &lf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &lf ) );

  pdFootZInit( &rf, &czuw );
  EXPECT_EQ( &czuw, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_EQ( NULL, pdFootZSR( &rf ) );
  EXPECT_EQ( 0, pdFootZSign( &rf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &rf ) );
}

TEST_F(pdFootZTest, Destroy)
{
  SetVacuousPrm();
  pdFootZDestroy( &lf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &lf ) );
  EXPECT_EQ( NULL, pdFootZSR( &lf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &lf ) );

  pdFootZDestroy( &rf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_EQ( NULL, pdFootZSR( &rf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &rf ) );
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
  zCH2D( &lsr, v, 4 );
  pdFootZSetSR( &lf, &lsr );
  pdFootZSetSR( &rf, NULL );
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
  zCH2D( &rsr, v, 4 );
  pdFootZSetSR( &lf, NULL );
  pdFootZSetSR( &rf, &rsr );
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
  zVec3D vl[4], vr[4];
  zVec2D delta, vel, zmp;
  zComplex pz;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &vl[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &vl[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &vl[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &vl[3],  0.04, 0.15, 0.0 );
  zCH2D( &lsr, vl, 4 );
  zVec3DCreate( &vr[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &vr[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &vr[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &vr[3],  0.04, -0.15, 0.0 );
  zCH2D( &rsr, vr, 4 );

  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );
  pdFootZSetSR( &lf, &lsr );
  pdFootZSetSR( &rf, &rsr );

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
