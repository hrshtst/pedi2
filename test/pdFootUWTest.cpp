#include "gtest/gtest.h"
#include "mock/pdCZHrzUWCalcRegZMP_fake.h"
#include <pedi2/pd_foot_uw.h>
#include "../src/pd_foot_uw.c"

static void _pdFootUWTestDummyOverride(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos, void *util)
{
  (void)fuw; (void)delta; (void)vel; (void)regzmp; (void)refpos; (void)util;
}

class pdFootUWTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzUWInit( &czuw, &vrt );
    pdFootUWInit( &lf, &czuw );
    pdFootUWInit( &rf, &czuw );
    pdFootUWSign( &lf ) = 1;
    pdFootUWSign( &rf ) = -1;
  };
  virtual void TearDown() {
    pdFootUWDestroy( &lf );
    pdFootUWDestroy( &rf );
    pdCZHrzUWDestroy( &czuw );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdFootUWCZPtr( &lf ) = NULL;
    pdFootUWSign( &lf ) = 0.1;
    pdFootUWLandingFn( &lf ) = _pdFootUWTestDummyOverride;
    pdFootUWLandingUtil( &lf ) = &lf;
    pdFootUWPhi( &lf ) = 1;
    pdFootUWRegZMPU( &lf ) = 2;
    pdFootUWRegZMPW( &lf ) = 3;
    pdFootUWRegZMPU( &lf ) = 4;
    pdFootUWRegZMPW( &lf ) = 5;

    pdFootUWCZPtr( &rf ) = NULL;
    pdFootUWSign( &rf ) = 0.1;
    pdFootUWLandingFn( &rf ) = _pdFootUWTestDummyOverride;
    pdFootUWLandingUtil( &rf ) = &rf;
    pdFootUWPhi( &rf ) = 1;
    pdFootUWRegZMPU( &rf ) = 2;
    pdFootUWRegZMPW( &rf ) = 3;
    pdFootUWRegZMPU( &rf ) = 4;
    pdFootUWRegZMPW( &rf ) = 5;
}

  pdCZVrt vrt;
  pdCZHrzUW czuw;
  pdFootUW lf, rf;
};

TEST_F(pdFootUWTest, Init)
{
  SetVacuousPrm();
  pdFootUWInit( &lf, &czuw );
  EXPECT_EQ( &czuw, pdFootUWCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootUWSign( &lf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &lf ) );
  EXPECT_EQ( NULL, pdFootUWLandingFn( &lf ) );
  EXPECT_EQ( NULL, pdFootUWLandingUtil( &lf ) );

  pdFootUWInit( &rf, &czuw );
  EXPECT_EQ( &czuw, pdFootUWCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootUWSign( &rf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &rf ) );
  EXPECT_EQ( NULL, pdFootUWLandingFn( &rf ) );
  EXPECT_EQ( NULL, pdFootUWLandingUtil( &rf ) );
}

TEST_F(pdFootUWTest, Destroy)
{
  SetVacuousPrm();
  pdFootUWDestroy( &lf );
  EXPECT_EQ( NULL, pdFootUWCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootUWSign( &lf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &lf ) );
  EXPECT_EQ( NULL, pdFootUWLandingFn( &lf ) );
  EXPECT_EQ( NULL, pdFootUWLandingUtil( &lf ) );

  pdFootUWDestroy( &rf );
  EXPECT_EQ( NULL, pdFootUWCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootUWSign( &rf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &rf ) );
  EXPECT_EQ( NULL, pdFootUWLandingFn( &rf ) );
  EXPECT_EQ( NULL, pdFootUWLandingUtil( &rf ) );
}

TEST_F(pdFootUWTest, ReferKappa)
{
  EXPECT_EQ( 0, pdFootUWKappa( &lf ) );
  pdCZHrzUWSetKappa( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWKappa( &lf ) );
  pdCZHrzUWSetKappa( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWKappa( &lf ) );
}

TEST_F(pdFootUWTest, ReferDist)
{
  EXPECT_EQ( 0, pdFootUWDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWDist( &lf ) );
}

TEST_F(pdFootUWTest, CalcRegZMP)
{
  zVec2D d, v;

  MOCK_EXPECT_RETURN( pdCZHrzUCalcRegZMP, 0.1 );
  EXPECT_EQ( 0.1, pdFootUWCalcRegZMPU( &lf, &d, &v ) );
  MOCK_EXPECT_RETURN( pdCZHrzWCalcRegZMP, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWCalcRegZMPW( &lf, &d, &v ) );
}

TEST_F(pdFootUWTest, CalcPhi_KappaIsZero)
{
  zVec2D delta, vel, regzmp;

  pdCZHrzUWSetKappa( &czuw, 0 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &regzmp, 2, 1 );
  EXPECT_NEAR( 0, pdFootUWCalcPhi( &lf, &delta, &vel, &regzmp ), 1e-12 );
}

TEST_F(pdFootUWTest, CalcPhi_KappaIsNotZero)
{
  zVec2D delta, vel, regzmp;

  pdCZHrzUWSetKappa( &czuw, 1 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &regzmp, 2, 1 );
  EXPECT_NEAR( zPI/6.0, pdFootUWCalcPhi( &lf, &delta, &vel, &regzmp ), 1e-12 );
}

TEST_F(pdFootUWTest, CalcRefPos_KappaIsZero)
{
  zVec2D delta, vel, regzmp, refpos;

  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetDist( &czuw, 1 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &regzmp, 2, 1 );
  // left foot
  pdFootUWCalcRefPos( &lf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( 2.0,       refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)+0.5, refpos.e[pdW], 1e-12 );
  // right foot
  pdFootUWCalcRefPos( &rf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( 2.0, refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 1.0, refpos.e[pdW], 1e-12 );
}

TEST_F(pdFootUWTest, CalcRefPos_KappaIsNotZero)
{
  zVec2D delta, vel, regzmp, refpos;

  pdCZHrzUWSetKappa( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 1 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &regzmp, 2, 1 );
  // left foot
  pdFootUWCalcRefPos( &lf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( 0.25,           refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 1+1.75*sqrt(3), refpos.e[pdW], 1e-12 );
  // right foot
  pdFootUWCalcRefPos( &rf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( 2.0, refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 1.0, refpos.e[pdW], 1e-12 );
}

TEST_F(pdFootUWTest, Update)
{
  zVec2D delta, vel;

  pdCZHrzUWSetKappa( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 1 );
  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  MOCK_EXPECT_RETURN( pdCZHrzUCalcRegZMP, 2 );
  MOCK_EXPECT_RETURN( pdCZHrzWCalcRegZMP, 1 );
  // left foot
  pdFootUWUpdate( &lf, &delta, &vel );
  EXPECT_NEAR( 0.25,           pdFootUWRefPosU(&lf), 1e-12 );
  EXPECT_NEAR( 1+1.75*sqrt(3), pdFootUWRefPosW(&lf), 1e-12 );
  // right foot
  pdFootUWUpdate( &rf, &delta, &vel );
  EXPECT_NEAR( 2.0, pdFootUWRefPosU(&rf), 1e-12 );
  EXPECT_NEAR( 1.0, pdFootUWRefPosW(&rf), 1e-12 );
}

static void _pdFootUWTestFixedOverride(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos, void *util)
{
  (void)fuw; (void)delta; (void)vel; (void)regzmp;
  zVec2DCopy( (zVec2D *)util, refpos );
}

static void _pdFootUWTestShiftOverride(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos, void *util)
{
  (void)fuw; (void)delta; (void)vel; (void)regzmp; (void)util;
  refpos->e[pdW] += 1.0;
}

static void _pdFootUWTestCapturePointOverride(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos, void *util)
{
  (void)delta; (void)regzmp; (void)util;
  zVec2DCreate( refpos, vel->e[pdU] / pdFootUWZeta( fuw ), vel->e[pdW] / pdFootUWZeta( fuw ) );
}

TEST_F(pdFootUWTest, CalcRefPos_Override)
{
  zVec2D delta, vel, regzmp, refpos, target;

  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetDist( &czuw, 1 );
  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &regzmp, 2, 1 );
  zVec2DCreate( &target, -0.3, 0.7 );

  pdFootUWLandingFn( &lf ) = _pdFootUWTestFixedOverride;
  pdFootUWLandingUtil( &lf ) = &target;
  pdFootUWCalcRefPos( &lf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( -0.3, refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 0.7, refpos.e[pdW], 1e-12 );

  pdFootUWLandingFn( &rf ) = _pdFootUWTestFixedOverride;
  pdFootUWLandingUtil( &rf ) = &target;
  pdFootUWCalcRefPos( &rf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( -0.3, refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 0.7, refpos.e[pdW], 1e-12 );
  // the regulated ZMP argument is not modified by the override path
  EXPECT_NEAR( 2.0, regzmp.e[pdU], 1e-12 );
  EXPECT_NEAR( 1.0, regzmp.e[pdW], 1e-12 );
}

TEST_F(pdFootUWTest, CalcRefPos_OverrideReceivesDefault)
{
  zVec2D delta, vel, regzmp, refpos;

  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetDist( &czuw, 1 );
  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &regzmp, 2, 1 );

  // the override runs after the default rule and sees its result
  pdFootUWLandingFn( &lf ) = _pdFootUWTestShiftOverride;
  pdFootUWCalcRefPos( &lf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( 2.0, refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)+0.5+1.0, refpos.e[pdW], 1e-12 );
}

TEST_F(pdFootUWTest, CalcRefPos_CapturePointOverride)
{
  zVec2D delta, vel, regzmp, refpos;

  pdCZVrtZeta( &vrt ) = 6.0;
  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetDist( &czuw, 1 );
  zVec2DCreate( &delta, 0, 0 );
  zVec2DCreate( &vel, 0.6, 1.2 );
  zVec2DCreate( &regzmp, 2, 1 );

  pdFootUWLandingFn( &lf ) = _pdFootUWTestCapturePointOverride;
  pdFootUWCalcRefPos( &lf, &delta, &vel, &regzmp, &refpos );
  EXPECT_NEAR( 0.1, refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 0.2, refpos.e[pdW], 1e-12 );
}

TEST_F(pdFootUWTest, Update_Override)
{
  zVec2D delta, vel, target;

  MOCK_EXPECT_RETURN( pdCZHrzUCalcRegZMP, 2.0 );
  MOCK_EXPECT_RETURN( pdCZHrzWCalcRegZMP, 1.0 );
  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetDist( &czuw, 1 );
  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &vel, 0, 0 );
  zVec2DCreate( &target, -0.3, 0.7 );

  pdFootUWLandingFn( &lf ) = _pdFootUWTestFixedOverride;
  pdFootUWLandingUtil( &lf ) = &target;
  pdFootUWUpdate( &lf, &delta, &vel );
  // the stored target is the override, while phi keeps the nominal rule
  EXPECT_NEAR( -0.3, pdFootUWRefPosU( &lf ), 1e-12 );
  EXPECT_NEAR( 0.7, pdFootUWRefPosW( &lf ), 1e-12 );
  EXPECT_NEAR( 0.0, pdFootUWPhi( &lf ), 1e-12 );
}
