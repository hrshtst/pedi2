#include "gtest/gtest.h"
#include "mock/pdCZHrzUWCalcRegZMP_fake.h"
#include <pedi2/pd_foot_uw.h>
#include "../src/pd_foot_uw.c"

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
    pdFootUWPhi( &lf ) = 1;
    pdFootUWRegZMPU( &lf ) = 2;
    pdFootUWRegZMPW( &lf ) = 3;
    pdFootUWRegZMPU( &lf ) = 4;
    pdFootUWRegZMPW( &lf ) = 5;

    pdFootUWCZPtr( &rf ) = NULL;
    pdFootUWSign( &rf ) = 0.1;
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

  pdFootUWInit( &rf, &czuw );
  EXPECT_EQ( &czuw, pdFootUWCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootUWSign( &rf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &rf ) );
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

  pdFootUWDestroy( &rf );
  EXPECT_EQ( NULL, pdFootUWCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootUWSign( &rf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &rf ) );
}

TEST_F(pdFootUWTest, ReferKappa)
{
  EXPECT_EQ( 0, pdFootUWKappa( &lf ) );
  pdCZHrzUWSetKappa( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWKappa( &lf ) );
  pdCZHrzUWSetKappa( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWKappa( &lf ) );
}

TEST_F(pdFootUWTest, ReferLambda)
{
  EXPECT_EQ( 0, pdFootUWLambda( &lf ) );
  pdCZHrzUWSetLambda( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWLambda( &lf ) );
  pdCZHrzUWSetLambda( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWLambda( &lf ) );
}

TEST_F(pdFootUWTest, ReferCanonDist)
{
  EXPECT_EQ( 0, pdFootUWCanonDist( &lf ) );
  pdCZHrzUWSetCanonDist( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWCanonDist( &lf ) );
  pdCZHrzUWSetCanonDist( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWCanonDist( &lf ) );
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
  zVec2D delta, regzmp;

  pdCZHrzUWSetKappa( &czuw, 0 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &regzmp, 2, 1 );
  EXPECT_NEAR( 0, pdFootUWCalcPhi( &lf, &delta, &regzmp ), 1e-12 );
}

TEST_F(pdFootUWTest, CalcPhi_KappaIsNotZero)
{
  zVec2D delta, regzmp;

  pdCZHrzUWSetKappa( &czuw, 1 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &regzmp, 2, 1 );
  EXPECT_NEAR( zPI/6.0, pdFootUWCalcPhi( &lf, &delta, &regzmp ), 1e-12 );
}

TEST_F(pdFootUWTest, CalcPhiLambda_LambdaIsZero)
{
  double refdist;

  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetLambda( &czuw, 0 );
  pdCZHrzUWSetCanonDist( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, sqrt(191) );

  refdist = sqrt(197);
  EXPECT_NEAR( 0, pdFootUWCalcPhiLambda( &lf, pdCZHrzUWDist(&czuw), refdist ), 1e-12 );
}

TEST_F(pdFootUWTest, CalcPhiLambda_LambdaIsNotZero)
{
  double refdist;
  double expected;

  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetLambda( &czuw, 0.3 );
  pdCZHrzUWSetCanonDist( &czuw, 2.0 );
  pdCZHrzUWSetDist( &czuw, 2*sqrt(5) );
  refdist = 2*3;

  expected = atan2(72.0-6.0*sqrt(35), 16.0*sqrt(7)+27.0*sqrt(5));
  EXPECT_NEAR( expected, pdFootUWCalcPhiLambda( &lf, pdCZHrzUWDist(&czuw), refdist ), 1e-12 );
}

TEST_F(pdFootUWTest, CalcRefPos_KappaIsZero)
{
  zVec2D delta, regzmp, refpos;

  pdCZHrzUWSetKappa( &czuw, 0 );
  pdCZHrzUWSetDist( &czuw, 1 );

  zVec2DCreate( &delta, 0, 2*sqrt(3) );
  zVec2DCreate( &regzmp, 2, 1 );
  // left foot
  pdFootUWCalcRefPos( &lf, &delta, &regzmp, &refpos );
  EXPECT_NEAR( 2.0,       refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 2*sqrt(3)+0.5, refpos.e[pdW], 1e-12 );
  // right foot
  pdFootUWCalcRefPos( &rf, &delta, &regzmp, &refpos );
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
  pdFootUWCalcRefPos( &lf, &delta, &regzmp, &refpos );
  EXPECT_NEAR( 0.25,           refpos.e[pdU], 1e-12 );
  EXPECT_NEAR( 1+1.75*sqrt(3), refpos.e[pdW], 1e-12 );
  // right foot
  pdFootUWCalcRefPos( &rf, &delta, &regzmp, &refpos );
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
