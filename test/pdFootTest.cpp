#include "gtest/gtest.h"
#include <zm/zm_complex.h>
#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>

class pdFootTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdCZInit( &ctrl );
    // left foot
    zVec3DCreate( &lf.p, -1, 0, 0 );
    lf.dy = 1;
    lf.sole_w = 0.07;
    // right foot
    zVec3DCreate( &rf.p,  1, 0, 0 );
    rf.dy = -1;
    rf.sole_w = 0.07;
  };
  virtual void TearDown() {};

  pdCZ ctrl;
  pdFoot lf;
  pdFoot rf;
  zComplex pz;
};

TEST_F(pdFootTest, CalcFootPhaseDoubleSupport)
{
  zComplexCreate(&pz, 0, 1 );
  pdFootPhase( &lf, &rf, 0, 0, 0, &pz );
  pdFootPhase( &rf, &lf, 0, 0, 0, &pz );
  EXPECT_EQ( 0, lf.phase );
  EXPECT_EQ( 1, rf.phase );

  zComplexCreate(&pz, 0, -1 );
  pdFootPhase( &lf, &rf, 0, 0, 0, &pz );
  pdFootPhase( &rf, &lf, 0, 0, 0, &pz );
  EXPECT_EQ( 1, lf.phase );
  EXPECT_EQ( 0, rf.phase );
}

TEST_F(pdFootTest, CalcFootPhaseSingleSupport)
{
  zComplexCreate(&pz, 1, 0 );
  pdFootPhase( &lf, &rf, 0, 0, 0, &pz );
  pdFootPhase( &rf, &lf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0, lf.phase );
  EXPECT_DOUBLE_EQ( 0.5, rf.phase );

  zComplexCreate(&pz, -1, 0 );
  pdFootPhase( &lf, &rf, 0, 0, 0, &pz );
  pdFootPhase( &rf, &lf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0.5, lf.phase );
  EXPECT_DOUBLE_EQ( 0, rf.phase );
}

// TODO: add test for calculation of foot phase
//       when (dx, dy) != (0, 0) and theta != 0
