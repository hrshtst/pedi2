#include "gtest/gtest.h"
#include <zm/zm_complex.h>
#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>

class pdFootTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdCZInit( &ctrl );
    pdCZPrmRad(&ctrl)->rho = 1;
    pdCZPrmRad(&ctrl)->dist = 2;
    // left foot
    zVec3DCreate( &lf.p, -1, 0, 0 );
    lf.dy = 1;
    lf.sole_w = 0.07;
    lf.h = 1;
    // right foot
    zVec3DCreate( &rf.p,  1, 0, 0 );
    rf.dy = -1;
    rf.sole_w = 0.07;
    rf.h = 1;
    // set dummy values
    lf.phase = rf.phase = 100;
    zVec3DElem(&lf.pd,zZ) = 100;
    zVec3DElem(&rf.pd,zZ) = 100;
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

TEST_F(pdFootTest, FootLiftDoubleSupport)
{
  zComplexCreate(&pz, 0, 1 );
  pdFootLift( &ctrl, &lf, &rf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&lf.pd,zZ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zZ) );

  zComplexCreate(&pz, 0, -1 );
  pdFootLift( &ctrl, &lf, &rf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&lf.pd,zZ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zZ) );
}

TEST_F(pdFootTest, FootLiftSingleSupport)
{
  zComplexCreate(&pz, 1, 0 );
  pdFootLift( &ctrl, &lf, &rf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&lf.pd,zZ) );
  EXPECT_DOUBLE_EQ( 1, zVec3DElem(&rf.pd,zZ) );

  zComplexCreate(&pz, -1, 0 );
  pdFootLift( &ctrl, &lf, &rf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 1, zVec3DElem(&lf.pd,zZ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zZ) );
}

TEST_F(pdFootTest, FootLiftRhoIsZero)
{
  pdCZPrmRad(&ctrl)->rho = 0;

  zComplexCreate(&pz, 1, 0 );
  pdFootLift( &ctrl, &lf, &rf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&lf.pd,zZ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zZ) );

  zComplexCreate(&pz, -1, 0 );
  pdFootLift( &ctrl, &lf, &rf, 0, 0, 0, &pz );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&lf.pd,zZ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zZ) );
}

// TODO: add test for calculation of foot phase
//       when (dx, dy) != (0, 0) and theta != 0

TEST_F(pdFootTest, FootIsOn)
{
  zVec3DElem(&lf.p,zZ) = 0;
  zVec3DElem(&rf.p,zZ) = 0;
  EXPECT_TRUE( pdFootIsOn( &lf ) );
  EXPECT_TRUE( pdFootIsOn( &rf ) );

  zVec3DElem(&lf.p,zZ) = 0.1;
  zVec3DElem(&rf.p,zZ) = 0.1;
  EXPECT_FALSE( pdFootIsOn( &lf ) );
  EXPECT_FALSE( pdFootIsOn( &rf ) );
}

TEST_F(pdFootTest, FootDoesAttemptToLift)
{
  zVec3DElem(&lf.pd,zZ) = 0;
  zVec3DElem(&rf.pd,zZ) = 0;
  EXPECT_FALSE( pdFootDoesAttemptToLift( &lf ) );
  EXPECT_FALSE( pdFootDoesAttemptToLift( &rf ) );

  zVec3DElem(&lf.pd,zZ) = 0.1;
  zVec3DElem(&rf.pd,zZ) = 0.1;
  EXPECT_TRUE( pdFootDoesAttemptToLift( &lf ) );
  EXPECT_TRUE( pdFootDoesAttemptToLift( &rf ) );
}
