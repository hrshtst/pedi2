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
    lf.dy = 1;
    lf.sole_w = 0.07;
    lf.h = 1;
    zVec3DCreate( &lf.p, -1, 0, 0 );
    zVec3DClear(&lf.pd);
    zVec3DClear(&lf.ps);
    zVec3DClear(&lf.a);
    zVec3DClear(&lf.as);
    // right foot
    rf.dy = -1;
    rf.sole_w = 0.07;
    rf.h = 1;
    zVec3DCreate( &rf.p,  1, 0, 0 );
    zVec3DClear(&rf.pd);
    zVec3DClear(&rf.ps);
    zVec3DClear(&rf.a);
    zVec3DClear(&rf.as);
    // set dummy values
    lf.phase = rf.phase = 100;
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
  zVec3DElem(&lf.pd,zZ) = 100;
  zVec3DElem(&rf.pd,zZ) = 100;

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
  zVec3DElem(&lf.pd,zZ) = 100;
  zVec3DElem(&rf.pd,zZ) = 100;

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
  zVec3DElem(&lf.pd,zZ) = 100;
  zVec3DElem(&rf.pd,zZ) = 100;

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

TEST_F(pdFootTest, FootMoveWhenBothFeetOn)
{
  zVec3DElem(&lf.p,zZ) = 0;
  zVec3DElem(&rf.p,zZ) = 0;
  zVec3DCreate(&lf.pd, 1, 2, 0);
  zVec3DCreate(&rf.pd, 3, 4, 0);
  pdFootMove( &ctrl, &lf, &rf, 0, 0, 0 );
  EXPECT_EQ( 1, zVec3DElem(&lf.pd,zX) );
  EXPECT_EQ( 2, zVec3DElem(&lf.pd,zY) );
  EXPECT_EQ( 3, zVec3DElem(&rf.pd,zX) );
  EXPECT_EQ( 4, zVec3DElem(&rf.pd,zY) );
}

TEST_F(pdFootTest, FootMoveLeftFootFloating)
{
  // left foot is floating
  pdCZPrmRad(&ctrl)->dist = 2;
  zVec3DElem(&lf.p,zZ) = 0.1;
  zVec3DElem(&rf.p,zZ) = 0;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&rf.pd,zY) );

  pdCZPrmRad(&ctrl)->dist = 3;
  zVec3DElem(&lf.p,zZ) = 0.1;
  zVec3DElem(&rf.p,zZ) = 0;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( -1.5, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(    0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(    1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(    0, zVec3DElem(&rf.pd,zY) );
}

TEST_F(pdFootTest, FootMoveRightFootFloating)
{
  // right foot is floating
  pdCZPrmRad(&ctrl)->dist = 2;
  zVec3DElem(&lf.p,zZ) = 0;
  zVec3DElem(&rf.p,zZ) = 0.1;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, 0, 0 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&rf.pd,zY) );

  pdCZPrmRad(&ctrl)->dist = 3;
  zVec3DElem(&lf.p,zZ) = 0;
  zVec3DElem(&rf.p,zZ) = 0.1;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, 0, 0 );
  EXPECT_DOUBLE_EQ(  -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(   0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ( 1.5, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(   0, zVec3DElem(&rf.pd,zY) );
}

TEST_F(pdFootTest, FootMoveLeftFootFloatingRefChange)
{
  // left foot is floating
  pdCZPrmRad(&ctrl)->dist = 2;
  zVec3DElem(&lf.p,zZ) = 0.1;
  zVec3DElem(&rf.p,zZ) = 0;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, 1, 0 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&rf.pd,zY) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, -1, 0 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&rf.pd,zY) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0.5, 0, 0 );
  EXPECT_DOUBLE_EQ( -0.5, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(    0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(    1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(    0, zVec3DElem(&rf.pd,zY) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, -0.5, 0, 0 );
  EXPECT_DOUBLE_EQ( -1.5, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(    0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(    1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(    0, zVec3DElem(&rf.pd,zY) );
}

TEST_F(pdFootTest, FootMoveRightFootFloatingRefChange)
{
  // right foot is floating
  pdCZPrmRad(&ctrl)->dist = 2;
  zVec3DElem(&lf.p,zZ) = 0;
  zVec3DElem(&rf.p,zZ) = 0.1;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, 1, 0 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zY) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0, -1, 0 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&rf.pd,zY) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0.5, 0, 0 );
  EXPECT_DOUBLE_EQ(  -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(   0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ( 1.5, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(   0, zVec3DElem(&rf.pd,zY) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, -0.5, 0, 0 );
  EXPECT_DOUBLE_EQ(  -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(   0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ( 0.5, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ(   0, zVec3DElem(&rf.pd,zY) );
}

TEST_F(pdFootTest, FootMoveLeftFootFloatingRotate)
{
  double cosPI_4 = 0.5 * sqrt(2);

  // left foot is floating
  pdCZPrmRad(&ctrl)->dist = 2;
  zVec3DElem(&lf.p,zZ) = 0.1;
  zVec3DElem(&rf.p,zZ) = 0;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0.5, 0.5, 0.5*zPI_2 );
  EXPECT_DOUBLE_EQ( 0.5-cosPI_4, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ( 0.5-cosPI_4, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ( 1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zY) );
  EXPECT_DOUBLE_EQ( 1.5*zPI_2, zVec3DElem(&lf.as,zX) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0.5, -0.5, -0.5*zPI_2 );
  EXPECT_DOUBLE_EQ(  0.5-cosPI_4, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ( -0.5+cosPI_4, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ( 1, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem(&rf.pd,zY) );
  EXPECT_DOUBLE_EQ( 0.5*zPI_2, zVec3DElem(&lf.as,zX) );
}

TEST_F(pdFootTest, FootMoveRightFootFloatingRotate)
{
  double cosPI_4 = 0.5 * sqrt(2);

  // right foot is floating
  pdCZPrmRad(&ctrl)->dist = 2;
  zVec3DElem(&lf.p,zZ) = 0;
  zVec3DElem(&rf.p,zZ) = 0.1;
  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0.5, 0.5, 0.5*zPI_2 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ( 0.5+cosPI_4, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ( 0.5+cosPI_4, zVec3DElem(&rf.pd,zY) );
  EXPECT_DOUBLE_EQ( 1.5*zPI_2, zVec3DElem(&rf.as,zX) );

  zVec3DCopy(&lf.p, &lf.pd);
  zVec3DCopy(&rf.p, &rf.pd);
  pdFootMove( &ctrl, &lf, &rf, 0.5, -0.5, -0.5*zPI_2 );
  EXPECT_DOUBLE_EQ( -1, zVec3DElem(&lf.pd,zX) );
  EXPECT_DOUBLE_EQ(  0, zVec3DElem(&lf.pd,zY) );
  EXPECT_DOUBLE_EQ(  0.5+cosPI_4, zVec3DElem(&rf.pd,zX) );
  EXPECT_DOUBLE_EQ( -0.5-cosPI_4, zVec3DElem(&rf.pd,zY) );
  EXPECT_DOUBLE_EQ( 0.5*zPI_2, zVec3DElem(&rf.as,zX) );
}
