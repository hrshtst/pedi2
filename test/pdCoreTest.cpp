#include "gtest/gtest.h"
#include <pedi2/pd_core.h>

const double TIME_STEP = 0.01;
#define GTEST_TOL 1e-12
#define GTEST_TOL_LOOSE 1e-04

class pdCoreTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCoreInit( &core, &cmd, TIME_STEP );
    destroy_flag = false;
  };
  virtual void TearDown() {
    if( !destroy_flag )
      pdCoreDestroy( &core );
  };

  bool destroy_flag;
  pdCore core;
  pdCmd cmd;
};

TEST_F(pdCoreTest, Init)
{
  EXPECT_EQ( 0, pdCoreTime( &core ) );
  EXPECT_EQ( TIME_STEP, pdCoreTimeStep( &core ) );
  EXPECT_EQ( &cmd, pdCoreCmd( &core ) );
  EXPECT_EQ( &core.cz, pdCoreCZPtr( &core ) );
  EXPECT_EQ( &core.lf, pdCoreLFPtr( &core ) );
  EXPECT_EQ( &core.rf, pdCoreRFPtr( &core ) );
  EXPECT_EQ( &core.robot, pdCoreRobotPtr( &core ) );
  EXPECT_EQ( &core.state, pdCoreStatePtr( &core ) );
}

TEST_F(pdCoreTest, Destroy)
{
  pdCoreDestroy( &core );
  EXPECT_EQ( 0, pdCoreTime( &core ) );
  EXPECT_EQ( 0, pdCoreTimeStep( &core ) );
  EXPECT_EQ( NULL, pdCoreCmd( &core ) );
  destroy_flag = true;
}

TEST_F(pdCoreTest, SetTime)
{
  pdCoreSetTime( &core, 10 );
  EXPECT_EQ( 10, pdCoreTime( &core ) );
  EXPECT_EQ( 10, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 10, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 10, pdFootTime( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, ResetTime)
{
  pdCoreSetTime( &core, 5 );
  EXPECT_EQ( 5, pdCoreTime( &core ) );
  EXPECT_EQ( 5, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 5, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 5, pdFootTime( pdCoreRFPtr( &core ) ) );
  pdCoreResetTime( &core );
  EXPECT_EQ( 0, pdCoreTime( &core ) );
  EXPECT_EQ( 0, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 0, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 0, pdFootTime( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, SetTimeStep)
{
  pdCoreSetTimeStep( &core, 0.005 );
  EXPECT_EQ( 0.005, pdCoreTimeStep( &core ) );
  EXPECT_EQ( 0.005, pdCZTimeStep( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 0.005, pdFootTimeStep( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 0.005, pdFootTimeStep( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, IncrTime)
{
  pdCoreIncrTime( &core );
  EXPECT_EQ( TIME_STEP, pdCoreTime( &core ) );
  EXPECT_EQ( TIME_STEP, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( TIME_STEP, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( TIME_STEP, pdFootTime( pdCoreRFPtr( &core ) ) );
  pdCoreIncrTime( &core );
  EXPECT_EQ( 2*TIME_STEP, pdCoreTime( &core ) );
  EXPECT_EQ( 2*TIME_STEP, pdCZTime( pdCoreCZPtr( &core ) ) );
  EXPECT_EQ( 2*TIME_STEP, pdFootTime( pdCoreLFPtr( &core ) ) );
  EXPECT_EQ( 2*TIME_STEP, pdFootTime( pdCoreRFPtr( &core ) ) );
}

TEST_F(pdCoreTest, Load)
{
  char filename[] = "model/mighty.zkc";

  pdCoreInit( &core, &cmd, TIME_STEP );
  pdCoreLoad( &core, filename );
  EXPECT_EQ( 25, (int)rkChainNum(pdRobotChainPtr(pdCoreRobotPtr(&core))));
}

TEST_F(pdCoreTest, JointSize)
{
  char model[] = "model/mighty.zkc";

  pdCoreLoad( &core, model );
  EXPECT_EQ( 26, pdCoreJointSize( &core ) );
}

TEST_F(pdCoreTest, JointDis)
{
  char model[] = "model/mighty.zkc";

  pdCoreLoad( &core, model );
  EXPECT_EQ( 26, pdCoreJointSize( &core ) );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 0 ),
               zVecElem( pdCoreJointDis( &core ), 0 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 1 ),
               zVecElem( pdCoreJointDis( &core ), 1 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 2 ),
               zVecElem( pdCoreJointDis( &core ), 2 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 10 ),
               zVecElem( pdCoreJointDis( &core ), 10 ), GTEST_TOL );
  EXPECT_NEAR( zVecElem( pdRobotJointDis( pdCoreRobotPtr( &core ) ), 20 ),
               zVecElem( pdCoreJointDis( &core ), 20 ), GTEST_TOL );
}
