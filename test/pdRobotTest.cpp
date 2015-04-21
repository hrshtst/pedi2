#include "gtest/gtest.h"
#include <pedi2/pd_robot.h>

class pdRobotTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdRobotInit( &robot );
  };
  virtual void TearDown() {};

  pdRobot robot;
};

TEST_F(pdRobotTest, Load)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 25, (int)rkChainNum(&robot.chain) );
  EXPECT_EQ(  0, robot.body_id );
  EXPECT_EQ( 12, robot.lf_id );
  EXPECT_EQ( 24, robot.rf_id );
  EXPECT_EQ( 26, zVecSize(robot.dis) );
}

TEST_F(pdRobotTest, GetJointSize)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 26, pdRobotGetJointSize(&robot) );
}

TEST_F(pdRobotTest, GetJointDis)
{
  char model[] = "model/mighty.zkc";
  zVec dis;

  pdRobotLoad( &robot, model );
  dis = zVecAlloc(pdRobotGetJointSize(&robot));
  zVec3DCreate( &robot.d_com_pos, 0, 0, 0.26 );
  zVec3DCreate( &robot.d_lf_pos, 0.042, 0, 0 );
  zVec3DCreate( &robot.d_rf_pos, -0.042, 0, 0 );
  pdRobotSolveIK( &robot );
  // zVecWrite( robot.dis );
  pdRobotGetJointDis( &robot, dis );
  // zVecWrite( dis );
  EXPECT_NEAR( -0.03515806383  , zVecElem(dis,0), 1e-4);
  EXPECT_NEAR( -0.0008198812396, zVecElem(dis,1), 1e-4);
  EXPECT_NEAR( 0.3210756019    , zVecElem(dis,2), 1e-4);
  EXPECT_NEAR( 0               , zVecElem(dis,3), 1e-4);
  EXPECT_NEAR( 0               , zVecElem(dis,4), 1e-4);
  EXPECT_NEAR( 0               , zVecElem(dis,5), 1e-4);
  EXPECT_NEAR( -0.3490658504   , zVecElem(dis,6), 1e-4);
  EXPECT_NEAR( 0.2617993878    , zVecElem(dis,7), 1e-4);
  EXPECT_NEAR( 1.221730476     , zVecElem(dis,8), 1e-4);
  EXPECT_NEAR( 1.047197551     , zVecElem(dis,9), 1e-4);
  EXPECT_NEAR( 0.6620286884    , zVecElem(dis,10), 1e-4);
  EXPECT_NEAR( -0.153427404    , zVecElem(dis,11), 1e-4);
  EXPECT_NEAR( 0.1185482202    , zVecElem(dis,12), 1e-4);
  EXPECT_NEAR( 1.019299067     , zVecElem(dis,13), 1e-4);
  EXPECT_NEAR( 0.3663931564    , zVecElem(dis,14), 1e-4);
  EXPECT_NEAR( 0.1936055216    , zVecElem(dis,15), 1e-4);
  EXPECT_NEAR( -0.3490658504   , zVecElem(dis,16), 1e-4);
  EXPECT_NEAR( 0.2617993878    , zVecElem(dis,17), 1e-4);
  EXPECT_NEAR( 1.221730476     , zVecElem(dis,18), 1e-4);
  EXPECT_NEAR( 1.047197551     , zVecElem(dis,19), 1e-4);
  EXPECT_NEAR( 0.2509528956    , zVecElem(dis,20), 1e-4);
  EXPECT_NEAR( -0.194208861    , zVecElem(dis,21), 1e-4);
  EXPECT_NEAR( 0.04943415161   , zVecElem(dis,22), 1e-4);
  EXPECT_NEAR( 0.9751175828    , zVecElem(dis,23), 1e-4);
  EXPECT_NEAR( 0.7289810787    , zVecElem(dis,24), 1e-4);
  EXPECT_NEAR( 0.2003247639    , zVecElem(dis,25), 1e-4);
  zVecFree( dis );
}
