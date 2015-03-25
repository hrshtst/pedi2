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
  char conf[]  = "model/mighty_ik.conf";

  pdRobotLoad( &robot, model, conf );
  EXPECT_EQ( 25, (int)rkChainNum(&robot.chain) );
  EXPECT_EQ(  0, robot.body_id );
  EXPECT_EQ( 12, robot.lf_id );
  EXPECT_EQ( 24, robot.rf_id );
  EXPECT_EQ( 26, zVecSize(robot.dis) );
}
