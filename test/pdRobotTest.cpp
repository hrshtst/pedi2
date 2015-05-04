#include "gtest/gtest.h"
#include <pedi2/pd_robot.h>

class pdRobotTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdRobotInit( &robot );
    destroy_flag = false;
  };
  virtual void TearDown() {
    if( !destroy_flag )
      pdRobotDestroy( &robot );
  };

  void SetVacuousPrm(){
    pdRobotChainPtr( &robot )->mass = 10000;
    rkChainReadFile( &robot._chain, (char *)"model/dummy.zkc" );
    rkIKCreate( &robot._ik, &robot._chain );
  };

  bool destroy_flag;
  pdRobot robot;
};

TEST_F(pdRobotTest, Init)
{
  SetVacuousPrm();
  pdRobotInit( &robot );
  EXPECT_EQ( 0, pdRobotChainPtr( &robot )->mass );
  EXPECT_EQ( pdRobotChainPtr( &robot ), pdRobotIKPtr( &robot )->chain );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  EXPECT_EQ( 0, zVecSize( pdRobotJointDis( &robot ) ) );
}

TEST_F(pdRobotTest, Destroy)
{
  SetVacuousPrm();
  pdRobotDestroy( &robot );
  EXPECT_EQ( 0, pdRobotChainPtr( &robot )->mass );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  destroy_flag = true;
}

TEST_F(pdRobotTest, Load)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 25, (int)rkChainNum( pdRobotChainPtr( &robot ) ) );
}

TEST_F(pdRobotTest, JointSize)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 26, pdRobotJointSize( &robot ) );
}

TEST_F(pdRobotTest, JointDis)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 26, pdRobotJointSize( &robot ) );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 0 ), GTEST_TOL );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 1 ), GTEST_TOL );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 2 ), GTEST_TOL );
}
