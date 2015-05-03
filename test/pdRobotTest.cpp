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
}

TEST_F(pdRobotTest, Destroy)
{
  SetVacuousPrm();
  pdRobotDestroy( &robot );
  EXPECT_EQ( 0, pdRobotChainPtr( &robot )->mass );
}
