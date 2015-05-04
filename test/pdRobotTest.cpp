#include "gtest/gtest.h"
#include <pedi2/pd_robot.h>

const int MIGHTY_BODY_ID = 0;
const int MIGHTY_LH_ID = 5;
const int MIGHTY_LF_ID = 12;
const int MIGHTY_RH_ID = 17;
const int MIGHTY_RF_ID = 24;

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
    robot._num_cell = 100;
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
  EXPECT_EQ( 0, robot._num_cell );
  EXPECT_EQ( NULL, robot._cell );
}

TEST_F(pdRobotTest, Destroy)
{
  SetVacuousPrm();
  pdRobotDestroy( &robot );
  EXPECT_EQ( 0, pdRobotChainPtr( &robot )->mass );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  EXPECT_EQ( 0, robot._num_cell );
  EXPECT_EQ( NULL, robot._cell );
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

TEST_F(pdRobotTest, CellNum)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 10, robot._num_cell );
}

TEST_F(pdRobotTest, CellNum_2)
{
  char model[] = "model/mighty2.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 6, robot._num_cell );
}

#define GTEST_TOL 1e-12
TEST_F(pdRobotTest, SolveIK)
{
  char model[] = "model/mighty.zkc";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D base_att, lf_att, rf_att;
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate(  &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(   &lf_pos, 0.0, 0.042, 0.0 );
  zVec3DCreate(   &rf_pos, 0.0, -0.042, 0.0 );
  zVec3DCreate( &base_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSolveIK( &robot );
  EXPECT_NEAR( 0.0, rkChainWldCOM( pdRobotChainPtr( &robot ) )->e[0], GTEST_TOL );
  EXPECT_NEAR( 0.0, rkChainWldCOM( pdRobotChainPtr( &robot ) )->e[1], GTEST_TOL );
  EXPECT_NEAR( 0.26, rkChainWldCOM( pdRobotChainPtr( &robot ) )->e[2], GTEST_TOL );
  EXPECT_NEAR( 0.0, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LF_ID )->e[0], GTEST_TOL );
  EXPECT_NEAR( 0.042, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LF_ID )->e[1], GTEST_TOL );
  EXPECT_NEAR( 0.0, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LF_ID )->e[2], GTEST_TOL );
  EXPECT_NEAR( 0.0, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RF_ID )->e[0], GTEST_TOL );
  EXPECT_NEAR( -0.042, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RF_ID )->e[1], GTEST_TOL );
  EXPECT_NEAR( 0.0, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RF_ID )->e[2], GTEST_TOL );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_BODY_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_LF_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_RF_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL );
}
