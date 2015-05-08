#include "gtest/gtest.h"
#include <pedi2/pd_robot.h>

const int MIGHTY_BODY_ID = 0;
const int MIGHTY_LH_ID = 5;
const int MIGHTY_LF_ID = 12;
const int MIGHTY_RH_ID = 17;
const int MIGHTY_RF_ID = 24;
#define GTEST_TOL 1e-12
#define GTEST_TOL_LOOSE 1e-04

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
  EXPECT_EQ( -1, robot._base_id );
  EXPECT_EQ( -1, robot._lf_id );
  EXPECT_EQ( -1, robot._rf_id );
  EXPECT_EQ( -1, robot._lh_id );
  EXPECT_EQ( -1, robot._rh_id );
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

TEST_F(pdRobotTest, Load_CheckCell)
{
  char model[] = "model/mighty.zkc";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 10, robot._num_cell );
  cp = zListTail( &pdRobotIKPtr(&robot)->clist ); // 0
  EXPECT_EQ( rkIKJacobiCOM, cp->data._cmat_fp );
  EXPECT_EQ( 0, cp->data.id );
  EXPECT_EQ( 0, cp->data.attr.id );
  cp = zListCellNext( cp );     // 1
  EXPECT_EQ( rkIKJacobiLinkWldAng, cp->data._cmat_fp );
  EXPECT_EQ( 1, cp->data.id );
  EXPECT_EQ( MIGHTY_BODY_ID, cp->data.attr.id );
  cp = zListCellNext( cp );     // 2
  EXPECT_EQ( rkIKJacobiLinkWldLin, cp->data._cmat_fp );
  EXPECT_EQ( 2, cp->data.id );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  cp = zListCellNext( cp );     // 3
  cp = zListCellNext( cp );     // 4
  cp = zListCellNext( cp );     // 5
  cp = zListCellNext( cp );     // 6
  cp = zListCellNext( cp );     // 7
  cp = zListCellNext( cp );     // 8
  EXPECT_EQ( rkIKJacobiLinkWldLin, cp->data._cmat_fp );
  EXPECT_EQ( 8, cp->data.id );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
}

TEST_F(pdRobotTest, DISABLED_Load_CheckCell_2)
{
  char model[] = "model/mighty2.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 6, robot._num_cell );
}

TEST_F(pdRobotTest, Load_AllocCellPtr)
{
  char model[] = "model/mighty.zkc";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  cp = zListTail( &pdRobotIKPtr(&robot)->clist ); // 0
  EXPECT_EQ( cp, robot._cell[0] );
  cp = zListCellNext( cp );     // 1
  EXPECT_EQ( cp, robot._cell[1] );
  cp = zListCellNext( cp );     // 2
  EXPECT_EQ( cp, robot._cell[2] );
  cp = zListCellNext( cp );     // 3
  cp = zListCellNext( cp );     // 4
  cp = zListCellNext( cp );     // 5
  cp = zListCellNext( cp );     // 6
  cp = zListCellNext( cp );     // 7
  cp = zListCellNext( cp );     // 8
  cp = zListCellNext( cp );     // 9
  EXPECT_EQ( cp, robot._cell[9] );
}

TEST_F(pdRobotTest, Load_CheckID)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ(  0, robot._base_id );
  EXPECT_EQ( 12, robot._lf_id );
  EXPECT_EQ( 24, robot._rf_id );
  EXPECT_EQ(  5, robot._lh_id );
  EXPECT_EQ( 17, robot._rh_id );
}

TEST_F(pdRobotTest, Load_InitRefVec)
{
  char model[] = "model/mighty.zkc";
  zVec3D *v;

  pdRobotLoad( &robot, model );
  v = robot._ref_vec;
  // COM pos
  EXPECT_NEAR( 0.02961612673, zVec3DElem( &v[0], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0003066383315, zVec3DElem( &v[0], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.2928640259, zVec3DElem( &v[0], zZ ), GTEST_TOL_LOOSE );
  // base att
  EXPECT_NEAR( 0, zVec3DElem( &v[1], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[1], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[1], zZ ), GTEST_TOL_LOOSE );
  // left foot pos
  EXPECT_NEAR( 0.034, zVec3DElem( &v[2], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.042, zVec3DElem( &v[2], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[2], zZ ), GTEST_TOL_LOOSE );
  // left foot att
  EXPECT_NEAR( 0, zVec3DElem( &v[3], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[3], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[3], zZ ), GTEST_TOL_LOOSE );
  // right foot pos
  EXPECT_NEAR( 0.034, zVec3DElem( &v[4], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.042, zVec3DElem( &v[4], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[4], zZ ), GTEST_TOL_LOOSE );
  // right foot att
  EXPECT_NEAR( 0, zVec3DElem( &v[5], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[5], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[5], zZ ), GTEST_TOL_LOOSE );
  // left hand pos
  EXPECT_NEAR( 0.039, zVec3DElem( &v[6], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.13019468, zVec3DElem( &v[6], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.28642462, zVec3DElem( &v[6], zZ ), GTEST_TOL_LOOSE );
  // left hand att
  EXPECT_NEAR( 0, zVec3DElem( &v[7], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[7], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[7], zZ ), GTEST_TOL_LOOSE );
  // right hand pos
  EXPECT_NEAR( 0.039, zVec3DElem( &v[8], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.13019468, zVec3DElem( &v[8], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.28642462, zVec3DElem( &v[8], zZ ), GTEST_TOL_LOOSE );
  // right hand att
  EXPECT_NEAR( 0, zVec3DElem( &v[9], zX ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[9], zY ), GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0, zVec3DElem( &v[9], zZ ), GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, Load_FileError)
{
  rkIKCreate( pdRobotIKPtr( &robot ), pdRobotChainPtr( &robot ) );
  EXPECT_FALSE( pdRobotLoad( &robot, "hoge.zkc" ) );
  destroy_flag = true;
}

TEST_F(pdRobotTest, Load_NotEnoughConstraintsError)
{
  EXPECT_FALSE( pdRobotLoad( &robot, "model/mighty2.zkc" ) );
  destroy_flag = true;
}

TEST_F(pdRobotTest, Load_LinkIDMismatchError)
{
  EXPECT_FALSE( pdRobotLoad( &robot, "model/mighty3.zkc" ) );
  destroy_flag = true;
}

TEST_F(pdRobotTest, Load_NotImplementedError)
{
  EXPECT_FALSE( pdRobotLoad( &robot, "model/mighty4.zkc" ) );
  destroy_flag = true;
}

TEST_F(pdRobotTest, SetRefVec_Error)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, 0, 0.26 );
  pdRobotSetRefVec( &robot, &v, 10 );
  SUCCEED();
}

TEST_F(pdRobotTest, SetRefCOM)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, 0, 0.26 );
  pdRobotSetRefCOM( &robot, &v );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[0], zX ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[0], zY ) );
  EXPECT_DOUBLE_EQ( 0.26, zVec3DElem( &robot._ref_vec[0], zZ ) );
}

TEST_F(pdRobotTest, SetRefBaseAtt)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0.1*zPI_2, 0, 0 );
  pdRobotSetRefBaseAtt( &robot, &v );
  EXPECT_DOUBLE_EQ( 0.1*zPI_2, zVec3DElem( &robot._ref_vec[1], zX ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[1], zY ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[1], zZ ) );
}

TEST_F(pdRobotTest, SetRefLFPos)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, 0.042, 0.0 );
  pdRobotSetRefLFPos( &robot, &v );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[2], zX ) );
  EXPECT_DOUBLE_EQ( 0.042, zVec3DElem( &robot._ref_vec[2], zY ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[2], zZ ) );
}

TEST_F(pdRobotTest, SetRefLFAtt)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0.2*zPI_2, 0.0, 0.0 );
  pdRobotSetRefLFAtt( &robot, &v );
  EXPECT_DOUBLE_EQ( 0.2*zPI_2, zVec3DElem( &robot._ref_vec[3], zX ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[3], zY ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[3], zZ ) );
}

TEST_F(pdRobotTest, SetRefRFPos)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, -0.042, 0.0 );
  pdRobotSetRefRFPos( &robot, &v );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[4], zX ) );
  EXPECT_DOUBLE_EQ( -0.042, zVec3DElem( &robot._ref_vec[4], zY ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[4], zZ ) );
}

TEST_F(pdRobotTest, SetRefRFAtt)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0.3*zPI_2, 0.0, 0.0 );
  pdRobotSetRefRFAtt( &robot, &v );
  EXPECT_DOUBLE_EQ( 0.3*zPI_2, zVec3DElem( &robot._ref_vec[5], zX ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[5], zY ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[5], zZ ) );
}

TEST_F(pdRobotTest, SetRefLHPos)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, 0.05, 0.27 );
  pdRobotSetRefLHPos( &robot, &v );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[6], zX ) );
  EXPECT_DOUBLE_EQ( 0.05, zVec3DElem( &robot._ref_vec[6], zY ) );
  EXPECT_DOUBLE_EQ( 0.27, zVec3DElem( &robot._ref_vec[6], zZ ) );
}

TEST_F(pdRobotTest, SetRefLHAtt)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0.4*zPI_2, 0.0, 0.0 );
  pdRobotSetRefLHAtt( &robot, &v );
  EXPECT_DOUBLE_EQ( 0.4*zPI_2, zVec3DElem( &robot._ref_vec[7], zX ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[7], zY ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[7], zZ ) );
}

TEST_F(pdRobotTest, SetRefRHPos)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, -0.05, 0.27 );
  pdRobotSetRefRHPos( &robot, &v );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[8], zX ) );
  EXPECT_DOUBLE_EQ( -0.05, zVec3DElem( &robot._ref_vec[8], zY ) );
  EXPECT_DOUBLE_EQ( 0.27, zVec3DElem( &robot._ref_vec[8], zZ ) );
}

TEST_F(pdRobotTest, SetRefRHAtt)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0.5*zPI_2, 0.0, 0.0 );
  pdRobotSetRefRHAtt( &robot, &v );
  EXPECT_DOUBLE_EQ( 0.5*zPI_2, zVec3DElem( &robot._ref_vec[9], zX ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[9], zY ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DElem( &robot._ref_vec[9], zZ ) );
}

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
