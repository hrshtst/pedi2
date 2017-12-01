#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_robot.h>

const int MIGHTY_BODY_ID = 0;
const int MIGHTY_LH_ID = 5;
const int MIGHTY_LF_ID = 12;
const int MIGHTY_RH_ID = 17;
const int MIGHTY_RF_ID = 24;
const double DT = 0.01;
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
    pdRobotCellNum( &robot ) = 100;
  };

  void SetRandomValues() {
    pdRobotChainPtr( &robot )->mass = ri.rand();
    rkChainReadFile( &robot._chain, (char *)"model/dummy.zkc" );
    rkIKCreate( &robot._ik, &robot._chain );
    pdRobotCellNum( &robot ) = (int)ri.rand();
  };

  void LoadAndSolveIK() {
    char model[] = "model/mighty.zkc";
    zVec3D com_pos, lf_pos, rf_pos, lh_pos, rh_pos;
    zVec3D base_att, lf_att, rf_att, lh_att, rh_att;

    pdRobotLoad( &robot, model );
    zVec3DCreate(  &com_pos, 0.0, 0.0, 0.26 );
    zVec3DCreate( &base_att, 0.0, 0.0, 0.0 );
    zVec3DCreate(   &lf_pos, 0.0, 0.042, 0.0 );
    zVec3DCreate(   &lf_att, 0.0, 0.0, 0.0 );
    zVec3DCreate(   &rf_pos, 0.0, -0.042, 0.0 );
    zVec3DCreate(   &rf_att, 0.0, 0.0, 0.0 );
    zVec3DCreate(   &lh_pos, 0.0, 0.13, 0.25 );
    zVec3DCreate(   &lh_att, 0.0, 0.0, 0.0 );
    zVec3DCreate(   &rh_pos, 0.0, -0.13, 0.25 );
    zVec3DCreate(   &rh_att, 0.0, 0.0, 0.0 );
    pdRobotSetRefCOM( &robot, &com_pos );
    pdRobotSetRefBaseAtt( &robot, &base_att );
    pdRobotSetRefLFPos( &robot, &lf_pos );
    pdRobotSetRefLFAtt( &robot, &lf_att );
    pdRobotSetRefRFPos( &robot, &rf_pos );
    pdRobotSetRefRFAtt( &robot, &rf_att );
    pdRobotSetRefLHPos( &robot, &lh_pos );
    pdRobotSetRefRHPos( &robot, &rh_pos );
    // pdRobotSetRefLHAtt( &robot, &lh_att );
    // pdRobotSetRefRHAtt( &robot, &rh_att );
    pdRobotSolveIK( &robot, 0 );
  };

  void SetRandomState() {
    ri.SetRandVec3D( state.com_pos );
    ri.SetRandVec3D( state.base_att );
    ri.SetRandVec3D( state.lf_pos );
    ri.SetRandVec3D( state.lf_att );
    ri.SetRandVec3D( state.rf_pos );
    ri.SetRandVec3D( state.rf_att );
    ri.SetRandVec3D( state.lh_pos );
    ri.SetRandVec3D( state.lh_att );
    ri.SetRandVec3D( state.rh_pos );
    ri.SetRandVec3D( state.rh_att );
  };

  bool destroy_flag;
  pdRobot robot;
  pdState state;
  pdBiped biped;
  pdCmd cmd;
  RandomInitializer ri;
};

TEST_F(pdRobotTest, Init)
{
  // SetVacuousPrm();
  SetRandomValues();
  pdRobotInit( &robot );
  EXPECT_EQ( 0, pdRobotChainPtr( &robot )->mass );
  EXPECT_EQ( pdRobotChainPtr( &robot ), pdRobotIKPtr( &robot )->chain );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  EXPECT_EQ( 0, zVecSize( pdRobotJointDis( &robot ) ) );
  EXPECT_EQ( NULL, pdRobotJointVel( &robot ) );
  EXPECT_EQ( 0, zVecSize( pdRobotJointVel( &robot ) ) );
  EXPECT_EQ( 0, pdRobotCellNum( &robot ) );
  EXPECT_EQ( NULL, robot._cell );
  EXPECT_EQ( NULL, robot._ref_vec );
  EXPECT_EQ( NULL, robot._ref_set_flag );
  EXPECT_EQ( -1, pdRobotBaseID( &robot ) );
  EXPECT_EQ( -1, pdRobotLFID( &robot ) );
  EXPECT_EQ( -1, pdRobotRFID( &robot ) );
  EXPECT_EQ( -1, pdRobotLHID( &robot ) );
  EXPECT_EQ( -1, pdRobotRHID( &robot ) );
  EXPECT_EQ( NULL, robot._sr_lf_vert );
  EXPECT_EQ( NULL, robot._sr_rf_vert );
  EXPECT_EQ( NULL, robot._sr_vert );
}

TEST_F(pdRobotTest, Destroy)
{
  // SetVacuousPrm();
  SetRandomValues();
  pdRobotDestroy( &robot );
  EXPECT_EQ( 0, pdRobotChainPtr( &robot )->mass );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  EXPECT_EQ( NULL, pdRobotJointVel( &robot ) );
  EXPECT_EQ( 0, pdRobotCellNum( &robot ) );
  EXPECT_EQ( NULL, robot._cell );
  EXPECT_EQ( NULL, robot._ref_vec );
  EXPECT_EQ( NULL, robot._ref_set_flag );
  EXPECT_EQ( NULL, robot._sr_lf_vert );
  EXPECT_EQ( NULL, robot._sr_rf_vert );
  EXPECT_EQ( NULL, robot._sr_vert );
  destroy_flag = true;
}

TEST_F(pdRobotTest, Load)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 25, (int)rkChainNum( pdRobotChainPtr( &robot ) ) );
}

TEST_F(pdRobotTest, LinkSetJointDis)
{
  char model[] = "model/mighty.zkc";
  zVec dis;
  double d[1];

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );

  d[0] = 0.02;
  pdRobotLinkSetJointDis( &robot, 1, d );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_EQ( 0.02, zVecElem( dis, 6 ) );
  d[0] = 0.04;
  pdRobotLinkSetJointDis( &robot, 3, d );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_EQ( 0.04, zVecElem( dis, 8 ) );
  zVecFree( dis );
}

TEST_F(pdRobotTest, SetJointDis)
{
  char model[] = "model/mighty.zkc";
  zVec dis, setdis;
  zIndex index;

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );
  setdis = zVecCreateList( 4, 0.01, 0.02, 0.03, 0.04 );
  index  = zIndexCreateList( 4, 1, 2, 3, 4 );

  pdRobotSetJointDis( &robot, index, setdis );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_EQ( 0.01, zVecElem( dis, 6 ) );
  EXPECT_EQ( 0.02, zVecElem( dis, 7 ) );
  EXPECT_EQ( 0.03, zVecElem( dis, 8 ) );
  EXPECT_EQ( 0.04, zVecElem( dis, 9 ) );

  zIndexFree( index );
  zVecFree( setdis );
  zVecFree( dis );
}

TEST_F(pdRobotTest, FK)
{
  char model[] = "model/mighty.zkc";
  zVec dis;

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );
  zVecClear( dis );
  zVecSetElem( dis,  7, 0.01 );
  zVecSetElem( dis,  8, 0.02 );
  zVecSetElem( dis,  9, 0.03 );
  zVecSetElem( dis, 10, 0.04 );
  zVecSetElem( dis, 11, 0.05 );

  pdRobotFK( &robot, dis );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_DOUBLE_EQ( 0.01, zVecElem( dis, 7 ) );
  EXPECT_DOUBLE_EQ( 0.02, zVecElem( dis, 8 ) );
  EXPECT_DOUBLE_EQ( 0.03, zVecElem( dis, 9 ) );
  EXPECT_DOUBLE_EQ( 0.04, zVecElem( dis, 10 ) );
  EXPECT_DOUBLE_EQ( 0.05, zVecElem( dis, 11 ) );
  zVecFree( dis );
}

TEST_F(pdRobotTest, FK_CheckJointVel)
{
  zVec dis, vel;

  LoadAndSolveIK();
  dis = zVecAlloc( 26 );
  vel = zVecAlloc( 26 );
  zVecClear( dis );
  zVecSetElem( dis,  7, 0.01 );
  zVecSetElem( dis,  8, 0.02 );
  zVecSetElem( dis,  9, 0.03 );
  zVecSetElem( dis, 10, 0.04 );
  zVecSetElem( dis, 11, 0.05 );

  pdRobotFK( &robot, dis );
  pdRobotGetJointDisAll( &robot, dis );
  pdRobotGetJointVelAll( &robot, vel );
  for(int i=0; i<26; i++){
    ASSERT_DOUBLE_EQ( 0.0, zVecElem( vel, i ) );
  }
  zVecFree( dis );
  zVecFree( vel );
}

TEST_F(pdRobotTest, FK_LargeBodyOffset)
{
  char model[] = "model/mighty.zkc";
  zVec dis;

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );
  zVecClear( dis );
  zVecSetElem( dis, 0, 10 );
  zVecSetElem( dis, 1, 10 );
  zVecSetElem( dis, 2, 10 );
  zVecSetElem( dis, 3, 0 );
  zVecSetElem( dis, 4, 0 );
  zVecSetElem( dis, 5, 0 );

  pdRobotFK( &robot, dis );
  double lf, rf;
  lf = zVec3DElem( rkChainLinkWldPos(pdRobotChainPtr(&robot), pdRobotLFID(&robot)), zZ );
  rf = zVec3DElem( rkChainLinkWldPos(pdRobotChainPtr(&robot), pdRobotRFID(&robot)), zZ );
  EXPECT_NEAR( 0, lf, 1e-06 );
  EXPECT_NEAR( 0, rf, 1e-06 );
  zVecFree( dis );
}

TEST_F(pdRobotTest, FKIndex)
{
  char model[] = "model/mighty.zkc";
  zVec dis;
  zIndex index;
  zVec3D com;

  pdRobotLoad( &robot, model );
  dis = zVecCreateList( 2, 1.57, 1.57 );
  index  = zIndexCreateList( 2, 7, 19 );
  pdRobotFKIndex( &robot, index, dis );
  pdRobotCOMPos( &robot, &com );
  EXPECT_EQ( 1.57, zVecElem( pdRobotJointDis(&robot), 7 ) );
  EXPECT_EQ( 1.57, zVecElem( pdRobotJointDis(&robot), 19 ) );
  EXPECT_NEAR( 0.2996089431, zVec3DElem( &com, zZ ), 1e-06 );
}

TEST_F(pdRobotTest, ResetJointDis)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  pdRobotResetJointDis( &robot );
  for(int i=0; i<26; i++){
    if( i==2 )
      EXPECT_NE( 0, zVecElem( pdRobotJointDis(&robot), i ) );
    else
      EXPECT_EQ( 0, zVecElem( pdRobotJointDis(&robot), i ) );
  }
}

TEST_F(pdRobotTest, ResetPose)
{
  char model[] = "model/mighty.zkc";
  zVec dis;

  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );
  pdRobotLoad( &robot, model );
  pdRobotDefaultBipedInit( &robot, &biped, &state );
  dis = zVecAlloc( pdRobotJointSize( &robot ) );
  pdRobotGetJointDisAll( &robot, dis );
  pdRobotUpdateState( &robot, &state );

  // method to testify
  zVecSetElem( dis, 0, -1 );
  zVecSetElem( dis, 1, 1 );
  zVecSetElem( dis, 2, 0.34 );
  zVecSetElem( dis, 3, 0 );
  zVecSetElem( dis, 4, 0 );
  zVecSetElem( dis, 5, 1.57 );
  pdRobotResetPose( &robot, &biped, &state, dis );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_DOUBLE_EQ( -1, zVecElem(dis,0) );
  EXPECT_DOUBLE_EQ( 1,  zVecElem(dis,1) );
  EXPECT_NEAR( 0.347, zVecElem(dis,2), 1e03 );
  EXPECT_DOUBLE_EQ( 0, zVecElem(dis,3) );
  EXPECT_DOUBLE_EQ( 0, zVecElem(dis,4) );
  EXPECT_DOUBLE_EQ( 1.57, zVecElem(dis,5) );
  zVecFree( dis );
}

TEST_F(pdRobotTest, Load_UnsetAllFlag)
{
  char model[] = "model/mighty.zkc";
  int i;

  pdRobotLoad( &robot, model );
  for( i=0; i<pdRobotCellNum( &robot ); i++ )
    robot._ref_set_flag[i] = true;
  pdRobotUnsetAllFlags( &robot );
  EXPECT_FALSE( robot._ref_set_flag[0] );
  EXPECT_FALSE( robot._ref_set_flag[1] );
  EXPECT_FALSE( robot._ref_set_flag[2] );
  EXPECT_FALSE( robot._ref_set_flag[3] );
  EXPECT_FALSE( robot._ref_set_flag[4] );
  EXPECT_FALSE( robot._ref_set_flag[5] );
  EXPECT_FALSE( robot._ref_set_flag[6] );
  EXPECT_FALSE( robot._ref_set_flag[7] );
  EXPECT_FALSE( robot._ref_set_flag[8] );
  EXPECT_FALSE( robot._ref_set_flag[9] );
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
  EXPECT_EQ( 26, zVecSize( pdRobotJointDis( &robot ) ) );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 0 ), GTEST_TOL );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 1 ), GTEST_TOL );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 2 ), GTEST_TOL );
}

TEST_F(pdRobotTest, JointVel)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  ASSERT_EQ( 26, zVecSize( pdRobotJointVel( &robot ) ) );
  for(int i=0; i<26; i++)
    EXPECT_DOUBLE_EQ( 0.0, zVecElem( pdRobotJointVel( &robot ), i ) );
}

TEST_F(pdRobotTest, Load_CheckCell)
{
  char model[] = "model/mighty.zkc";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 10, pdRobotCellNum( &robot ) );
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
  EXPECT_EQ(  0, pdRobotBaseID( &robot ) );
  EXPECT_EQ( 12, pdRobotLFID( &robot ) );
  EXPECT_EQ( 24, pdRobotRFID( &robot ) );
  EXPECT_EQ(  5, pdRobotLHID( &robot ) );
  EXPECT_EQ( 17, pdRobotRHID( &robot ) );
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
  zEchoOff();
  rkIKCreate( pdRobotIKPtr( &robot ), pdRobotChainPtr( &robot ) );
  EXPECT_FALSE( pdRobotLoad( &robot, "hoge.zkc" ) );
  destroy_flag = true;
  zEchoOn();
}

TEST_F(pdRobotTest, Load_NotEnoughConstraintsError)
{
  zEchoOff();
  EXPECT_FALSE( pdRobotLoad( &robot, "model/mighty2.zkc" ) );
  destroy_flag = true;
  zEchoOn();
}

TEST_F(pdRobotTest, Load_LinkIDMismatchError)
{
  zEchoOff();
  EXPECT_FALSE( pdRobotLoad( &robot, "model/mighty3.zkc" ) );
  destroy_flag = true;
  zEchoOn();
}

TEST_F(pdRobotTest, Load_NotImplementedError)
{
  zEchoOff();
  EXPECT_FALSE( pdRobotLoad( &robot, "model/mighty4.zkc" ) );
  destroy_flag = true;
  zEchoOn();
}

TEST_F(pdRobotTest, SetRefVec_Error)
{
  char model[] = "model/mighty.zkc";
  zVec3D v;

  zEchoOff();
  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, 0, 0.26 );
  EXPECT_FALSE( pdRobotSetRefVec( &robot, &v, 10 ) );
  zEchoOn();
}

TEST_F(pdRobotTest, SetExtraRefVec)
{
  char model[] = "model/mighty6.zkc";
  zVec3D v;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v, 0, 0, -zPI_2 );
  pdRobotSetExtraRefVec( &robot, &v, 0 );
  EXPECT_EQ( 11, pdRobotCellNum( &robot ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[10], zX ) );
  EXPECT_DOUBLE_EQ( 0, zVec3DElem( &robot._ref_vec[10], zY ) );
  EXPECT_DOUBLE_EQ( -zPI_2, zVec3DElem( &robot._ref_vec[10], zZ ) );
  EXPECT_TRUE( pdRobotExtraFlagIsOn( &robot, 0 ) );
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
  EXPECT_TRUE( pdRobotCOMFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotBaseAttFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotLFPosFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotLFAttFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotRFPosFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotRFAttFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotLHPosFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotLHAttFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotRHPosFlagIsOn( &robot ) );
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
  EXPECT_TRUE( pdRobotRHAttFlagIsOn( &robot ) );
}

TEST_F(pdRobotTest, SetBipedRefVec)
{
  char model[] = "model/mighty.zkc";

  pdRobotLoad( &robot, model );
  // set random values to testified vectors
  for(int i=0; i<PD_ROBOT_REQUIRED_CONST_NUM; i++)
    ri.SetRandVec3D( pdRobotRefVec( &robot, i ) );
  // set random values to referential vectors
  ri.SetRandVec3D( pdBipedRefCOMPos(&biped) );
  ri.SetRandVec3D( pdBipedRefBaseAtt(&biped) );
  ri.SetRandVec3D( pdBipedRefLFPos(&biped) );
  ri.SetRandVec3D( pdBipedRefLFAtt(&biped) );
  ri.SetRandVec3D( pdBipedRefRFPos(&biped) );
  ri.SetRandVec3D( pdBipedRefRFAtt(&biped) );

  // call method to testify
  pdRobotSetBipedRefVec( &robot, &biped );

  zVec3D *alias;
  alias = pdRobotRefCOM( &robot );
  EXPECT_DOUBLE_EQ( pdBipedRefCOMPosX(&biped), zVec3DElem(alias,zX) );
  EXPECT_DOUBLE_EQ( pdBipedRefCOMPosY(&biped), zVec3DElem(alias,zY) );
  EXPECT_DOUBLE_EQ( pdBipedRefCOMPosZ(&biped), zVec3DElem(alias,zZ) );
  alias = pdRobotRefBaseAtt( &robot );
  EXPECT_DOUBLE_EQ( pdBipedRefBaseAttX(&biped), zVec3DElem(alias,zX) );
  EXPECT_DOUBLE_EQ( pdBipedRefBaseAttY(&biped), zVec3DElem(alias,zY) );
  EXPECT_DOUBLE_EQ( pdBipedRefBaseAttZ(&biped), zVec3DElem(alias,zZ) );
  alias = pdRobotRefLFPos( &robot );
  EXPECT_DOUBLE_EQ( pdBipedRefLFPosX(&biped), zVec3DElem(alias,zX) );
  EXPECT_DOUBLE_EQ( pdBipedRefLFPosY(&biped), zVec3DElem(alias,zY) );
  EXPECT_DOUBLE_EQ( pdBipedRefLFPosZ(&biped), zVec3DElem(alias,zZ) );
  alias = pdRobotRefLFAtt( &robot );
  EXPECT_DOUBLE_EQ( pdBipedRefLFAttX(&biped), zVec3DElem(alias,zX) );
  EXPECT_DOUBLE_EQ( pdBipedRefLFAttY(&biped), zVec3DElem(alias,zY) );
  EXPECT_DOUBLE_EQ( pdBipedRefLFAttZ(&biped), zVec3DElem(alias,zZ) );
  alias = pdRobotRefRFPos( &robot );
  EXPECT_DOUBLE_EQ( pdBipedRefRFPosX(&biped), zVec3DElem(alias,zX) );
  EXPECT_DOUBLE_EQ( pdBipedRefRFPosY(&biped), zVec3DElem(alias,zY) );
  EXPECT_DOUBLE_EQ( pdBipedRefRFPosZ(&biped), zVec3DElem(alias,zZ) );
  alias = pdRobotRefRFAtt( &robot );
  EXPECT_DOUBLE_EQ( pdBipedRefRFAttX(&biped), zVec3DElem(alias,zX) );
  EXPECT_DOUBLE_EQ( pdBipedRefRFAttY(&biped), zVec3DElem(alias,zY) );
  EXPECT_DOUBLE_EQ( pdBipedRefRFAttZ(&biped), zVec3DElem(alias,zZ) );
}

TEST_F(pdRobotTest, SolveIK)
{
  zVec3D v;

  LoadAndSolveIK();
  EXPECT_NEAR( 0.0,    rkChainWldCOM( pdRobotChainPtr( &robot ) )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainWldCOM( pdRobotChainPtr( &robot ) )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.26,   rkChainWldCOM( pdRobotChainPtr( &robot ) )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LF_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.042,  rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LF_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LF_ID )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RF_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.042, rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RF_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RF_ID )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LH_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.13,   rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LH_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.25,   rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_LH_ID )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RH_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.13,  rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RH_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.25,   rkChainLinkWldPos( pdRobotChainPtr( &robot ), MIGHTY_RH_ID )->e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_BODY_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_LF_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_RF_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  // zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_LH_ID ), &v);
  // EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  // zMat3DToZYX( rkChainLinkWldAtt( pdRobotChainPtr( &robot ), MIGHTY_RH_ID ), &v);
  // EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, SolveIKCheckJointVel)
{
  zVec vel;

  LoadAndSolveIK();
  vel = zVecAlloc(26);
  pdRobotGetJointVelAll( &robot, vel );
  for(int i=0; i<26; i++){
    if( fabs(zVecElem(vel,i)) > GTEST_TOL ){
      SUCCEED();
      return;
    }
  }
  FAIL();
}

TEST_F(pdRobotTest, AllFlagsAreFlaseAfterSolveIK)
{
  LoadAndSolveIK();
  EXPECT_FALSE( pdRobotCOMFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotBaseAttFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotLFPosFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotLFAttFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotRFPosFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotRFAttFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotLHPosFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotLHAttFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotRHPosFlagIsOn( &robot ) );
  EXPECT_FALSE( pdRobotRHAttFlagIsOn( &robot ) );
}

TEST_F(pdRobotTest, COMPos)
{
  zVec3D com;

  LoadAndSolveIK();
  zVec3DCreate( &com, 1, 2, 3 );
  pdRobotCOMPos( &robot, &com );
  EXPECT_NEAR( 0.0,  com.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,  com.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.26, com.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, BaseAtt)
{
  zVec3D att;

  LoadAndSolveIK();
  zVec3DCreate( &att, 1, 2, 3 );
  pdRobotBaseAtt( &robot, &att );
  EXPECT_NEAR( 0.0,  att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,  att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, FootPos)
{
  zVec3D lf, rf;

  LoadAndSolveIK();
  zVec3DCreate( &lf, 1, 2, 3 );
  zVec3DCreate( &rf, 4, 5, 6 );
  pdRobotFootPos( &robot, &lf, &rf );
  EXPECT_NEAR( 0.0,    lf.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.042,  lf.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    lf.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rf.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.042, rf.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rf.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, FootAtt)
{
  zVec3D lf, rf;

  LoadAndSolveIK();
  zVec3DCreate( &lf, 1, 2, 3 );
  zVec3DCreate( &rf, 4, 5, 6 );
  pdRobotFootAtt( &robot, &lf, &rf );
  EXPECT_NEAR( 0.0, lf.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, HandPos)
{
  zVec3D lh, rh;

  LoadAndSolveIK();
  zVec3DCreate( &lh, 1, 2, 3 );
  zVec3DCreate( &rh, 4, 5, 6 );
  pdRobotHandPos( &robot, &lh, &rh );
  EXPECT_NEAR(  0.0,  lh.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR(  0.13, lh.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR(  0.25, lh.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR(  0.0,  rh.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.13, rh.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR(  0.25, rh.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, HandAtt)
{
  zVec3D lh, rh;

  LoadAndSolveIK();
  zVec3DCreate( &lh, 1, 2, 3 );
  zVec3DCreate( &rh, 4, 5, 6 );
  pdRobotHandAtt( &robot, &lh, &rh );
  // EXPECT_NEAR( 0.0, lh.e[0], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, lh.e[1], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, lh.e[2], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, rh.e[0], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, rh.e[1], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0, rh.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, SupportRegion_Double)
{
  char model[] = "model/mighty.zkc";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D base_att, lf_att, rf_att;
  zVec3DList sr_lf, sr_rf, sr;
  zVec3DListCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(  &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(   &lf_pos, 0.0, 0.042, 0.0 );
  zVec3DCreate(   &rf_pos, 0.0, -0.042, 0.0 );
  zVec3DCreate( &base_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefBaseAtt( &robot, &base_att );
  pdRobotSetRefLFAtt( &robot, &lf_att );
  pdRobotSetRefRFAtt( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );

  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );

  // sr_lf
  cp = zListTail( &sr_lf );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );

  // sr_rf
  cp = zListTail( &sr_rf );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );

  // sr
  cp = zListTail( &sr );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, SupportRegion_Single_Left)
{
  char model[] = "model/mighty.zkc";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D base_att, lf_att, rf_att;
  zVec3DList sr_lf, sr_rf, sr;
  zVec3DListCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(  &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(   &lf_pos, 0.0, 0.042, 0.0 );
  zVec3DCreate(   &rf_pos, 0.0, -0.042, 0.01 );
  zVec3DCreate( &base_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefBaseAtt( &robot, &base_att );
  pdRobotSetRefLFAtt( &robot, &lf_att );
  pdRobotSetRefRFAtt( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );

  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );

  // sr_lf
  cp = zListTail( &sr_lf );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );

  // sr_rf
  EXPECT_EQ( 0, zListNum( &sr_rf ) );

  // sr
  cp = zListTail( &sr );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, SupportRegion_Single_Right)
{
  char model[] = "model/mighty.zkc";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D base_att, lf_att, rf_att;
  zVec3DList sr_lf, sr_rf, sr;
  zVec3DListCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(  &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(   &lf_pos, 0.0, 0.042, 0.01 );
  zVec3DCreate(   &rf_pos, 0.0, -0.042, 0.0 );
  zVec3DCreate( &base_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(   &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefBaseAtt( &robot, &base_att );
  pdRobotSetRefLFAtt( &robot, &lf_att );
  pdRobotSetRefRFAtt( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );

  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );

  // sr_lf
  EXPECT_EQ( 0, zListNum( &sr_lf ) );

  // sr_rf
  cp = zListTail( &sr_rf );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );

  // sr
  cp = zListTail( &sr_rf );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( 0.0564, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.006, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
  cp = zListCellNext( cp );
  EXPECT_NEAR( -0.0426, cp->data->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.078, cp->data->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, cp->data->e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, UpdateState)
{
  LoadAndSolveIK();
  pdStateInit( &state );

  SetRandomState();
  pdRobotUpdateState( &robot, &state );
  EXPECT_NEAR( 0.0,   state.com_pos.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,   state.com_pos.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.26,  state.com_pos.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,  state.base_att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,  state.base_att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,  state.base_att.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.lf_pos.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.042,  state.lf_pos.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.lf_pos.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.rf_pos.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.042, state.rf_pos.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.rf_pos.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.lf_att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.lf_att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.lf_att.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.rf_att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.rf_att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.rf_att.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.lh_pos.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.13,   state.lh_pos.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.25,   state.lh_pos.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    state.rh_pos.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.13,  state.rh_pos.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.25,   state.rh_pos.e[2], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0,    state.lh_att.e[0], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0,    state.lh_att.e[1], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0,    state.lh_att.e[2], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0,    state.rh_att.e[0], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0,    state.rh_att.e[1], GTEST_TOL_LOOSE );
  // EXPECT_NEAR( 0.0,    state.rh_att.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, JointReg)
{
  char model[] = "model/mighty5.zkc";
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKPtr( &robot );

  const int CHECK_ID = 1;        // left_shoulder_flexion
  EXPECT_FALSE( ik->joint_sw[CHECK_ID] );
  EXPECT_EQ( 0, ik->joint_weight[CHECK_ID] );

  // method to testify
  pdRobotJointReg( &robot, CHECK_ID, 0.01 );
  EXPECT_TRUE( ik->joint_sw[CHECK_ID] );
  EXPECT_EQ( 0.01, ik->joint_weight[CHECK_ID] );
}

TEST_F(pdRobotTest, JointRegAll)
{
  char model[] = "model/mighty5.zkc";
  rkChain *c;
  rkIK *ik;

  pdRobotLoad( &robot, model );
  c = pdRobotChainPtr( &robot );
  ik = pdRobotIKPtr( &robot );

  // method to testify
  pdRobotJointRegAll( &robot, 0.02 );
  for( uint i=0; i<pdRobotLinkNum(&robot); i++ )
    if( rkChainLinkJointType(c,i) != RK_JOINT_FIXED ){
      EXPECT_TRUE( ik->joint_sw[i] );
      EXPECT_EQ( 0.02, ik->joint_weight[i] );
    }
}

TEST_F(pdRobotTest, JointUnreg)
{
  char model[] = "model/mighty5.zkc";
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKPtr( &robot );

  const int CHECK_ID = 1;        // left_shoulder_flexion
  pdRobotJointReg( &robot, CHECK_ID, 0.01 );
  EXPECT_TRUE( ik->joint_sw[CHECK_ID] );
  EXPECT_LT( 0.0, ik->joint_weight[CHECK_ID] );

  // method to testify
  pdRobotJointUnreg( &robot, CHECK_ID );
  EXPECT_FALSE( ik->joint_sw[CHECK_ID] );
  EXPECT_EQ( 0, ik->joint_weight[CHECK_ID] );
}

TEST_F(pdRobotTest, JointRegIndex)
{
  char model[] = "model/mighty5.zkc";
  zIndex index;
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKPtr( &robot );
  index = zIndexCreateList( 4, 1, 2, 3, 4 );
  // check
  for( int i=1; i<=4; i++ ){
    EXPECT_FALSE( ik->joint_sw[i] );
    EXPECT_EQ( 0, ik->joint_weight[i] );
  }

  // method to testify
  pdRobotJointRegIndex( &robot, index, 0.02 );
  for( int i=1; i<=4; i++ ){
    EXPECT_TRUE( ik->joint_sw[i] );
    EXPECT_EQ( 0.02, ik->joint_weight[i] );
  }
  zIndexFree( index );
}

TEST_F(pdRobotTest, JointUnregIndex)
{
  char model[] = "model/mighty5.zkc";
  zIndex index;
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKPtr( &robot );
  index = zIndexCreateList( 6, 5, 6, 7, 8, 9, 10 );
  // check
  for( int i=6; i<=10; i++ ){
    EXPECT_TRUE( ik->joint_sw[i] );
    EXPECT_LT( 0, ik->joint_weight[i] );
  }

  // method to testify
  pdRobotJointUnregIndex( &robot, index );
  for( int i=6; i<=10; i++ ){
    EXPECT_FALSE( ik->joint_sw[i] );
    EXPECT_EQ( 0, ik->joint_weight[i] );
  }
  zIndexFree( index );
}

