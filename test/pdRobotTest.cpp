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
#define GTEST_TOL_LOOSE 1e-03

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

  void SetRandomValues() {
    rkChainReadZTK( pdRobotChain( &robot ), "model/dummy.ztk" );
  };

  void LoadAndSolveIK() {
    char model[] = "model/mighty.ztk";
    zVec3D com_pos, lf_pos, rf_pos, lh_pos, rh_pos;
    zVec3D torso_att, lf_att, rf_att, lh_att, rh_att;

    pdRobotLoad( &robot, model );
    zVec3DCreate( &com_pos,   0.0, 0.0,    0.26  );
    zVec3DCreate( &torso_att, 0.0, 0.0,    0.0   );
    zVec3DCreate( &lf_pos,    0.0, 0.042,  0.0   );
    zVec3DCreate( &lf_att,    0.0, 0.0,    0.0   );
    zVec3DCreate( &rf_pos,    0.0, -0.042, 0.0   );
    zVec3DCreate( &rf_att,    0.0, 0.0,    0.0   );
    zVec3DCreate( &lh_pos,    0.0, 0.13,   0.25  );
    zVec3DCreate( &lh_att,    1.3, -0.01, -0.373 );
    zVec3DCreate( &rh_pos,    0.0, -0.13,  0.25  );
    zVec3DCreate( &rh_att,   -1.3, -0.005, 0.373 );
    pdRobotSetRefCOM( &robot, &com_pos );
    pdRobotSetRefTorsoZYX( &robot, &torso_att );
    pdRobotSetRefLFPos( &robot, &lf_pos );
    pdRobotSetRefLFZYX( &robot, &lf_att );
    pdRobotSetRefRFPos( &robot, &rf_pos );
    pdRobotSetRefRFZYX( &robot, &rf_att );
    pdRobotSetRefLHPos( &robot, &lh_pos );
    pdRobotSetRefRHPos( &robot, &rh_pos );
    pdRobotSetRefLHZYX( &robot, &lh_att );
    pdRobotSetRefRHZYX( &robot, &rh_att );
    pdRobotSolveIK( &robot, 100 );
  };

  void SetRandomState() {
    ri.SetRandVec3D( state.com_pos );
    ri.SetRandVec3D( state.torso_att );
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
  zVec3DList *list;
  SetRandomValues();
  pdRobotInit( &robot );
  EXPECT_EQ( NULL, rkChainRoot( pdRobotChain( &robot ) ) );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  EXPECT_EQ( 0, zVecSize( pdRobotJointDis( &robot ) ) );
  EXPECT_EQ( NULL, robot.disold );
  EXPECT_EQ( 0, zVecSize( robot.disold ) );
  EXPECT_EQ( -1, pdRobotTorsoID( &robot ) );
  EXPECT_EQ( -1, pdRobotLFID( &robot ) );
  EXPECT_EQ( -1, pdRobotRFID( &robot ) );
  EXPECT_EQ( -1, pdRobotLHID( &robot ) );
  EXPECT_EQ( -1, pdRobotRHID( &robot ) );
  list = pdRobotSRLFVert( &robot )->data.list;
  EXPECT_EQ( 0, zListSize( list ) );
  EXPECT_EQ( zListTail( list ), pdRobotSRLFVert( &robot )->pointer.cp );
  EXPECT_EQ( zListRoot( list ), zListHead( list ) );
  EXPECT_EQ( zListRoot( list ), zListTail( list ) );
  list = pdRobotSRRFVert( &robot )->data.list;
  EXPECT_EQ( 0, zListSize( list ) );
  EXPECT_EQ( zListTail( list ), pdRobotSRRFVert( &robot )->pointer.cp );
  EXPECT_EQ( zListRoot( list ), zListHead( list ) );
  EXPECT_EQ( zListRoot( list ), zListTail( list ) );
  list = pdRobotSRVert( &robot )->data.list;
  EXPECT_EQ( 0, zListSize( list ) );
  EXPECT_EQ( zListTail( list ), pdRobotSRVert( &robot )->pointer.cp );
  EXPECT_EQ( zListRoot( list ), zListHead( list ) );
  EXPECT_EQ( zListRoot( list ), zListTail( list ) );
}

TEST_F(pdRobotTest, Destroy)
{
  SetRandomValues();
  pdRobotDestroy( &robot );
  EXPECT_EQ( NULL, rkChainRoot( pdRobotChain( &robot ) ) );
  EXPECT_EQ( NULL, pdRobotJointDis( &robot ) );
  EXPECT_EQ( NULL, robot.disold );
  EXPECT_EQ( NULL, pdRobotSRLFVert( &robot )->data.list );
  EXPECT_EQ( NULL, pdRobotSRRFVert( &robot )->data.list );
  EXPECT_EQ( NULL, pdRobotSRVert( &robot )->data.list );
  destroy_flag = true;
}

TEST_F(pdRobotTest, Load)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 25, pdRobotLinkNum( &robot ) );
  EXPECT_EQ( 26, pdRobotJointSize( &robot ) );
  EXPECT_EQ( pdRobotJointSize( &robot ), zVecSize( pdRobotJointDis( &robot ) ) );
  EXPECT_EQ( pdRobotJointSize( &robot ), zVecSize( robot.disold ) );
}

TEST_F(pdRobotTest, SetFootIKPriority)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  EXPECT_TRUE( pdRobotSetFootIKPriority( &robot, PD_FOOT_LEFT, 9, 8 ) );
  EXPECT_EQ( 9, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) ) );
  EXPECT_EQ( 8, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
  EXPECT_TRUE( pdRobotSetFootIKPriority( &robot, PD_FOOT_RIGHT, 7, 6 ) );
  EXPECT_EQ( 7, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) ) );
  EXPECT_EQ( 6, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
}

TEST_F(pdRobotTest, PrioritizeSupportFeet)
{
  char model[] = "model/mighty.ztk";
  zLoop3DCell cell[2];
  zLoop3DCell *cp;

  pdRobotLoad( &robot, model );
  pdStateInit( &state );
  /* right foot on the ground, left foot swinging */
  zStackPush( &state.sr_rf, &cell[0] );
  state.lf_pos.c.z = 0.02;
  EXPECT_TRUE( pdRobotPrioritizeSupportFeet( &robot, &state ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SUPPORT_POS, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SUPPORT_ATT, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SWING_POS, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SWING_ATT, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
  /* both feet on the ground */
  zStackPush( &state.sr_lf, &cell[1] );
  state.lf_pos.c.z = 0.0;
  EXPECT_TRUE( pdRobotPrioritizeSupportFeet( &robot, &state ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SUPPORT_POS, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SUPPORT_ATT, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SUPPORT_POS, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) ) );
  EXPECT_EQ( PD_ROBOT_PRIORITY_FOOT_SUPPORT_ATT, rkIKCellPriority( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
  zStackPop( &state.sr_lf, &cp );
  zStackPop( &state.sr_rf, &cp );
}

TEST_F(pdRobotTest, JointDis)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( 26, pdRobotJointSize( &robot ) );
  EXPECT_EQ( 26, zVecSize( pdRobotJointDis( &robot ) ) );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 0 ), GTEST_TOL );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 1 ), GTEST_TOL );
  EXPECT_NEAR( 0.0, zVecElem( pdRobotJointDis( &robot ), 2 ), GTEST_TOL );
}

TEST_F(pdRobotTest, JointDisold)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  ASSERT_EQ( 26, zVecSize( robot.disold ) );
  for(int i=0; i<26; i++)
    EXPECT_DOUBLE_EQ( zVecElem(robot.dis,i), zVecElem(robot.disold,i) );
}

TEST_F(pdRobotTest, Load_CheckConstraints)
{
  char model[] = "model/mighty.ztk";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  ASSERT_EQ( 10, pdRobotIKCellListSize( &robot ) );

  // IK cells are stored in priority order (highest to lowest)
  cp = zListTail( pdRobotIKCellList( &robot ) ); // 0
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LF_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LF_POS, cp->data.priority );
  cp = zListCellNext( cp );     // 2
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RF_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RF_POS, cp->data.priority );

  cp = zListCellNext( cp );     // 1
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LF_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LF_ATT, cp->data.priority );
  cp = zListCellNext( cp );     // 3
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RF_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RF_ATT, cp->data.priority );

  cp = zListCellNext( cp );     // 8
  EXPECT_STREQ( "com", cp->data.constraint->typestr );
  EXPECT_EQ( 0, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_COM, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_COM, cp->data.priority );

  cp = zListCellNext( cp );     // 9
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_BODY_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_TORSO_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_TORSO_ATT, cp->data.priority );

  cp = zListCellNext( cp );     // 4
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LH_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LH_POS, cp->data.priority );
  cp = zListCellNext( cp );     // 6
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RH_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RH_POS, cp->data.priority );
  cp = zListCellNext( cp );     // 5
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LH_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LH_ATT, cp->data.priority );
  cp = zListCellNext( cp );     // 7
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RH_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RH_ATT, cp->data.priority );
}

TEST_F(pdRobotTest, Load_CheckJointWeights_NoHandConstraints)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  ASSERT_EQ( 6, pdRobotIKCellListSize( &robot ) );

  int fixed_joint_id_list[] = { 1, 2, 3, 4, 5, 13, 14, 15, 16, 17, -1 };
  for( const int *idp=fixed_joint_id_list; *idp >= 0; idp++ ){
    int id = *idp;
    EXPECT_FALSE( pdRobotIKSolver(&robot)->joint_is_enabled[id] );
    EXPECT_EQ( 0.0, pdRobotIKSolver(&robot)->joint_weight[id] );
  }
}

TEST_F(pdRobotTest, Load_EnsureIKDisabled)
{
  char model[] = "model/mighty.ztk";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
}

TEST_F(pdRobotTest, Load_CheckID)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( MIGHTY_BODY_ID, pdRobotTorsoID( &robot ) );
  EXPECT_EQ( MIGHTY_LF_ID,   pdRobotLFID( &robot ) );
  EXPECT_EQ( MIGHTY_RF_ID,   pdRobotRFID( &robot ) );
  EXPECT_EQ( MIGHTY_LH_ID,   pdRobotLHID( &robot ) );
  EXPECT_EQ( MIGHTY_RH_ID,   pdRobotRHID( &robot ) );
}

TEST_F(pdRobotTest, Load_CheckID_NoHandConstraints)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";

  pdRobotLoad( &robot, model );
  EXPECT_EQ( MIGHTY_BODY_ID, pdRobotTorsoID( &robot ) );
  EXPECT_EQ( MIGHTY_LF_ID,   pdRobotLFID( &robot ) );
  EXPECT_EQ( MIGHTY_RF_ID,   pdRobotRFID( &robot ) );
  EXPECT_EQ( -1,             pdRobotLHID( &robot ) );
  EXPECT_EQ( -1,             pdRobotRHID( &robot ) );
}

TEST_F(pdRobotTest, Load_FileError)
{
  testing::internal::CaptureStderr();
  EXPECT_FALSE( pdRobotLoad( &robot, "non_exist.ztk" ) );
  std::string msg = testing::internal::GetCapturedStderr();
  std::string expected = \
    "run-time error: cannot open file: non_exist.ztk (zOpenFile).\n"\
    "run-time error: cannot load model file: non_exist.ztk (pdRobotLoad).\n";
  EXPECT_EQ( expected, msg );
}

TEST_F(pdRobotTest, Load_NoIKConfig)
{
  char model[] = "model/mighty_no_ik_config.ztk";
  rkIK *ik;
  rkIKCell *cp;
  int i;

  pdRobotLoad( &robot, model );
  // All joints are registered by default.
  ik = pdRobotIKSolver( &robot );
  for( i=0; i<rkChainLinkNum( pdRobotChain(&robot) ); i++ ){
    if( rkChainLinkJointDOF(pdRobotChain(&robot),i) > 0 ){
      EXPECT_TRUE( ik->joint_is_enabled[i] );
      EXPECT_EQ( 0.001, ik->joint_weight[i] );
    }
  }
  // Constraint on COM is only created at this moment.
  ASSERT_EQ( 1, pdRobotIKCellListSize( &robot ) );
  cp = zListTail( pdRobotIKCellList( &robot ) );
  EXPECT_STREQ( "com", cp->data.constraint->typestr );
  EXPECT_EQ( 0, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_COM, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_COM, cp->data.priority );
  // Ensure IK cells are disabled
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
}

TEST_F(pdRobotTest, BindTorso)
{
  char model[] = "model/mighty_no_ik_config.ztk";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  pdRobotBindTorso( &robot, "body" );

  ASSERT_EQ( 2, pdRobotIKCellListSize( &robot ) );

  cp = zListTail( pdRobotIKCellList( &robot ) );
  EXPECT_STREQ( "com", cp->data.constraint->typestr );
  EXPECT_EQ( 0, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_COM, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_COM, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_BODY_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_TORSO_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_TORSO_ATT, cp->data.priority );

  // Ensure IK cells are disabled
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
  EXPECT_EQ( MIGHTY_BODY_ID, pdRobotTorsoID( &robot ) );
}

TEST_F(pdRobotTest, BindTorso_NotFoundError)
{
  char model[] = "model/mighty_no_ik_config.ztk";

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  testing::internal::CaptureStderr();
  ASSERT_FALSE( pdRobotBindTorso( &robot, "torso" ) );
  std::string msg = testing::internal::GetCapturedStderr();
  std::string expected = \
    "run-time error: torso: unknown link (rkChainFindLink).\n";
  EXPECT_EQ( expected, msg );
}

TEST_F(pdRobotTest, BindFeet)
{
  char model[] = "model/mighty_no_ik_config.ztk";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  pdRobotBindFeet( &robot, "left_foot", "right_foot" );

  ASSERT_EQ( 5, pdRobotIKCellListSize( &robot ) );

  cp = zListTail( pdRobotIKCellList( &robot ) );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LF_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LF_POS, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RF_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RF_POS, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LF_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LF_ATT, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RF_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RF_ATT, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "com", cp->data.constraint->typestr );
  EXPECT_EQ( 0, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_COM, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_COM, cp->data.priority );

  // Ensure IK cells are disabled
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
  EXPECT_EQ( MIGHTY_LF_ID, pdRobotLFID( &robot ) );
  EXPECT_EQ( MIGHTY_RF_ID, pdRobotRFID( &robot ) );
}

TEST_F(pdRobotTest, BindFeet_AllocSR)
{
  char model[] = "model/mighty_no_ik_config.ztk";

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  pdRobotBindFeet( &robot, "left_foot", "right_foot" );
  EXPECT_EQ( 8, zVec3DDataCapacity( pdRobotSRLFVert(&robot) ) );
  EXPECT_EQ( 8, zVec3DDataCapacity( pdRobotSRRFVert(&robot) ) );
  EXPECT_EQ( 16, zVec3DDataCapacity( pdRobotSRVert(&robot) ) );
}

TEST_F(pdRobotTest, BindHands)
{
  char model[] = "model/mighty_no_ik_config.ztk";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  pdRobotBindHands( &robot, "left_hand", "right_hand" );

  ASSERT_EQ( 5, pdRobotIKCellListSize( &robot ) );

  cp = zListTail( pdRobotIKCellList( &robot ) );
  EXPECT_STREQ( "com", cp->data.constraint->typestr );
  EXPECT_EQ( 0, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_COM, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_COM, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LH_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LH_POS, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RH_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RH_POS, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LH_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LH_ATT, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RH_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RH_ATT, cp->data.priority );

  // Ensure IK cells are disabled
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
  EXPECT_EQ( MIGHTY_LH_ID, pdRobotLHID( &robot ) );
  EXPECT_EQ( MIGHTY_RH_ID, pdRobotRHID( &robot ) );
}

TEST_F(pdRobotTest, BindTorsoLimb)
{
  char model[] = "model/mighty_no_ik_config.ztk";
  rkIKCell *cp;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  pdRobotBindTorsoLimb( &robot, "body", "left_foot", "right_foot", "left_hand", "right_hand" );

  ASSERT_EQ( 10, pdRobotIKCellListSize( &robot ) );

  cp = zListTail( pdRobotIKCellList( &robot ) );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LF_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LF_POS, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RF_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RF_POS, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LF_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LF_ATT, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RF_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RF_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RF_ATT, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "com", cp->data.constraint->typestr );
  EXPECT_EQ( 0, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_COM, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_COM, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_BODY_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_TORSO_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_TORSO_ATT, cp->data.priority );

  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LH_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LH_POS, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_pos", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RH_POS, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RH_POS, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_LH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_LH_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_LH_ATT, cp->data.priority );
  cp = zListCellNext( cp );
  EXPECT_STREQ( "world_att", cp->data.constraint->typestr );
  EXPECT_EQ( MIGHTY_RH_ID, cp->data.attr.id );
  EXPECT_STREQ( PD_ROBOT_IKCELL_NAME_RH_ATT, zName(&cp->data) );
  EXPECT_EQ( PD_ROBOT_DEFAULT_PRIORITY_RH_ATT, cp->data.priority );

  // Ensure IK cells are disabled
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
  EXPECT_EQ( MIGHTY_LF_ID, pdRobotLFID( &robot ) );
  EXPECT_EQ( MIGHTY_RF_ID, pdRobotRFID( &robot ) );
  EXPECT_EQ( MIGHTY_LH_ID, pdRobotLHID( &robot ) );
  EXPECT_EQ( MIGHTY_RH_ID, pdRobotRHID( &robot ) );
  EXPECT_EQ( MIGHTY_BODY_ID, pdRobotTorsoID( &robot ) );
}

TEST_F(pdRobotTest, FindIKCellByName)
{
  char model[] = "model/mighty.ztk";
  rkIKCell *expected, *actual;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  expected = zListHead( pdRobotIKCellList( &robot ) ); // rh_att
  actual = pdRobotFindIKCellByName( &robot, "right_hand_att" );
  EXPECT_EQ( expected, actual );
}

TEST_F(pdRobotTest, FindIKCellByName_ReturnNull)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  rkIKCell *actual;

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  actual = pdRobotFindIKCellByName( &robot, "left_hand_pos" );
  EXPECT_EQ( NULL, actual );
}

TEST_F(pdRobotTest, FindLinkIDByIKCellName)
{
  char model[] = "model/mighty.ztk";
  const char *torso_names[] = {
    PD_ROBOT_IKCELL_NAME_TORSO_ATT,
    NULL,
  };
  const char *left_foot_names[] = {
    PD_ROBOT_IKCELL_NAME_LF_POS,
    PD_ROBOT_IKCELL_NAME_LF_ATT,
    NULL,
  };

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  EXPECT_EQ( MIGHTY_BODY_ID, pdRobotFindLinkIDByIKCellName( &robot, torso_names ) );
  EXPECT_EQ( MIGHTY_LF_ID, pdRobotFindLinkIDByIKCellName( &robot, left_foot_names ) );
}

TEST_F(pdRobotTest, FindLinkIDByIKCellName_ReturnInvalid)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  const char *left_hand_names[] = {
    PD_ROBOT_IKCELL_NAME_LH_POS,
    PD_ROBOT_IKCELL_NAME_LH_ATT,
    NULL,
  };

  pdRobotLoad( &robot, model );
  ASSERT_TRUE( pdRobotIKSolver( &robot ) );
  EXPECT_EQ( -1, pdRobotFindLinkIDByIKCellName( &robot, left_hand_names ) );
}

TEST_F(pdRobotTest, LinkSetJointDis)
{
  char model[] = "model/mighty.ztk";
  zVec dis;
  double d[1];

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );

  d[0] = 0.02;
  pdRobotLinkJointSetDis( &robot, 1, d );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_EQ( 0.02, zVecElem( dis, 6 ) );
  d[0] = 0.04;
  pdRobotLinkJointSetDis( &robot, 3, d );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_EQ( 0.04, zVecElem( dis, 8 ) );
  zVecFree( dis );
}

TEST_F(pdRobotTest, SetJointDis)
{
  char model[] = "model/mighty.ztk";
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

TEST_F(pdRobotTest, GetJointDiffAll)
{
  zVec3D com_pos;
  zVec diff;

  LoadAndSolveIK();
  zVec3DCreate( &com_pos, 0.0, 0.0, 0.27 );
  diff = zVecAlloc( pdRobotJointSize(&robot) );
  pdRobotSetRefCOM( &robot, &com_pos );
  rkIKCellEnable( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) );
  rkIKCellEnable( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) );
  pdRobotSolveIK( &robot, 0 );
  pdRobotGetJointDiffAll( &robot, diff );
  for(int i=0; i<pdRobotJointSize(&robot); ++i){
    if( fabs(zVecElem(diff,i) > GTEST_TOL_LOOSE ) ){
      SUCCEED();
      return;
    }
  }
  FAIL() << "Any element of joint difference should be greater or smaller than zero.";
}

TEST_F(pdRobotTest, GetJointVelAll)
{
  zVec3D com_pos;
  zVec vel;

  LoadAndSolveIK();
  zVec3DCreate( &com_pos, 0.0, 0.0, 0.27 );
  vel = zVecAlloc( pdRobotJointSize(&robot) );
  pdRobotSetRefCOM( &robot, &com_pos );
  rkIKCellEnable( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) );
  rkIKCellEnable( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) );
  pdRobotSolveIK( &robot, 0 );
  pdRobotGetJointVelAll( &robot, DT, vel );
  for(int i=0; i<pdRobotJointSize(&robot); ++i){
    if( fabs(zVecElem(vel,i) > GTEST_TOL_LOOSE ) ){
      SUCCEED();
      return;
    }
  }
  FAIL() << "Any element of joint velocity should be greater or smaller than zero.";
}

TEST_F(pdRobotTest, FK)
{
  char model[] = "model/mighty.ztk";
  zVec dis;

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );
  zVecZero( dis );
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
  zVecZero( dis );
  zVecSetElem( dis,  7, 0.01 );
  zVecSetElem( dis,  8, 0.02 );
  zVecSetElem( dis,  9, 0.03 );
  zVecSetElem( dis, 10, 0.04 );
  zVecSetElem( dis, 11, 0.05 );

  pdRobotFK( &robot, dis );
  pdRobotGetJointDisAll( &robot, dis );
  pdRobotGetJointVelAll( &robot, DT, vel );
  for(int i=0; i<26; i++){
    ASSERT_DOUBLE_EQ( 0.0, zVecElem( vel, i ) );
  }
  zVecFree( dis );
  zVecFree( vel );
}

TEST_F(pdRobotTest, FK_LargeBodyOffset)
{
  char model[] = "model/mighty.ztk";
  zVec dis;

  pdRobotLoad( &robot, model );
  dis = zVecAlloc( 26 );
  zVecZero( dis );
  zVecSetElem( dis, 0, 10 );
  zVecSetElem( dis, 1, 10 );
  zVecSetElem( dis, 2, 10 );
  zVecSetElem( dis, 3, 0 );
  zVecSetElem( dis, 4, 0 );
  zVecSetElem( dis, 5, 0 );

  pdRobotFK( &robot, dis );
  double lf, rf;
  lf = rkChainLinkWldPos(pdRobotChain(&robot), pdRobotLFID(&robot))->c.z;
  rf = rkChainLinkWldPos(pdRobotChain(&robot), pdRobotRFID(&robot))->c.z;
  EXPECT_NEAR( 0, lf, 1e-06 );
  EXPECT_NEAR( 0, rf, 1e-06 );
  zVecFree( dis );
}

TEST_F(pdRobotTest, FKIndex)
{
  char model[] = "model/mighty.ztk";
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
  EXPECT_NEAR( 0.2996089431, com.c.z, 1e-06 );
}

TEST_F(pdRobotTest, ResetJointDis)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  pdRobotResetJointDis( &robot );
  for(int i=0; i<26; i++){
    if( i==2 )
      EXPECT_NE( 0, zVecElem( pdRobotJointDis(&robot), i ) );
    else
      EXPECT_EQ( 0, zVecElem( pdRobotJointDis(&robot), i ) );
  }
}

TEST_F(pdRobotTest, SetRef)
{
  char model[] = "model/mighty.ztk";
  zVec3D v;

  pdRobotLoad( &robot, model );
  EXPECT_TRUE( pdRobotSetRef( &robot, "com", 0.1, 0.2, 0.3 ) );
  pdRobotGetRefPos( &robot, "com", &v );
  EXPECT_DOUBLE_EQ( 0.1, v.c.x );
  EXPECT_DOUBLE_EQ( 0.2, v.c.y );
  EXPECT_DOUBLE_EQ( 0.3, v.c.z );
}

TEST_F(pdRobotTest, SetRefVec)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.1, 0.2, 0.3 );
  EXPECT_TRUE( pdRobotSetRefVec( &robot, "com", &v1 ) );
  pdRobotGetRefPos( &robot, "com", &v2 );
  EXPECT_DOUBLE_EQ( 0.1, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.2, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.3, v2.c.z );
}

TEST_F(pdRobotTest, SetRefAtt)
{
  char model[] = "model/mighty.ztk";
  zMat3D m1, m2;

  pdRobotLoad( &robot, model );
  zMat3DCreate( &m1, 0.1, 0.2, 0.3, 0.2, 0.3, 0.1, 0.3, 0.1, 0.2 );
  EXPECT_TRUE( pdRobotSetRefAtt( &robot, "torso_att", &m1 ) );
  pdRobotGetRefAtt( &robot, "torso_att", &m2 );
  EXPECT_DOUBLE_EQ( 0.1, m2.e[0][0] );
  EXPECT_DOUBLE_EQ( 0.2, m2.e[0][1] );
  EXPECT_DOUBLE_EQ( 0.3, m2.e[0][2] );
  EXPECT_DOUBLE_EQ( 0.2, m2.e[1][0] );
  EXPECT_DOUBLE_EQ( 0.3, m2.e[1][1] );
  EXPECT_DOUBLE_EQ( 0.1, m2.e[1][2] );
  EXPECT_DOUBLE_EQ( 0.3, m2.e[2][0] );
  EXPECT_DOUBLE_EQ( 0.1, m2.e[2][1] );
  EXPECT_DOUBLE_EQ( 0.2, m2.e[2][2] );
}

TEST_F(pdRobotTest, SetRef_NotFoundCell)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";

  pdRobotLoad( &robot, model );
  testing::internal::CaptureStderr();
  EXPECT_FALSE( pdRobotSetRef( &robot, "left_hand_pos", 0.1, 0.2, 0.3 ) );
  std::string msg = testing::internal::GetCapturedStderr();
  std::string expected = \
    "run-time error: constraint 'left_hand_pos' is not bound (pdRobotSetRef).\n";
  EXPECT_EQ( expected, msg );
}

TEST_F(pdRobotTest, SetRefCOM)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0, 0, 0.26 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_COM ) ) );
  ASSERT_TRUE( pdRobotSetRefCOM( &robot, &v1 ) );
  pdRobotGetRefCOM( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0, v2.c.x );
  EXPECT_DOUBLE_EQ( 0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.26, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_COM ) ) );
}

TEST_F(pdRobotTest, SetRefTorsoZYX)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.1*zPI_2, 0.0, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_TORSO_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefTorsoZYX( &robot, &v1 ) );
  pdRobotGetRefTorsoZYX( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0.1*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_TORSO_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefTorsoAtt)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;
  zMat3D m1, m2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.1*zPI_2, 0.0, 0.0 );
  zMat3DFromZYX( &m1, v1.c.x, v1.c.y, v1.c.z );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_TORSO_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefTorsoAtt( &robot, &m1 ) );
  pdRobotGetRefTorsoAtt( &robot, &m2 );
  zMat3DToZYX( &m2, &v2 );
  EXPECT_DOUBLE_EQ( 0.1*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_TORSO_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefLFPos)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.0, 0.042, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) ) );
  ASSERT_TRUE( pdRobotSetRefLFPos( &robot, &v1 ) );
  pdRobotGetRefLFPos( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.042, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_POS ) ) );
}

TEST_F(pdRobotTest, SetRefLFZYX)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.2*zPI_2, 0.0, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefLFZYX( &robot, &v1 ) );
  pdRobotGetRefLFZYX( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0.2*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefLFAtt)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;
  zMat3D m1, m2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.2*zPI_2, 0.0, 0.0 );
  zMat3DFromZYX( &m1, v1.c.x, v1.c.y, v1.c.z );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefLFAtt( &robot, &m1 ) );
  pdRobotGetRefLFAtt( &robot, &m2 );
  zMat3DToZYX( &m2, &v2 );
  EXPECT_DOUBLE_EQ( 0.2*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LF_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefRFPos)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0, -0.042, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) ) );
  ASSERT_TRUE( pdRobotSetRefRFPos( &robot, &v1 ) );
  pdRobotGetRefRFPos( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0, v2.c.x );
  EXPECT_DOUBLE_EQ( -0.042, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_POS ) ) );
}

TEST_F(pdRobotTest, SetRefRFZYX)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.3*zPI_2, 0.0, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefRFZYX( &robot, &v1 ) );
  pdRobotGetRefRFZYX( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0.3*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefRFAtt)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;
  zMat3D m1, m2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.3*zPI_2, 0.0, 0.0 );
  zMat3DFromZYX( &m1, v1.c.x, v1.c.y, v1.c.z );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefRFAtt( &robot, &m1 ) );
  pdRobotGetRefRFAtt( &robot, &m2 );
  zMat3DToZYX( &m2, &v2 );
  EXPECT_DOUBLE_EQ( 0.3*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RF_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefLHPos)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0, 0.05, 0.27 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LH_POS ) ) );
  ASSERT_TRUE( pdRobotSetRefLHPos( &robot, &v1 ) );
  pdRobotGetRefLHPos( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.05, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.27, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LH_POS ) ) );
}

TEST_F(pdRobotTest, SetRefLHZYX)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.4*zPI_2, 0.0, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LH_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefLHZYX( &robot, &v1 ) );
  pdRobotGetRefLHZYX( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0.4*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LH_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefLHAtt)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;
  zMat3D m1, m2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.4*zPI_2, 0.0, 0.0 );
  zMat3DFromZYX( &m1, v1.c.x, v1.c.y, v1.c.z );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LH_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefLHAtt( &robot, &m1 ) );
  pdRobotGetRefLHAtt( &robot, &m2 );
  zMat3DToZYX( &m2, &v2 );
  EXPECT_DOUBLE_EQ( 0.4*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_LH_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefRHPos)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0, -0.05, 0.27 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RH_POS ) ) );
  ASSERT_TRUE( pdRobotSetRefRHPos( &robot, &v1 ) );
  pdRobotGetRefRHPos( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0, v2.c.x );
  EXPECT_DOUBLE_EQ( -0.05, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.27, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RH_POS ) ) );
}

TEST_F(pdRobotTest, SetRefRHZYX)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.5*zPI_2, 0.0, 0.0 );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RH_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefRHZYX( &robot, &v1 ) );
  pdRobotGetRefRHZYX( &robot, &v2 );
  EXPECT_DOUBLE_EQ( 0.5*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RH_ATT ) ) );
}

TEST_F(pdRobotTest, SetRefRHAtt)
{
  char model[] = "model/mighty.ztk";
  zVec3D v1, v2;
  zMat3D m1, m2;

  pdRobotLoad( &robot, model );
  zVec3DCreate( &v1, 0.5*zPI_2, 0.0, 0.0 );
  zMat3DFromZYX( &m1, v1.c.x, v1.c.y, v1.c.z );
  ASSERT_FALSE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RH_ATT ) ) );
  ASSERT_TRUE( pdRobotSetRefRHAtt( &robot, &m1 ) );
  pdRobotGetRefRHAtt( &robot, &m2 );
  zMat3DToZYX( &m2, &v2 );
  EXPECT_DOUBLE_EQ( 0.5*zPI_2, v2.c.x );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.y );
  EXPECT_DOUBLE_EQ( 0.0, v2.c.z );
  EXPECT_TRUE( rkIKCellIsEnabled( pdRobotFindIKCellByName( &robot, PD_ROBOT_IKCELL_NAME_RH_ATT ) ) );
}

TEST_F(pdRobotTest, RegisterIKJointID)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKSolver( &robot );

  const int CHECK_ID = 1;        // left_shoulder_flexion
  EXPECT_FALSE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_EQ( 0, ik->joint_weight[CHECK_ID] );

  // method to testify
  pdRobotRegisterIKJointID( &robot, CHECK_ID, 0.01 );
  EXPECT_TRUE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_EQ( 0.01, ik->joint_weight[CHECK_ID] );
}

TEST_F(pdRobotTest, RegisterIKJointIDAll)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  rkChain *c;
  rkIK *ik;

  pdRobotLoad( &robot, model );
  c = pdRobotChain( &robot );
  ik = pdRobotIKSolver( &robot );

  // method to testify
  pdRobotRegisterIKJointAll( &robot, 0.02 );
  for( int i=0; i<pdRobotLinkNum(&robot); i++ )
    if( rkChainLinkJoint(c,i)->com != &rk_joint_fixed ){
      EXPECT_TRUE( ik->joint_is_enabled[i] );
      EXPECT_EQ( 0.02, ik->joint_weight[i] );
    }
}

TEST_F(pdRobotTest, UnregisterIKJointID)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKSolver( &robot );

  const int CHECK_ID = 1;        // left_shoulder_flexion
  pdRobotRegisterIKJointID( &robot, CHECK_ID, 0.01 );
  EXPECT_TRUE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_LT( 0.0, ik->joint_weight[CHECK_ID] );

  // method to testify
  pdRobotUnregisterIKJointID( &robot, CHECK_ID );
  EXPECT_FALSE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_EQ( 0, ik->joint_weight[CHECK_ID] );
}

TEST_F(pdRobotTest, RegisterIKJoint)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKSolver( &robot );

  const int CHECK_ID = 1;        // left_shoulder_flexion
  EXPECT_FALSE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_EQ( 0, ik->joint_weight[CHECK_ID] );

  // method to testify
  pdRobotRegisterIKJoint( &robot, "left_shoulder_flexion", 0.01 );
  EXPECT_TRUE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_EQ( 0.01, ik->joint_weight[CHECK_ID] );
}

TEST_F(pdRobotTest, UnregisterIKJoint)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";
  rkIK *ik;

  pdRobotLoad( &robot, model );
  ik = pdRobotIKSolver( &robot );

  const int CHECK_ID = 1;        // left_shoulder_flexion
  pdRobotRegisterIKJoint( &robot, "left_shoulder_flexion", 0.01 );
  EXPECT_TRUE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_LT( 0.0, ik->joint_weight[CHECK_ID] );

  // method to testify
  pdRobotUnregisterIKJoint( &robot, "left_shoulder_flexion" );
  EXPECT_FALSE( ik->joint_is_enabled[CHECK_ID] );
  EXPECT_EQ( 0, ik->joint_weight[CHECK_ID] );
}

TEST_F(pdRobotTest, SolveIK)
{
  zVec3D v;

  LoadAndSolveIK();
  EXPECT_NEAR( 0.0,    rkChainWldCOM( pdRobotChain( &robot ) )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainWldCOM( pdRobotChain( &robot ) )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.26,   rkChainWldCOM( pdRobotChain( &robot ) )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_LF_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.042,  rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_LF_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_LF_ID )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_RF_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.042, rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_RF_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_RF_ID )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_LH_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.13,   rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_LH_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.25,   rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_LH_ID )->e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0,    rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_RH_ID )->e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.13,  rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_RH_ID )->e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.25,   rkChainLinkWldPos( pdRobotChain( &robot ), MIGHTY_RH_ID )->e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( &robot ), MIGHTY_BODY_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( &robot ), MIGHTY_LF_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( &robot ), MIGHTY_RF_ID ), &v);
  EXPECT_NEAR( 0.0, v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, v.e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( &robot ), MIGHTY_LH_ID ), &v);
  EXPECT_NEAR( 1.3,    v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.01,  v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.373, v.e[2], GTEST_TOL_LOOSE );
  zMat3DToZYX( rkChainLinkWldAtt( pdRobotChain( &robot ), MIGHTY_RH_ID ), &v);
  EXPECT_NEAR( -1.3,   v.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.005, v.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.373,  v.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, SolveIK_CheckJointVel)
{
  zVec vel;

  LoadAndSolveIK();
  vel = zVecAlloc(26);
  pdRobotGetJointVelAll( &robot, DT, vel );
  for(int i=0; i<26; i++){
    if( fabs(zVecElem(vel,i)) > GTEST_TOL_LOOSE ){
      SUCCEED();
      return;
    }
  }
  FAIL() << "Any element of velocity should be greater or smaller than zero.";
}

TEST_F(pdRobotTest, Load_EnsureIKDisabledAfterSolveIK)
{
  rkIKCell *cp;

  LoadAndSolveIK();
  zListForEach( pdRobotIKCellList(&robot), cp )
    EXPECT_FALSE( rkIKCellIsEnabled(cp) );
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

TEST_F(pdRobotTest, TorsoZYX)
{
  zVec3D att;

  LoadAndSolveIK();
  zVec3DCreate( &att, 1, 2, 3 );
  pdRobotTorsoZYX( &robot, &att );
  EXPECT_NEAR( 0.0, att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, TorsoAtt)
{
  zMat3D att;

  LoadAndSolveIK();
  zMat3DCreate( &att, 1, 2, 3, 4, 5, 6, 7, 8, 9 );
  pdRobotTorsoAtt( &robot, &att );
  EXPECT_NEAR( 1.0, att.e[0][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[0][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[0][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[1][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, att.e[1][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[1][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[2][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, att.e[2][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, att.e[2][2], GTEST_TOL_LOOSE );
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

TEST_F(pdRobotTest, FootZYX)
{
  zVec3D lf, rf;

  LoadAndSolveIK();
  zVec3DCreate( &lf, 1, 2, 3 );
  zVec3DCreate( &rf, 4, 5, 6 );
  pdRobotFootZYX( &robot, &lf, &rf );
  EXPECT_NEAR( 0.0, lf.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, FootAtt)
{
  zMat3D lf, rf;

  LoadAndSolveIK();
  zMat3DCreate( &lf, 1, 2, 3, 4, 5, 6, 7, 8, 9 );
  zMat3DCreate( &rf, 1, 2, 3, 4, 5, 6, 7, 8, 9 );
  pdRobotFootAtt( &robot, &lf, &rf );
  EXPECT_NEAR( 1.0, lf.e[0][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[0][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[0][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[1][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, lf.e[1][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[1][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[2][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, lf.e[2][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, lf.e[2][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, rf.e[0][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[0][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[0][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[1][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, rf.e[1][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[1][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[2][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, rf.e[2][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 1.0, rf.e[2][2], GTEST_TOL_LOOSE );
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

TEST_F(pdRobotTest, HandZYX)
{
  zVec3D lh, rh;

  LoadAndSolveIK();
  zVec3DCreate( &lh, 1, 2, 3 );
  zVec3DCreate( &rh, 4, 5, 6 );
  pdRobotHandZYX( &robot, &lh, &rh );
  EXPECT_NEAR( 1.3, lh.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.01, lh.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.373, lh.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -1.3, rh.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.005, rh.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.373, rh.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, HandAtt)
{
  zMat3D lh, rh;
  zMat3D expected_att;

  LoadAndSolveIK();
  zMat3DCreate( &lh, 1, 2, 3, 4, 5, 6, 7, 8, 9 );
  zMat3DCreate( &rh, 1, 2, 3, 4, 5, 6, 7, 8, 9 );
  pdRobotHandAtt( &robot, &lh, &rh );

  zMat3DFromZYX( &expected_att, 1.3, -0.01, -0.373 );
  EXPECT_NEAR( expected_att.e[0][0], lh.e[0][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[0][1], lh.e[0][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[0][2], lh.e[0][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[1][0], lh.e[1][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[1][1], lh.e[1][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[1][2], lh.e[1][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[2][0], lh.e[2][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[2][1], lh.e[2][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[2][2], lh.e[2][2], GTEST_TOL_LOOSE );

  zMat3DFromZYX( &expected_att, -1.3, -0.005, 0.373 );
  EXPECT_NEAR( expected_att.e[0][0], rh.e[0][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[0][1], rh.e[0][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[0][2], rh.e[0][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[1][0], rh.e[1][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[1][1], rh.e[1][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[1][2], rh.e[1][2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[2][0], rh.e[2][0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[2][1], rh.e[2][1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( expected_att.e[2][2], rh.e[2][2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, BipedDefaultInit)
{
  char model[] = "model/mighty.ztk";
  zVec dis, vel;

  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );
  pdRobotLoad( &robot, model );
  pdRobotBipedDefaultInit( &robot, &biped, &state );
  dis = zVecAlloc( pdRobotJointSize( &robot ) );
  vel = zVecAlloc( pdRobotJointSize( &robot ) );
  pdRobotGetJointDisAll( &robot, dis );
  pdRobotGetJointVelAll( &robot, DT, vel );

  for(int i=0; i<26; i++){
    ASSERT_DOUBLE_EQ( zVecElem(dis,i), zVecElem(robot.disold,i) );
  }
  for(int i=0; i<26; i++){
    ASSERT_DOUBLE_EQ( 0.0, zVecElem(vel,i) );
  }
  zVecFree( dis );
  zVecFree( vel );
}

TEST_F(pdRobotTest, BipedResetPose)
{
  char model[] = "model/mighty.ztk";
  zVec dis;

  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );
  pdRobotLoad( &robot, model );
  pdRobotBipedDefaultInit( &robot, &biped, &state );
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
  pdRobotBipedResetPose( &robot, &biped, &state, dis );
  pdRobotGetJointDisAll( &robot, dis );
  EXPECT_DOUBLE_EQ( -1, zVecElem(dis,0) );
  EXPECT_DOUBLE_EQ( 1,  zVecElem(dis,1) );
  EXPECT_NEAR( 0.347, zVecElem(dis,2), GTEST_TOL_LOOSE );
  EXPECT_DOUBLE_EQ( 0, zVecElem(dis,3) );
  EXPECT_DOUBLE_EQ( 0, zVecElem(dis,4) );
  EXPECT_DOUBLE_EQ( 1.57, zVecElem(dis,5) );
  zVecFree( dis );
}

TEST_F(pdRobotTest, BipedResetPose_CheckVel)
{
  char model[] = "model/mighty.ztk";
  zVec dis, vel;

  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );
  pdRobotLoad( &robot, model );
  pdRobotBipedDefaultInit( &robot, &biped, &state );
  dis = zVecAlloc( pdRobotJointSize( &robot ) );
  vel = zVecAlloc( pdRobotJointSize( &robot ) );
  pdRobotGetJointDisAll( &robot, dis );
  pdRobotUpdateState( &robot, &state );

  // method to testify
  zVecSetElem( dis, 0, -1 );
  zVecSetElem( dis, 1, 1 );
  zVecSetElem( dis, 2, 0.34 );
  zVecSetElem( dis, 3, 0 );
  zVecSetElem( dis, 4, 0 );
  zVecSetElem( dis, 5, 1.57 );
  pdRobotBipedResetPose( &robot, &biped, &state, dis );
  pdRobotGetJointVelAll( &robot, DT, vel );
  for(int i=0; i<26; i++){
    ASSERT_DOUBLE_EQ( 0.0, zVecElem(vel,i) );
  }
  zVecFree( dis );
  zVecFree( vel );
}

TEST_F(pdRobotTest, BipedSetRefVec)
{
  char model[] = "model/mighty.ztk";

  pdRobotLoad( &robot, model );
  // set random values to referential vectors
  ri.SetRandVec3D( pdBipedRefCOMPos(&biped) );
  ri.SetRandVec3D( pdBipedRefTorsoAtt(&biped) );
  ri.SetRandVec3D( pdBipedRefLFPos(&biped) );
  ri.SetRandVec3D( pdBipedRefLFAtt(&biped) );
  ri.SetRandVec3D( pdBipedRefRFPos(&biped) );
  ri.SetRandVec3D( pdBipedRefRFAtt(&biped) );

  // call method to testify
  pdRobotBipedSetRefVec( &robot, &biped );

  zVec3D v, e;
  zMat3D m;
  pdRobotGetRefCOM( &robot, &v );
  EXPECT_DOUBLE_EQ( pdBipedRefCOMPosX(&biped), v.c.x );
  EXPECT_DOUBLE_EQ( pdBipedRefCOMPosY(&biped), v.c.y );
  EXPECT_DOUBLE_EQ( pdBipedRefCOMPosZ(&biped), v.c.z );

  pdRobotGetRefTorsoZYX( &robot, &v );
  zMat3DFromZYX( &m, pdBipedRefTorsoAtt(&biped)->c.x,
                 pdBipedRefTorsoAtt(&biped)->c.y, pdBipedRefTorsoAtt(&biped)->c.z );
  zMat3DToZYX( &m, &e );
  EXPECT_DOUBLE_EQ( e.c.x, v.c.x );
  EXPECT_DOUBLE_EQ( e.c.y, v.c.y );
  EXPECT_DOUBLE_EQ( e.c.z, v.c.z );

  pdRobotGetRefLFPos( &robot, &v );
  EXPECT_DOUBLE_EQ( pdBipedRefLFPosX(&biped), v.c.x );
  EXPECT_DOUBLE_EQ( pdBipedRefLFPosY(&biped), v.c.y );
  EXPECT_DOUBLE_EQ( pdBipedRefLFPosZ(&biped), v.c.z );

  pdRobotGetRefLFZYX( &robot, &v );
  zMat3DFromZYX( &m, pdBipedRefLFAtt(&biped)->c.x,
                 pdBipedRefLFAtt(&biped)->c.y, pdBipedRefLFAtt(&biped)->c.z );
  zMat3DToZYX( &m, &e );
  EXPECT_DOUBLE_EQ( e.c.x, v.c.x );
  EXPECT_DOUBLE_EQ( e.c.y, v.c.y );
  EXPECT_DOUBLE_EQ( e.c.z, v.c.z );

  pdRobotGetRefRFPos( &robot, &v );
  EXPECT_DOUBLE_EQ( pdBipedRefRFPosX(&biped), v.c.x );
  EXPECT_DOUBLE_EQ( pdBipedRefRFPosY(&biped), v.c.y );
  EXPECT_DOUBLE_EQ( pdBipedRefRFPosZ(&biped), v.c.z );

  pdRobotGetRefRFZYX( &robot, &v );
  zMat3DFromZYX( &m, pdBipedRefRFAtt(&biped)->c.x,
                 pdBipedRefRFAtt(&biped)->c.y, pdBipedRefRFAtt(&biped)->c.z );
  zMat3DToZYX( &m, &e );
  EXPECT_DOUBLE_EQ( e.c.x, v.c.x );
  EXPECT_DOUBLE_EQ( e.c.y, v.c.y );
  EXPECT_DOUBLE_EQ( e.c.z, v.c.z );
}

TEST_F(pdRobotTest, SupportRegion_Double)
{
  char model[] = "model/mighty.ztk";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D torso_att, lf_att, rf_att;
  zLoop3D sr_lf, sr_rf, sr;
  zLoop3DCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(   &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(    &lf_pos, 0.0, 0.042, 0.0 );
  zVec3DCreate(    &rf_pos, 0.0, -0.042, 0.0 );
  zVec3DCreate( &torso_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );
  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );

  // Left foot
  ASSERT_EQ( 4, zListSize( &sr_lf ));
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

  // Right foot
  ASSERT_EQ( 4, zListSize( &sr_rf ));
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

  // Support region
  zVec3D p[4];
  zVec3DCreate( &p[0],  0.0564,  0.078, 0.0 );
  zVec3DCreate( &p[1],  0.0564, -0.078, 0.0 );
  zVec3DCreate( &p[2], -0.0426, -0.078, 0.0 );
  zVec3DCreate( &p[3], -0.0426,  0.078, 0.0 );
  ASSERT_GE( zListSize( &sr ), 4 );
  bool found = false;
  for( int i=0; i<4; i++ ){
    found = false;
    zListForEach( &sr, cp )
      if( zVec3DEqual( &p[i], cp->data ) ) found = true;
    EXPECT_TRUE( found ) << "Expected point was not found: ("
                         << p[i].e[0] << ", "
                         << p[i].e[1] << ", "
                         << p[i].e[2] << ")";
  }
}

TEST_F(pdRobotTest, SupportRegion_Single_Left)
{
  char model[] = "model/mighty.ztk";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D torso_att, lf_att, rf_att;
  zLoop3D sr_lf, sr_rf, sr;
  zLoop3DCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(   &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(    &lf_pos, 0.0, 0.042, 0.0 );
  zVec3DCreate(    &rf_pos, 0.0, -0.042, 0.01 );
  zVec3DCreate( &torso_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );

  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );

  // Left foot
  ASSERT_EQ( 4, zListSize( &sr_lf ));
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

  // Right foot
  ASSERT_EQ( 0, zListSize( &sr_rf ) );

  // Support region
  ASSERT_EQ( 4, zListSize( &sr ));
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
  char model[] = "model/mighty.ztk";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D torso_att, lf_att, rf_att;
  zLoop3D sr_lf, sr_rf, sr;
  zLoop3DCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(   &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(    &lf_pos, 0.0, 0.042, 0.01 );
  zVec3DCreate(    &rf_pos, 0.0, -0.042, 0.0 );
  zVec3DCreate( &torso_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &rf_att, 0.0, 0.0, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );

  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );

  // Left foot
  ASSERT_EQ( 0, zListSize( &sr_lf ));

  // Right foot
  ASSERT_EQ( 4, zListSize( &sr_rf ));
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

  // Support region
  ASSERT_EQ( 4, zListSize( &sr ));
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

TEST_F(pdRobotTest, SupportRegion_SingleToDouble)
{
  char model[] = "model/mighty.ztk";
  zVec3D com_pos, lf_pos, rf_pos;
  zVec3D torso_att, lf_att, rf_att;
  zLoop3D sr_lf, sr_rf, sr;
  zLoop3DCell *cp;

  pdRobotLoad( &robot, model );
  zVec3DCreate(   &com_pos, 0.0, 0.0, 0.26 );
  zVec3DCreate(    &lf_pos, 0.0, 0.042, 0.0 );
  zVec3DCreate(    &rf_pos, 0.0, -0.042, 0.0 );
  zVec3DCreate( &torso_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &lf_att, 0.0, 0.0, 0.0 );
  zVec3DCreate(    &rf_att, 0.0, 0.0, 0.0 );

  // Single support phase on left foot
  zVec3DCreate( &lf_pos, 0.0,  0.042, 0.0 );
  zVec3DCreate( &rf_pos, 0.0, -0.042, 0.01 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );
  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );
  // Left foot
  ASSERT_EQ( 4, zListSize( &sr_lf ));
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
  // Right foot
  ASSERT_EQ( 0, zListSize( &sr_rf ));
  // Support region
  ASSERT_EQ( 4, zListSize( &sr ));
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

  // Double support phase (1st)
  zVec3DCreate( &lf_pos, 0.0,  0.042, 0.0 );
  zVec3DCreate( &rf_pos, 0.0, -0.042, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );
  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );
  // Left foot
  ASSERT_EQ( 4, zListSize( &sr_lf ));
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
  // Right foot
  ASSERT_EQ( 4, zListSize( &sr_rf ));
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
  // Support region
  zVec3D p[4];
  zVec3DCreate( &p[0],  0.0564,  0.078, 0.0 );
  zVec3DCreate( &p[1],  0.0564, -0.078, 0.0 );
  zVec3DCreate( &p[2], -0.0426, -0.078, 0.0 );
  zVec3DCreate( &p[3], -0.0426,  0.078, 0.0 );
  ASSERT_GE( zListSize( &sr ), 4 );
  bool found = false;
  for( int i=0; i<4; i++ ){
    found = false;
    zListForEach( &sr, cp )
      if( zVec3DEqual( &p[i], cp->data ) ) found = true;
    EXPECT_TRUE( found ) << "Expected point was not found: ("
                         << p[i].e[0] << ", "
                         << p[i].e[1] << ", "
                         << p[i].e[2] << ")";
  }

  // Single support phase on right foot
  zVec3DCreate( &lf_pos, 0.0,  0.042, 0.01 );
  zVec3DCreate( &rf_pos, 0.0, -0.042, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );
  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );
  // Left foot
  ASSERT_EQ( 0, zListSize( &sr_lf ));
  // Right foot
  ASSERT_EQ( 4, zListSize( &sr_rf ));
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
  // Support region
  ASSERT_EQ( 4, zListSize( &sr ));
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

  // Double support phase (2nd)
  zVec3DCreate( &lf_pos, 0.0,  0.042, 0.0 );
  zVec3DCreate( &rf_pos, 0.0, -0.042, 0.0 );
  pdRobotSetRefCOM( &robot, &com_pos );
  pdRobotSetRefLFPos( &robot, &lf_pos );
  pdRobotSetRefRFPos( &robot, &rf_pos );
  pdRobotSetRefTorsoZYX( &robot, &torso_att );
  pdRobotSetRefLFZYX( &robot, &lf_att );
  pdRobotSetRefRFZYX( &robot, &rf_att );
  pdRobotSolveIK( &robot, 0 );
  pdRobotSupportRegion( &robot, &sr_lf, &sr_rf, &sr );
  // Left foot
  ASSERT_EQ( 4, zListSize( &sr_lf ));
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
  // Right foot
  ASSERT_EQ( 4, zListSize( &sr_rf ));
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
  // Support region
  zVec3DCreate( &p[0],  0.0564,  0.078, 0.0 );
  zVec3DCreate( &p[1],  0.0564, -0.078, 0.0 );
  zVec3DCreate( &p[2], -0.0426, -0.078, 0.0 );
  zVec3DCreate( &p[3], -0.0426,  0.078, 0.0 );
  ASSERT_GE( zListSize( &sr ), 4 );
  for( int i=0; i<4; i++ ){
    found = false;
    zListForEach( &sr, cp )
      if( zVec3DEqual( &p[i], cp->data ) ) found = true;
    EXPECT_TRUE( found ) << "Expected point was not found: ("
                         << p[i].e[0] << ", "
                         << p[i].e[1] << ", "
                         << p[i].e[2] << ")";
  }
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
  EXPECT_NEAR( 0.0, state.torso_att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, state.torso_att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.0, state.torso_att.e[2], GTEST_TOL_LOOSE );
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
  EXPECT_NEAR( 1.3,    state.lh_att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.01,  state.lh_att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.373, state.lh_att.e[2], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -1.3,   state.rh_att.e[0], GTEST_TOL_LOOSE );
  EXPECT_NEAR( -0.005, state.rh_att.e[1], GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.373,  state.rh_att.e[2], GTEST_TOL_LOOSE );
}

TEST_F(pdRobotTest, Print)
{
  char model[] = "model/mighty_no_hand_constraint.ztk";

  pdRobotLoad( &robot, model );

  testing::internal::CaptureStdout();
  pdRobotPrint( &robot );
  std::string msg = testing::internal::GetCapturedStdout();
  std::string expected = \
    "==========\n" \
    "          Torso ID: 0\n" \
    "Left/Right foot ID: 12/24\n" \
    "Left/Right hand ID: -1/-1\n" \
    "IK cell size: 6\n" \
    "----------\n" \
    "IK Cell Name: left_foot_pos (world_pos)\n" \
    " Ref (position): ( 0, 0, 0 )\n" \
    "Link ID(Sub ID): 12 (0)\n" \
    "Attention Point: ( 0, 0, 0 )\n" \
    "         Weight: ( 1, 1, 1 )\n" \
    " Attribute Mask: 0x01\n" \
    "       Priority: 5\n" \
    "      Cell Mode: 0x07\n" \
    "       Enabled?: false\n" \
    "----------\n" \
    "IK Cell Name: right_foot_pos (world_pos)\n" \
    " Ref (position): ( 0, 0, 0 )\n" \
    "Link ID(Sub ID): 24 (0)\n" \
    "Attention Point: ( 0, 0, 0 )\n" \
    "         Weight: ( 1, 1, 1 )\n" \
    " Attribute Mask: 0x01\n" \
    "       Priority: 5\n" \
    "      Cell Mode: 0x07\n" \
    "       Enabled?: false\n" \
    "----------\n" \
    "IK Cell Name: left_foot_att (world_att)\n" \
    " Ref (attitude): {\n" \
    " 0, 0, 0\n" \
    " 0, 0, 0\n" \
    " 0, 0, 0\n" \
    "}\n" \
    "Link ID(Sub ID): 12 (0)\n" \
    "Attention Point: ( 0, 0, 0 )\n" \
    "         Weight: ( 1, 1, 1 )\n" \
    " Attribute Mask: 0x01\n" \
    "       Priority: 4\n" \
    "      Cell Mode: 0x07\n" \
    "       Enabled?: false\n" \
    "----------\n" \
    "IK Cell Name: right_foot_att (world_att)\n" \
    " Ref (attitude): {\n" \
    " 0, 0, 0\n" \
    " 0, 0, 0\n" \
    " 0, 0, 0\n" \
    "}\n" \
    "Link ID(Sub ID): 24 (0)\n" \
    "Attention Point: ( 0, 0, 0 )\n" \
    "         Weight: ( 1, 1, 1 )\n" \
    " Attribute Mask: 0x01\n" \
    "       Priority: 4\n" \
    "      Cell Mode: 0x07\n" \
    "       Enabled?: false\n" \
    "----------\n" \
    "IK Cell Name: com (com)\n" \
    " Ref (position): ( 0, 0, 0 )\n" \
    "Link ID(Sub ID): 0 (0)\n" \
    "Attention Point: ( 0, 0, 0 )\n" \
    "         Weight: ( 1, 1, 1 )\n" \
    " Attribute Mask: 0000\n" \
    "       Priority: 3\n" \
    "      Cell Mode: 0x07\n" \
    "       Enabled?: false\n" \
    "----------\n" \
    "IK Cell Name: torso_att (world_att)\n" \
    " Ref (attitude): {\n" \
    " 0, 0, 0\n" \
    " 0, 0, 0\n" \
    " 0, 0, 0\n" \
    "}\n" \
    "Link ID(Sub ID): 0 (0)\n" \
    "Attention Point: ( 0, 0, 0 )\n" \
    "         Weight: ( 1, 1, 1 )\n" \
    " Attribute Mask: 0x01\n" \
    "       Priority: 2\n" \
    "      Cell Mode: 0x07\n" \
    "       Enabled?: false\n";
  EXPECT_EQ( expected, msg );
}

TEST_F(pdRobotTest, BipedSingleSupportInit)
{
  char model[] = "model/mighty.ztk";
  zVec vel;

  pdCmdDefaultInit( &cmd );
  cmd.zd = 0.26;
  cmd.dist = 0.084;
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );
  pdRobotLoad( &robot, model );
  pdRobotBipedSingleSupportInit( &robot, &biped, &state, PD_FOOT_RIGHT, 0.02 );

  EXPECT_NEAR( 0.0, state.rf_pos.c.z, GTEST_TOL_LOOSE );
  EXPECT_NEAR( 0.02, state.lf_pos.c.z, GTEST_TOL_LOOSE );
  // the COM settles above the stance-foot center
  EXPECT_NEAR( state.rf_pos.c.x, state.com_pos.c.x, GTEST_TOL_LOOSE );
  EXPECT_NEAR( state.rf_pos.c.y, state.com_pos.c.y, GTEST_TOL_LOOSE );
  // the support region reduces to the stance-foot hull
  EXPECT_EQ( 0, zListSize( &state.sr_lf ) );
  EXPECT_GT( zListSize( &state.sr_rf ), 0 );
  EXPECT_EQ( zListSize( &state.sr_rf ), zListSize( &state.sr ) );
  EXPECT_TRUE( biped.mode.balancing );
  // zero joint velocity after initialization
  vel = zVecAlloc( pdRobotJointSize( &robot ) );
  pdRobotGetJointVelAll( &robot, DT, vel );
  for(int i=0; i<26; i++){
    ASSERT_DOUBLE_EQ( 0.0, zVecElem(vel,i) );
  }
  zVecFree( vel );
}

TEST_F(pdRobotTest, BipedResetPoseSingleSupport)
{
  char model[] = "model/mighty.ztk";
  zVec dis;

  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );
  pdRobotLoad( &robot, model );
  dis = zVecAlloc( pdRobotJointSize( &robot ) );
  pdRobotGetJointDisAll( &robot, dis );
  zVecSetElem( dis, 0, -0.6 );
  zVecSetElem( dis, 1, -0.8 );
  pdRobotBipedResetPoseSingleSupport( &robot, &biped, &state, dis, PD_FOOT_RIGHT, 0.02 );

  EXPECT_DOUBLE_EQ( 0.0, state.rf_pos.c.z );
  EXPECT_DOUBLE_EQ( 0.02, state.lf_pos.c.z );
  // the desired ZMP starts at the stance-foot center on the ground
  EXPECT_DOUBLE_EQ( state.rf_pos.c.x, state.deszmp.c.x );
  EXPECT_DOUBLE_EQ( state.rf_pos.c.y, state.deszmp.c.y );
  EXPECT_DOUBLE_EQ( 0.0, state.deszmp.c.z );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DNorm( &state.com_vel ) );
  EXPECT_DOUBLE_EQ( 0.0, zVec3DNorm( &state.com_acc ) );
  EXPECT_TRUE( biped.mode.balancing );
  zVecFree( dis );
}
