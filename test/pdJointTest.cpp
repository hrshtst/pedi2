#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_joint.h>

const double TIME_STEP = 0.01;

class pdJointTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdJointInit( &joint );
  };
  virtual void TearDown() {};

  void SetRandomValues() {
    joint.dis = ri.rand();
    joint.vel = ri.rand();
    joint.disold = ri.rand();
    joint.velold = ri.rand();
    joint.refdis = ri.rand();
    joint.refvel = ri.rand();
    joint.refdisold = ri.rand();
    joint.refvelold = ri.rand();
    joint.output = ri.rand();
  };

  pdJoint joint;
  RandomInitializer ri;
};

TEST_F(pdJointTest, Init)
{
  SetRandomValues();
  pdJointInit( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
}

TEST_F(pdJointTest, SetDisDefault)
{
  double dis1, dis2;

  pdJointInit( &joint );
  dis1 = ri.rand();
  pdJointSetDisDefault( &joint, dis1 );
  EXPECT_EQ( dis1, pdJointDis( &joint ) );

  dis2 = ri.rand();
  pdJointSetDisDefault( &joint, dis2 );
  EXPECT_EQ( dis2, pdJointDis( &joint ) );
  EXPECT_EQ( dis1, pdJointDisOld( &joint ) );
}

TEST_F(pdJointTest, SetVelDefault)
{
  double vel1, vel2;

  pdJointInit( &joint );
  vel1 = ri.rand();
  pdJointSetVelDefault( &joint, vel1 );
  EXPECT_EQ( vel1, pdJointVel( &joint ) );
  EXPECT_TRUE( joint.is_set_vel );

  vel2 = ri.rand();
  pdJointSetVelDefault( &joint, vel2 );
  EXPECT_EQ( vel2, pdJointVel( &joint ) );
  EXPECT_EQ( vel1, pdJointVelOld( &joint ) );
  EXPECT_TRUE( joint.is_set_vel );
}

TEST_F(pdJointTest, SetRefDisDefault)
{
  double refdis1, refdis2;

  pdJointInit( &joint );
  refdis1 = ri.rand();
  pdJointSetRefDisDefault( &joint, refdis1 );
  EXPECT_EQ( refdis1, pdJointRefDis( &joint ) );

  refdis2 = ri.rand();
  pdJointSetRefDisDefault( &joint, refdis2 );
  EXPECT_EQ( refdis2, pdJointRefDis( &joint ) );
  EXPECT_EQ( refdis1, pdJointRefDisOld( &joint ) );
}

TEST_F(pdJointTest, SetRefVelDefault)
{
  double refvel1, refvel2;

  pdJointInit( &joint );
  refvel1 = ri.rand();
  pdJointSetRefVelDefault( &joint, refvel1 );
  EXPECT_EQ( refvel1, pdJointRefVel( &joint ) );
  EXPECT_TRUE( joint.is_set_refvel );

  refvel2 = ri.rand();
  pdJointSetRefVelDefault( &joint, refvel2 );
  EXPECT_EQ( refvel2, pdJointRefVel( &joint ) );
  EXPECT_EQ( refvel1, pdJointRefVelOld( &joint ) );
  EXPECT_TRUE( joint.is_set_refvel );
}

TEST_F(pdJointTest, RefreshDefault)
{
  double dis;

  pdJointInit( &joint );
  SetRandomValues();
  dis = ri.rand();
  pdJointRefreshDefault( &joint, dis );
  EXPECT_EQ( dis, pdJointDis( &joint ) );
  EXPECT_EQ( dis, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0,   pdJointVel( &joint ) );
  EXPECT_EQ( 0,   pdJointVelOld( &joint ) );
  EXPECT_EQ( dis, pdJointRefDis( &joint ) );
  EXPECT_EQ( dis, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0,   pdJointRefVel( &joint ) );
  EXPECT_EQ( 0,   pdJointRefVelOld( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
}

TEST_F(pdJointTest, UpdateDefault)
{
  pdJointInit( &joint );
  pdJointRefreshDefault( &joint, 10 );

  pdJointSetRefDisDefault( &joint, 20 );
  pdJointSetDisDefault( &joint, 10 );
  pdJointUpdateDefault( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );

  pdJointSetRefDisDefault( &joint, 30 );
  pdJointSetDisDefault( &joint, 15 );
  pdJointUpdateDefault( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
}

TEST_F(pdJointTest, UpdateDefault_SetVel)
{
  pdJointInit( &joint );
  pdJointRefreshDefault( &joint, 10 );

  pdJointSetRefDisDefault( &joint, 20 );
  pdJointSetDisDefault( &joint, 10 );
  pdJointUpdateDefault( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );

  pdJointSetRefDisDefault( &joint, 30 );
  pdJointSetDisDefault( &joint, 15 );
  pdJointSetVelDefault( &joint, 600 );
  pdJointUpdateDefault( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 600, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );
}

TEST_F(pdJointTest, UpdateDefault_SetRefVel)
{
  pdJointInit( &joint );
  pdJointRefreshDefault( &joint, 10 );

  pdJointSetRefDisDefault( &joint, 20 );
  pdJointSetDisDefault( &joint, 10 );
  pdJointUpdateDefault( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );

  pdJointSetRefDisDefault( &joint, 30 );
  pdJointSetDisDefault( &joint, 15 );
  pdJointSetRefVelDefault( &joint, 900 );
  pdJointUpdateDefault( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 900, pdJointRefVel( &joint ) );
}

TEST_F(pdJointTest, DestroyDefault)
{
  pdJointInit( &joint );
  zNameSet( &joint, (char*)ZNONAME );
  SetRandomValues();
  // check
  EXPECT_TRUE( zNamePtr(&joint) );
  EXPECT_NE( 0.0, pdJointDis( &joint ) );
  EXPECT_NE( 0.0, pdJointVel( &joint ) );
  EXPECT_NE( 0.0, pdJointDisOld( &joint ) );
  EXPECT_NE( 0.0, pdJointVelOld( &joint ) );
  EXPECT_NE( 0.0, pdJointRefDis( &joint ) );
  EXPECT_NE( 0.0, pdJointRefVel( &joint ) );
  EXPECT_NE( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_NE( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_NE( 0.0, pdJointOutput( &joint ) );
  // test
  pdJointDestroyDefault( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
}


typedef struct{
  double pgain, dgain;
  double trqmin, trqmax;
} _pdJointPDTrq;

class pdJointPDTrqTest : public testing::Test {
protected:
  virtual void SetUp() {
    pgain = ri.rand();
    dgain = ri.rand();
  };
  virtual void TearDown() {};

  void SetRandomValues() {
    joint.dis = ri.rand();
    joint.vel = ri.rand();
    joint.disold = ri.rand();
    joint.velold = ri.rand();
    joint.refdis = ri.rand();
    joint.refvel = ri.rand();
    joint.refdisold = ri.rand();
    joint.refvelold = ri.rand();
    joint.output = ri.rand();
  };

  double pgain;
  double dgain;
  pdJoint joint;
  RandomInitializer ri;
};

TEST_F(pdJointPDTrqTest, Create)
{
  EXPECT_TRUE( pdJointCreatePDTrq( &joint, pgain, dgain ) );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
  EXPECT_EQ( pgain, ((_pdJointPDTrq*)joint._prm)->pgain );
  EXPECT_EQ( dgain, ((_pdJointPDTrq*)joint._prm)->dgain );
  EXPECT_EQ( -HUGE_VAL, ((_pdJointPDTrq*)joint._prm)->trqmin );
  EXPECT_EQ(  HUGE_VAL, ((_pdJointPDTrq*)joint._prm)->trqmax );
  EXPECT_TRUE( joint._prm );
  EXPECT_EQ( &pd_joint_pd_trq_met, joint._met );
  pdJointDestroy( &joint );
}

TEST_F(pdJointPDTrqTest, Destroy)
{
  pdJointCreatePDTrq( &joint, pgain, dgain );
  zNameSet( &joint, (char*)ZNONAME );
  SetRandomValues();
  pdJointDestroy( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
}

TEST_F(pdJointPDTrqTest, Update)
{
  pdJointCreatePDTrq( &joint, 2, 3 );
  pdJointRefresh( &joint, 0 );
  pdJointSetDis( &joint, 0 );
  pdJointSetRefDis( &joint, 10 );
  pdJointSetRefDis( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( 20, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVelOld( &joint ) );

  pdJointSetDis( &joint, 5 );
  pdJointSetRefDis( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -1490, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVelOld( &joint ) );

  pdJointSetDis( &joint, 7 );
  pdJointSetRefDis( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -594, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 200, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 5, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 500, pdJointVelOld( &joint ) );
}

TEST_F(pdJointPDTrqTest, Output_Limit)
{
  pdJointCreatePDTrq( &joint, 2, 3 );
  pdJointPDTrqSetLim( &joint, -100, 100 );

  pdJointRefresh( &joint, 0 );
  pdJointSetRefDis( &joint, 200 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( 100, pdJointOutput( &joint ) );

  pdJointRefresh( &joint, 0 );
  pdJointSetRefDis( &joint, -200 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -100, pdJointOutput( &joint ) );
}

TEST_F(pdJointPDTrqTest, SetPgain)
{
  double p;

  pdJointCreatePDTrq( &joint, pgain, dgain );
  p = ri.rand();
  EXPECT_NE( p, ((_pdJointPDTrq*)joint._prm)->pgain );

  pdJointPDTrqSetPgain( &joint, p );
  EXPECT_EQ( p, ((_pdJointPDTrq*)joint._prm)->pgain );
}

TEST_F(pdJointPDTrqTest, SetDgain)
{
  double d;

  pdJointCreatePDTrq( &joint, pgain, dgain );
  d = ri.rand();
  EXPECT_NE( d, ((_pdJointPDTrq*)joint._prm)->dgain );

  pdJointPDTrqSetDgain( &joint, d );
  EXPECT_EQ( d, ((_pdJointPDTrq*)joint._prm)->dgain );
}


typedef struct{
  double pgain, igain, dgain;
  double trqmin, trqmax;
} _pdJointPIDTrq;

class pdJointPIDTrqTest : public testing::Test {
protected:
  virtual void SetUp() {
    pgain = ri.rand();
    igain = ri.rand();
    dgain = ri.rand();
  };
  virtual void TearDown() {};

  void SetRandomValues() {
    joint.dis = ri.rand();
    joint.vel = ri.rand();
    joint.disold = ri.rand();
    joint.velold = ri.rand();
    joint.refdis = ri.rand();
    joint.refvel = ri.rand();
    joint.refdisold = ri.rand();
    joint.refvelold = ri.rand();
    joint.output = ri.rand();
  };

  double pgain;
  double igain;
  double dgain;
  pdJoint joint;
  RandomInitializer ri;
};

TEST_F(pdJointPIDTrqTest, Create)
{
  EXPECT_TRUE( pdJointCreatePIDTrq( &joint, pgain, igain, dgain ) );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
  EXPECT_EQ( pgain, ((_pdJointPIDTrq*)joint._prm)->pgain );
  EXPECT_EQ( igain, ((_pdJointPIDTrq*)joint._prm)->igain );
  EXPECT_EQ( dgain, ((_pdJointPIDTrq*)joint._prm)->dgain );
  EXPECT_EQ( -HUGE_VAL, ((_pdJointPIDTrq*)joint._prm)->trqmin );
  EXPECT_EQ(  HUGE_VAL, ((_pdJointPIDTrq*)joint._prm)->trqmax );
  EXPECT_TRUE( joint._prm );
  EXPECT_EQ( &pd_joint_pid_trq_met, joint._met );
  pdJointDestroy( &joint );
}

TEST_F(pdJointPIDTrqTest, Destroy)
{
  pdJointCreatePIDTrq( &joint, pgain, igain, dgain );
  zNameSet( &joint, (char*)ZNONAME );
  SetRandomValues();
  pdJointDestroy( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
}

TEST_F(pdJointPIDTrqTest, Update)
{
  pdJointCreatePIDTrq( &joint, 2, 4, 3 );
  pdJointRefresh( &joint, 0 );
  pdJointSetDis( &joint, 0 );
  pdJointSetRefDis( &joint, 10 );
  pdJointSetRefDis( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( 20.4, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVelOld( &joint ) );

  pdJointSetDis( &joint, 5 );
  pdJointSetRefDis( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -1489.4, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVelOld( &joint ) );

  pdJointSetDis( &joint, 7 );
  pdJointSetRefDis( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -593.28, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 200, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 5, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 500, pdJointVelOld( &joint ) );
}

TEST_F(pdJointPIDTrqTest, Output_Limit)
{
  pdJointCreatePIDTrq( &joint, 2, 4, 3 );
  pdJointPIDTrqSetLim( &joint, -100, 100 );

  pdJointRefresh( &joint, 0 );
  pdJointSetRefDis( &joint, 200 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( 100, pdJointOutput( &joint ) );

  pdJointRefresh( &joint, 0 );
  pdJointSetRefDis( &joint, -200 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -100, pdJointOutput( &joint ) );
}

TEST_F(pdJointPIDTrqTest, SetPgain)
{
  double p;

  pdJointCreatePIDTrq( &joint, pgain, igain, dgain );
  p = ri.rand();
  EXPECT_NE( p, ((_pdJointPIDTrq*)joint._prm)->pgain );

  pdJointPIDTrqSetPgain( &joint, p );
  EXPECT_EQ( p, ((_pdJointPIDTrq*)joint._prm)->pgain );
}

TEST_F(pdJointPIDTrqTest, SetIgain)
{
  double i;

  pdJointCreatePIDTrq( &joint, pgain, igain, dgain );
  i = ri.rand();
  EXPECT_NE( i, ((_pdJointPIDTrq*)joint._prm)->igain );

  pdJointPIDTrqSetIgain( &joint, i );
  EXPECT_EQ( i, ((_pdJointPIDTrq*)joint._prm)->igain );
}

TEST_F(pdJointPIDTrqTest, SetDgain)
{
  double d;

  pdJointCreatePIDTrq( &joint, pgain, igain, dgain );
  d = ri.rand();
  EXPECT_NE( d, ((_pdJointPIDTrq*)joint._prm)->dgain );

  pdJointPIDTrqSetDgain( &joint, d );
  EXPECT_EQ( d, ((_pdJointPIDTrq*)joint._prm)->dgain );
}
