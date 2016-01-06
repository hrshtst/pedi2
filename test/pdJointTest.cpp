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
    joint.offset = (int)ri.rand();
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
  EXPECT_EQ( 0, pdJointOffset( &joint ) );
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
  int offset;

  pdJointInit( &joint );
  SetRandomValues();
  dis = ri.rand();
  offset = pdJointOffset( &joint );
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
  EXPECT_EQ( offset, pdJointOffset( &joint ) );
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
  EXPECT_EQ( 0, pdJointOffset( &joint ) );
}

TEST_F(pdJointTest, SetOffset)
{
  pdJointInit( &joint );
  pdJointSetOffset( &joint, 1 );
  EXPECT_EQ( 1, pdJointOffset( &joint ) );
  pdJointSetOffset( &joint, 2 );
  EXPECT_EQ( 2, pdJointOffset( &joint ) );
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
    joint.offset = (int)ri.rand();
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
  EXPECT_EQ( 0, pdJointOffset( &joint ) );
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
  EXPECT_EQ( 0, pdJointOffset( &joint ) );
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

TEST_F(pdJointPDTrqTest, FRead)
{
  char filename[] = "model/joint_pd.conf";
  _pdJointPDTrq *pd;
  FILE *fp;

  fp = fopen( filename, "r" );
  pdJointFRead( fp, &joint );
  pd = (_pdJointPDTrq*)joint._prm;
  EXPECT_STREQ( "test_joint", zNamePtr( &joint ) );
  EXPECT_EQ( 1000, pd->pgain );
  EXPECT_EQ( 50, pd->dgain );
  EXPECT_EQ( 5000, pd->trqmax );
  EXPECT_EQ( -5000, pd->trqmin );
  EXPECT_EQ( &pd_joint_pd_trq_met, joint._met );
  pdJointDestroy( &joint );
  fclose( fp );
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
    joint.offset = (int)ri.rand();
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
  EXPECT_EQ( 0, pdJointOffset( &joint ) );
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
  EXPECT_EQ( 0, pdJointOffset( &joint ) );
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

TEST_F(pdJointPIDTrqTest, FRead)
{
  char filename[] = "model/joint_pid.conf";
  _pdJointPIDTrq *pid;
  FILE *fp;

  fp = fopen( filename, "r" );
  pdJointFRead( fp, &joint );
  pid = (_pdJointPIDTrq*)joint._prm;
  EXPECT_STREQ( "test_joint_pid", zNamePtr( &joint ) );
  EXPECT_EQ( 200, pid->pgain );
  EXPECT_EQ( 50, pid->igain );
  EXPECT_EQ( 10, pid->dgain );
  EXPECT_EQ( 1000, pid->trqmax );
  EXPECT_EQ( -1000, pid->trqmin );
  EXPECT_EQ( &pd_joint_pid_trq_met, joint._met );
  pdJointDestroy( &joint );
  fclose( fp );
}


class pdJointArrayTest : public testing::Test {
protected:
  virtual void SetUp() {
    pgain = ri.rand();
    igain = ri.rand();
    dgain = ri.rand();
  };
  virtual void TearDown() {};

  void MakeJointArray() {
    pdJointArrayAlloc( &arr, 4 );
    pdJointCreatePDTrq( zArrayElem(&arr,0), pgain, dgain );
    pdJointCreatePDTrq( zArrayElem(&arr,1), pgain, dgain );
    pdJointCreatePDTrq( zArrayElem(&arr,2), pgain, dgain );
    pdJointCreatePDTrq( zArrayElem(&arr,3), pgain, dgain );
    zNameSet( zArrayElem(&arr,0), (char*)"joint01" );
    zNameSet( zArrayElem(&arr,1), (char*)"joint02" );
    zNameSet( zArrayElem(&arr,2), (char*)"joint03" );
    zNameSet( zArrayElem(&arr,3), (char*)"joint04" );
  };

  double pgain, igain, dgain;
  pdJointArray arr;
  RandomInitializer ri;
};

TEST_F(pdJointArrayTest, ArrayAlloc)
{
  ASSERT_TRUE( pdJointArrayAlloc( &arr, 5 ) );
  EXPECT_EQ( 5, zArrayNum( &arr ) );
  for( int i=0; i<(int)zArrayNum(&arr); i++ ){
    EXPECT_EQ( i, pdJointOffset( zArrayElem(&arr,i) ) );
  }
  zArrayFree( &arr );
}

TEST_F(pdJointArrayTest, SetDis)
{
  zVec v;
  int i;

  v = zVecAlloc( 4 );
  for( i=0; i<4; i++ )
    zVecSetElem( v, i, ri.rand() );
  MakeJointArray();
  pdJointArraySetDis( &arr, v );
  for( i=0; i<4; i++ )
    EXPECT_EQ( zVecElem(v,i), pdJointArrayDis(&arr,i) );
  pdJointArrayDestroy( &arr );
  zVecFree( v );
}

TEST_F(pdJointArrayTest, SetVel)
{
  zVec v;
  int i;

  v = zVecAlloc( 4 );
  for( i=0; i<4; i++ )
    zVecSetElem( v, i, ri.rand() );
  MakeJointArray();
  pdJointArraySetVel( &arr, v );
  for( i=0; i<4; i++ )
    EXPECT_EQ( zVecElem(v,i), pdJointArrayVel(&arr,i) );
  pdJointArrayDestroy( &arr );
  zVecFree( v );
}

TEST_F(pdJointArrayTest, SetRefDis)
{
  zVec v;
  int i;

  v = zVecAlloc( 4 );
  for( i=0; i<4; i++ )
    zVecSetElem( v, i, ri.rand() );
  MakeJointArray();
  pdJointArraySetRefDis( &arr, v );
  for( i=0; i<4; i++ )
    EXPECT_EQ( zVecElem(v,i), pdJointArrayRefDis(&arr,i) );
  pdJointArrayDestroy( &arr );
  zVecFree( v );
}

TEST_F(pdJointArrayTest, SetRefVel)
{
  zVec v;
  int i;

  v = zVecAlloc( 4 );
  for( i=0; i<4; i++ )
    zVecSetElem( v, i, ri.rand() );
  MakeJointArray();
  pdJointArraySetRefVel( &arr, v );
  for( i=0; i<4; i++ )
    EXPECT_EQ( zVecElem(v,i), pdJointArrayRefVel(&arr,i) );
  pdJointArrayDestroy( &arr );
  zVecFree( v );
}

TEST_F(pdJointArrayTest, Refresh)
{
  zVec v;
  int i;

  v = zVecAlloc( 4 );
  for( i=0; i<4; i++ )
    zVecSetElem( v, i, ri.rand() );
  MakeJointArray();
  pdJointArrayRefresh( &arr, v );
  for( i=0; i<4; i++ ){
    EXPECT_EQ( zVecElem(v,i), pdJointArrayDis(&arr,i) );
    EXPECT_EQ( zVecElem(v,i), pdJointArrayRefDis(&arr,i) );
  }
  pdJointArrayDestroy( &arr );
  zVecFree( v );
}

TEST_F(pdJointArrayTest, NameFind)
{
  pdJointArrayAlloc( &arr, 2 );
  pdJointCreatePDTrq( zArrayElem(&arr,0), pgain, dgain );
  pdJointCreatePIDTrq( zArrayElem(&arr,1), pgain, igain, dgain );
  zNameSet( zArrayElem(&arr,0), (char*)"joint_pi" );
  zNameSet( zArrayElem(&arr,1), (char*)"joint_pid" );
  EXPECT_EQ( zArrayElem(&arr,0), pdJointArrayNameFind( &arr, "joint_pi" ) );
  EXPECT_EQ( zArrayElem(&arr,1), pdJointArrayNameFind( &arr, "joint_pid" ) );
  pdJointArrayDestroy( &arr );
}

TEST_F(pdJointArrayTest, FRead)
{
  char filename[] = "model/joint.conf";
  FILE *fp;

  fp  = fopen( filename, "r" );
  pdJointArrayFRead( fp, &arr );
  EXPECT_STREQ( "rhip_y",      zNamePtr( zArrayElem(&arr, 0) ) );
  EXPECT_STREQ( "rhip_p",      zNamePtr( zArrayElem(&arr, 1) ) );
  EXPECT_STREQ( "rhip_r",      zNamePtr( zArrayElem(&arr, 2) ) );
  EXPECT_STREQ( "rknee_p",     zNamePtr( zArrayElem(&arr, 3) ) );
  EXPECT_STREQ( "rankle_r",    zNamePtr( zArrayElem(&arr, 4) ) );
  EXPECT_STREQ( "rankle_p",    zNamePtr( zArrayElem(&arr, 5) ) );
  EXPECT_STREQ( "lhip_y",      zNamePtr( zArrayElem(&arr, 6) ) );
  EXPECT_STREQ( "lhip_p",      zNamePtr( zArrayElem(&arr, 7) ) );
  EXPECT_STREQ( "lhip_r",      zNamePtr( zArrayElem(&arr, 8) ) );
  EXPECT_STREQ( "lknee_p",     zNamePtr( zArrayElem(&arr, 9) ) );
  EXPECT_STREQ( "lankle_r",    zNamePtr( zArrayElem(&arr,10) ) );
  EXPECT_STREQ( "lankle_p",    zNamePtr( zArrayElem(&arr,11) ) );
  EXPECT_STREQ( "waist_p",     zNamePtr( zArrayElem(&arr,12) ) );
  EXPECT_STREQ( "waist_r",     zNamePtr( zArrayElem(&arr,13) ) );
  EXPECT_STREQ( "neck_p",      zNamePtr( zArrayElem(&arr,14) ) );
  EXPECT_STREQ( "rscaplae_y",  zNamePtr( zArrayElem(&arr,15) ) );
  EXPECT_STREQ( "rshoulder_p", zNamePtr( zArrayElem(&arr,16) ) );
  EXPECT_STREQ( "rshoulder_r", zNamePtr( zArrayElem(&arr,17) ) );
  EXPECT_STREQ( "rshoulder_y", zNamePtr( zArrayElem(&arr,18) ) );
  EXPECT_STREQ( "relbow_p",    zNamePtr( zArrayElem(&arr,19) ) );
  EXPECT_STREQ( "rwrist_y",    zNamePtr( zArrayElem(&arr,20) ) );
  EXPECT_STREQ( "rwrist_r",    zNamePtr( zArrayElem(&arr,21) ) );
  EXPECT_STREQ( "rwrist_p",    zNamePtr( zArrayElem(&arr,22) ) );
  EXPECT_STREQ( "lscaplae_y",  zNamePtr( zArrayElem(&arr,23) ) );
  EXPECT_STREQ( "lshoulder_p", zNamePtr( zArrayElem(&arr,24) ) );
  EXPECT_STREQ( "lshoulder_r", zNamePtr( zArrayElem(&arr,25) ) );
  EXPECT_STREQ( "lshoulder_y", zNamePtr( zArrayElem(&arr,26) ) );
  EXPECT_STREQ( "lelbow_p",    zNamePtr( zArrayElem(&arr,27) ) );
  EXPECT_STREQ( "lwrist_y",    zNamePtr( zArrayElem(&arr,28) ) );
  EXPECT_STREQ( "lwrist_r",    zNamePtr( zArrayElem(&arr,29) ) );
  EXPECT_STREQ( "lwrist_p",    zNamePtr( zArrayElem(&arr,30) ) );
  pdJointArrayDestroy( &arr );
  fclose( fp );
}

TEST_F(pdJointArrayTest, CreateDefaultIndex)
{
  char modelfile[] = "model/hydra.zkc";
  char conffile[]  = "model/joint.conf";
  rkChain chain;
  pdJointArray joint;
  zIndex index;
  register int i;

  rkChainReadFile( &chain, modelfile );
  pdJointArrayReadFile( &joint, conffile );
  index = pdJointArrayCreateDefaultIndex( &joint, &chain );
  EXPECT_EQ( 31, zArrayNum( index ) );
  for( i=0; i<(int)zArrayNum(index); i++ )
    EXPECT_EQ( 6+i, zIndexElem( index, i ) );
  zIndexFree( index );
  pdJointArrayDestroy( &joint );
  rkChainDestroy( &chain );
}

TEST_F(pdJointArrayTest, CreateDefaultIndex_NoNameErr)
{
  char modelfile[] = "model/hydra.zkc";
  char conffile[]  = "model/joint_noname_err.conf";
  rkChain chain;
  pdJointArray joint;
  zIndex index;

  rkChainReadFile( &chain, modelfile );
  pdJointArrayReadFile( &joint, conffile );
  zEchoOff();
  index = pdJointArrayCreateDefaultIndex( &joint, &chain );
  EXPECT_EQ( NULL, index );
  pdJointArrayDestroy( &joint );
  rkChainDestroy( &chain );
  zEchoOn();
}

TEST_F(pdJointArrayTest, SetDisIndex)
{
  zVec q;
  zIndex idx;

  q = zVecCreateList( 6, ri.rand(), ri.rand(), ri.rand(),
                         ri.rand(), ri.rand(), ri.rand() );
  idx = zIndexCreateList( 4, 0, 1, 3, 5 );
  MakeJointArray();
  pdJointArraySetDisIndex( &arr, idx, q );
  EXPECT_EQ( zVecElem(q,0), pdJointArrayDis(&arr,0) );
  EXPECT_EQ( zVecElem(q,1), pdJointArrayDis(&arr,1) );
  EXPECT_NE( zVecElem(q,2), pdJointArrayDis(&arr,2) );
  EXPECT_EQ( zVecElem(q,3), pdJointArrayDis(&arr,2) );
  EXPECT_NE( zVecElem(q,4), pdJointArrayDis(&arr,3) );
  EXPECT_EQ( zVecElem(q,5), pdJointArrayDis(&arr,3) );
}

TEST_F(pdJointArrayTest, SetVelIndex)
{
  zVec v;
  zIndex idx;

  v = zVecCreateList( 6, ri.rand(), ri.rand(), ri.rand(),
                         ri.rand(), ri.rand(), ri.rand() );
  idx = zIndexCreateList( 4, 0, 1, 3, 5 );
  MakeJointArray();
  pdJointArraySetVelIndex( &arr, idx, v );
  EXPECT_EQ( zVecElem(v,0), pdJointArrayVel(&arr,0) );
  EXPECT_EQ( zVecElem(v,1), pdJointArrayVel(&arr,1) );
  EXPECT_NE( zVecElem(v,2), pdJointArrayVel(&arr,2) );
  EXPECT_EQ( zVecElem(v,3), pdJointArrayVel(&arr,2) );
  EXPECT_NE( zVecElem(v,4), pdJointArrayVel(&arr,3) );
  EXPECT_EQ( zVecElem(v,5), pdJointArrayVel(&arr,3) );
}

TEST_F(pdJointArrayTest, SetRefDisIndex)
{
  zVec q;
  zIndex idx;

  q = zVecCreateList( 6, ri.rand(), ri.rand(), ri.rand(),
                         ri.rand(), ri.rand(), ri.rand() );
  idx = zIndexCreateList( 4, 0, 1, 3, 5 );
  MakeJointArray();
  pdJointArraySetRefDisIndex( &arr, idx, q );
  EXPECT_EQ( zVecElem(q,0), pdJointArrayRefDis(&arr,0) );
  EXPECT_EQ( zVecElem(q,1), pdJointArrayRefDis(&arr,1) );
  EXPECT_NE( zVecElem(q,2), pdJointArrayRefDis(&arr,2) );
  EXPECT_EQ( zVecElem(q,3), pdJointArrayRefDis(&arr,2) );
  EXPECT_NE( zVecElem(q,4), pdJointArrayRefDis(&arr,3) );
  EXPECT_EQ( zVecElem(q,5), pdJointArrayRefDis(&arr,3) );
}

TEST_F(pdJointArrayTest, SetRefVelIndex)
{
  zVec v;
  zIndex idx;

  v = zVecCreateList( 6, ri.rand(), ri.rand(), ri.rand(),
                         ri.rand(), ri.rand(), ri.rand() );
  idx = zIndexCreateList( 4, 0, 1, 3, 5 );
  MakeJointArray();
  pdJointArraySetRefVelIndex( &arr, idx, v );
  EXPECT_EQ( zVecElem(v,0), pdJointArrayRefVel(&arr,0) );
  EXPECT_EQ( zVecElem(v,1), pdJointArrayRefVel(&arr,1) );
  EXPECT_NE( zVecElem(v,2), pdJointArrayRefVel(&arr,2) );
  EXPECT_EQ( zVecElem(v,3), pdJointArrayRefVel(&arr,2) );
  EXPECT_NE( zVecElem(v,4), pdJointArrayRefVel(&arr,3) );
  EXPECT_EQ( zVecElem(v,5), pdJointArrayRefVel(&arr,3) );
}

TEST_F(pdJointArrayTest, RefreshIndex)
{
  zVec q;
  zIndex idx;

  q = zVecCreateList( 6, ri.rand(), ri.rand(), ri.rand(),
                         ri.rand(), ri.rand(), ri.rand() );
  idx = zIndexCreateList( 4, 0, 1, 3, 5 );
  MakeJointArray();
  pdJointArrayRefreshIndex( &arr, idx, q );
  EXPECT_EQ( zVecElem(q,0), pdJointArrayDis(&arr,0) );
  EXPECT_EQ( zVecElem(q,1), pdJointArrayDis(&arr,1) );
  EXPECT_NE( zVecElem(q,2), pdJointArrayDis(&arr,2) );
  EXPECT_EQ( zVecElem(q,3), pdJointArrayDis(&arr,2) );
  EXPECT_NE( zVecElem(q,4), pdJointArrayDis(&arr,3) );
  EXPECT_EQ( zVecElem(q,5), pdJointArrayDis(&arr,3) );

  EXPECT_EQ( zVecElem(q,0), pdJointArrayRefDis(&arr,0) );
  EXPECT_EQ( zVecElem(q,1), pdJointArrayRefDis(&arr,1) );
  EXPECT_NE( zVecElem(q,2), pdJointArrayRefDis(&arr,2) );
  EXPECT_EQ( zVecElem(q,3), pdJointArrayRefDis(&arr,2) );
  EXPECT_NE( zVecElem(q,4), pdJointArrayRefDis(&arr,3) );
  EXPECT_EQ( zVecElem(q,5), pdJointArrayRefDis(&arr,3) );
}
