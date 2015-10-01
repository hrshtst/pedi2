#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_joint.h>

const double TIME_STEP = 0.01;

class pdJointTest : public testing::Test {
protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdJoint joint;
  RandomInitializer ri;
};

TEST_F(pdJointTest, Init)
{
  pdJointInit( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
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

  pdJointSetVelDefault( &joint, vel2 );
  EXPECT_EQ( vel2, pdJointVel( &joint ) );
  EXPECT_EQ( vel1, pdJointVelOld( &joint ) );
}

TEST_F(pdJointTest, SetRefDefault)
{
  double refdis;

  pdJointInit( &joint );
  refdis = ri.rand();
  pdJointSetRefDefault( &joint, refdis );
  EXPECT_EQ( refdis, pdJointRefDis( &joint ) );
}

TEST_F(pdJointTest, RefreshDefault)
{
  double dis;

  pdJointInit( &joint );
  dis = ri.rand();
  pdJointRefreshDefault( &joint, dis );
  EXPECT_EQ( dis, pdJointDis( &joint ) );
  EXPECT_EQ( dis, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0, pdJointVel( &joint ) );
  EXPECT_EQ( 0, pdJointVelOld( &joint ) );
}

TEST_F(pdJointTest, DestroyDefault)
{
  pdJointInit( &joint );
  zNameSet( &joint, (char*)ZNONAME );
  pdJointDis( &joint ) = ri.rand();
  pdJointVel( &joint ) = ri.rand();
  pdJointDisOld( &joint ) = ri.rand();
  pdJointVelOld( &joint ) = ri.rand();
  pdJointRefDis( &joint ) = ri.rand();
  pdJointOutput( &joint ) = ri.rand();
  // check
  EXPECT_TRUE( zNamePtr(&joint) );
  EXPECT_NE( 0.0, pdJointDis( &joint ) );
  EXPECT_NE( 0.0, pdJointVel( &joint ) );
  EXPECT_NE( 0.0, pdJointDisOld( &joint ) );
  EXPECT_NE( 0.0, pdJointVelOld( &joint ) );
  EXPECT_NE( 0.0, pdJointRefDis( &joint ) );
  EXPECT_NE( 0.0, pdJointOutput( &joint ) );
  // test
  pdJointDestroyDefault( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
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
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
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
  pdJointDis( &joint ) = ri.rand();
  pdJointVel( &joint ) = ri.rand();
  pdJointDisOld( &joint ) = ri.rand();
  pdJointVelOld( &joint ) = ri.rand();
  pdJointRefDis( &joint ) = ri.rand();
  pdJointOutput( &joint ) = ri.rand();
  pdJointDestroy( &joint );
  EXPECT_EQ( NULL, zNamePtr( &joint ) );
  EXPECT_EQ( 0.0, pdJointDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointVel( &joint ) );
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
}

TEST_F(pdJointPDTrqTest, Update)
{
  pdJointCreatePDTrq( &joint, 2, 3 );
  pdJointRefresh( &joint, 0 );
  pdJointSetRef( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( 20, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVelOld( &joint ) );

  pdJointSetDis( &joint, 5 );
  pdJointSetRef( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -1490, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 0, pdJointVelOld( &joint ) );

  pdJointSetDis( &joint, 7 );
  pdJointSetRef( &joint, 10 );
  pdJointUpdate( &joint, 0.01 );
  EXPECT_DOUBLE_EQ( -594, pdJointOutput( &joint ) );
  EXPECT_DOUBLE_EQ( 200, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 5, pdJointDisOld( &joint ) );
  EXPECT_DOUBLE_EQ( 500, pdJointVelOld( &joint ) );
}
