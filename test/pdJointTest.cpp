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
  double dis;

  pdJointInit( &joint );
  dis = ri.rand();
  pdJointSetDisDefault( &joint, dis );
  EXPECT_EQ( dis, pdJointDis( &joint ) );
}

TEST_F(pdJointTest, SetVelDefault)
{
  double vel;

  pdJointInit( &joint );
  vel = ri.rand();
  pdJointSetVelDefault( &joint, vel );
  EXPECT_EQ( vel, pdJointVel( &joint ) );
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
  EXPECT_EQ( 0, pdJointVel( &joint ) );
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
  EXPECT_EQ( 0.0, pdJointDisOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointVelOld( &joint ) );
  EXPECT_EQ( 0.0, pdJointRefDis( &joint ) );
  EXPECT_EQ( 0.0, pdJointOutput( &joint ) );
  EXPECT_EQ( NULL, joint._prm );
  EXPECT_EQ( NULL, joint._met );
}
