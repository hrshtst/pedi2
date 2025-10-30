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
  EXPECT_EQ( NULL, joint.prp );
  EXPECT_EQ( NULL, joint.com );
}

TEST_F(pdJointTest, SetDisDefault)
{
  double dis1, dis2;

  pdJointInit( &joint );
  dis1 = ri.rand();
  pdJointDefaultSetDis( &joint, dis1 );
  EXPECT_EQ( dis1, pdJointDis( &joint ) );

  dis2 = ri.rand();
  pdJointDefaultSetDis( &joint, dis2 );
  EXPECT_EQ( dis2, pdJointDis( &joint ) );
  EXPECT_EQ( dis1, pdJointDisOld( &joint ) );
}

TEST_F(pdJointTest, SetVelDefault)
{
  double vel1, vel2;

  pdJointInit( &joint );
  vel1 = ri.rand();
  pdJointDefaultSetVel( &joint, vel1 );
  EXPECT_EQ( vel1, pdJointVel( &joint ) );
  EXPECT_TRUE( joint.is_set_vel );

  vel2 = ri.rand();
  pdJointDefaultSetVel( &joint, vel2 );
  EXPECT_EQ( vel2, pdJointVel( &joint ) );
  EXPECT_EQ( vel1, pdJointVelOld( &joint ) );
  EXPECT_TRUE( joint.is_set_vel );
}

TEST_F(pdJointTest, SetRefDisDefault)
{
  double refdis1, refdis2;

  pdJointInit( &joint );
  refdis1 = ri.rand();
  pdJointDefaultSetRefDis( &joint, refdis1 );
  EXPECT_EQ( refdis1, pdJointRefDis( &joint ) );

  refdis2 = ri.rand();
  pdJointDefaultSetRefDis( &joint, refdis2 );
  EXPECT_EQ( refdis2, pdJointRefDis( &joint ) );
  EXPECT_EQ( refdis1, pdJointRefDisOld( &joint ) );
}

TEST_F(pdJointTest, SetRefVelDefault)
{
  double refvel1, refvel2;

  pdJointInit( &joint );
  refvel1 = ri.rand();
  pdJointDefaultSetRefVel( &joint, refvel1 );
  EXPECT_EQ( refvel1, pdJointRefVel( &joint ) );
  EXPECT_TRUE( joint.is_set_refvel );

  refvel2 = ri.rand();
  pdJointDefaultSetRefVel( &joint, refvel2 );
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
  pdJointDefaultRefresh( &joint, dis );
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
  pdJointDefaultRefresh( &joint, 10 );

  pdJointDefaultSetRefDis( &joint, 20 );
  pdJointDefaultSetDis( &joint, 10 );
  pdJointDefaultUpdate( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );

  pdJointDefaultSetRefDis( &joint, 30 );
  pdJointDefaultSetDis( &joint, 15 );
  pdJointDefaultUpdate( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );
  EXPECT_FALSE( joint.is_set_vel );
  EXPECT_FALSE( joint.is_set_refvel );
}

TEST_F(pdJointTest, UpdateDefault_SetVel)
{
  pdJointInit( &joint );
  pdJointDefaultRefresh( &joint, 10 );

  pdJointDefaultSetRefDis( &joint, 20 );
  pdJointDefaultSetDis( &joint, 10 );
  pdJointDefaultUpdate( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );

  pdJointDefaultSetRefDis( &joint, 30 );
  pdJointDefaultSetDis( &joint, 15 );
  pdJointDefaultSetVel( &joint, 600 );
  pdJointDefaultUpdate( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 600, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );
}

TEST_F(pdJointTest, UpdateDefault_SetRefVel)
{
  pdJointInit( &joint );
  pdJointDefaultRefresh( &joint, 10 );

  pdJointDefaultSetRefDis( &joint, 20 );
  pdJointDefaultSetDis( &joint, 10 );
  pdJointDefaultUpdate( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 1000, pdJointRefVel( &joint ) );

  pdJointDefaultSetRefDis( &joint, 30 );
  pdJointDefaultSetDis( &joint, 15 );
  pdJointDefaultSetRefVel( &joint, 900 );
  pdJointDefaultUpdate( &joint, TIME_STEP );
  EXPECT_DOUBLE_EQ( 500, pdJointVel( &joint ) );
  EXPECT_DOUBLE_EQ( 900, pdJointRefVel( &joint ) );
}

TEST_F(pdJointTest, DestroyDefault)
{
  pdJointInit( &joint );
  zNameSet( &joint, ZNONAME );
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
  pdJointDefaultDestroy( &joint );
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
  EXPECT_EQ( NULL, joint.prp );
  EXPECT_EQ( NULL, joint.com );
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
  EXPECT_TRUE( pdJointPDTrqCreate( &joint, pgain, dgain ) );
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
  EXPECT_EQ( pgain, ((_pdJointPDTrq*)joint.prp)->pgain );
  EXPECT_EQ( dgain, ((_pdJointPDTrq*)joint.prp)->dgain );
  EXPECT_EQ( -HUGE_VAL, ((_pdJointPDTrq*)joint.prp)->trqmin );
  EXPECT_EQ(  HUGE_VAL, ((_pdJointPDTrq*)joint.prp)->trqmax );
  EXPECT_TRUE( joint.prp );
  EXPECT_EQ( &pd_joint_pd_trq_com, joint.com );
  pdJointDestroy( &joint );
}

TEST_F(pdJointPDTrqTest, Destroy)
{
  pdJointPDTrqCreate( &joint, pgain, dgain );
  zNameSet( &joint, ZNONAME );
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
  EXPECT_EQ( NULL, joint.prp );
  EXPECT_EQ( NULL, joint.com );
}

TEST_F(pdJointPDTrqTest, Update)
{
  pdJointPDTrqCreate( &joint, 2, 3 );
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
  pdJointPDTrqCreate( &joint, 2, 3 );
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

  pdJointPDTrqCreate( &joint, pgain, dgain );
  p = ri.rand();
  EXPECT_NE( p, ((_pdJointPDTrq*)joint.prp)->pgain );

  pdJointPDTrqSetPgain( &joint, p );
  EXPECT_EQ( p, ((_pdJointPDTrq*)joint.prp)->pgain );
}

TEST_F(pdJointPDTrqTest, SetDgain)
{
  double d;

  pdJointPDTrqCreate( &joint, pgain, dgain );
  d = ri.rand();
  EXPECT_NE( d, ((_pdJointPDTrq*)joint.prp)->dgain );

  pdJointPDTrqSetDgain( &joint, d );
  EXPECT_EQ( d, ((_pdJointPDTrq*)joint.prp)->dgain );
}

TEST_F(pdJointPDTrqTest, FromZTK)
{
  char filename[] = "model/joint_pd.ztk";
  _pdJointPDTrq *pd;
  ZTK ztk;

  ZTKParse( &ztk, filename );
  pdJointFromZTK( &joint, &ztk );
  pd = (_pdJointPDTrq*)joint.prp;
  EXPECT_STREQ( "test_joint", zNamePtr( &joint ) );
  EXPECT_EQ( 1000, pd->pgain );
  EXPECT_EQ( 50, pd->dgain );
  EXPECT_EQ( 5000, pd->trqmax );
  EXPECT_EQ( -5000, pd->trqmin );
  EXPECT_EQ( &pd_joint_pd_trq_com, joint.com );
  pdJointDestroy( &joint );
  ZTKDestroy( &ztk );
}

TEST_F(pdJointPDTrqTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdJointPDTrqCreate( &joint, 1000, 50 );
  pdJointPDTrqSetLim( &joint, -5000, 5000 );
  zNameSet( &joint, "test_joint" );
  sprintf( expected,
           "name: %s\ntype: PDtrq\n"
           "pgain: %.10g\ndgain: %.10g\n"
           "min: %.10g\nmax: %.10g\n",
           zName(&joint), 1000.0, 50.0, -5000.0, 5000.0 );

  pdJointFPrintZTK( fp, &joint );
  fflush( fp );
  EXPECT_STREQ( expected, buf );
  pdJointDestroy( &joint );
  fclose( fp );
}

TEST_F(pdJointPDTrqTest, FPrintZTK_inf)
{
  char buf[2*BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdJointPDTrqCreate( &joint, 2000, 100 );
  zNameSet( &joint, "test_joint" );
  sprintf( expected,
           "name: %s\ntype: PDtrq\n"
           "pgain: %.10g\ndgain: %.10g\n",
           zName(&joint), 2000.0, 100.0 );

  pdJointFPrintZTK( fp, &joint );
  fprintf( fp, "%c", '\0' );
  fflush( fp );
  EXPECT_STREQ( expected, buf );
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
  EXPECT_TRUE( pdJointPIDTrqCreate( &joint, pgain, igain, dgain ) );
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
  EXPECT_EQ( pgain, ((_pdJointPIDTrq*)joint.prp)->pgain );
  EXPECT_EQ( igain, ((_pdJointPIDTrq*)joint.prp)->igain );
  EXPECT_EQ( dgain, ((_pdJointPIDTrq*)joint.prp)->dgain );
  EXPECT_EQ( -HUGE_VAL, ((_pdJointPIDTrq*)joint.prp)->trqmin );
  EXPECT_EQ(  HUGE_VAL, ((_pdJointPIDTrq*)joint.prp)->trqmax );
  EXPECT_TRUE( joint.prp );
  EXPECT_EQ( &pd_joint_pid_trq_com, joint.com );
  pdJointDestroy( &joint );
}

TEST_F(pdJointPIDTrqTest, Destroy)
{
  pdJointPIDTrqCreate( &joint, pgain, igain, dgain );
  zNameSet( &joint, ZNONAME );
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
  EXPECT_EQ( NULL, joint.prp );
  EXPECT_EQ( NULL, joint.com );
}

TEST_F(pdJointPIDTrqTest, Update)
{
  pdJointPIDTrqCreate( &joint, 2, 4, 3 );
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
  pdJointPIDTrqCreate( &joint, 2, 4, 3 );
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

  pdJointPIDTrqCreate( &joint, pgain, igain, dgain );
  p = ri.rand();
  EXPECT_NE( p, ((_pdJointPIDTrq*)joint.prp)->pgain );

  pdJointPIDTrqSetPgain( &joint, p );
  EXPECT_EQ( p, ((_pdJointPIDTrq*)joint.prp)->pgain );
}

TEST_F(pdJointPIDTrqTest, SetIgain)
{
  double i;

  pdJointPIDTrqCreate( &joint, pgain, igain, dgain );
  i = ri.rand();
  EXPECT_NE( i, ((_pdJointPIDTrq*)joint.prp)->igain );

  pdJointPIDTrqSetIgain( &joint, i );
  EXPECT_EQ( i, ((_pdJointPIDTrq*)joint.prp)->igain );
}

TEST_F(pdJointPIDTrqTest, SetDgain)
{
  double d;

  pdJointPIDTrqCreate( &joint, pgain, igain, dgain );
  d = ri.rand();
  EXPECT_NE( d, ((_pdJointPIDTrq*)joint.prp)->dgain );

  pdJointPIDTrqSetDgain( &joint, d );
  EXPECT_EQ( d, ((_pdJointPIDTrq*)joint.prp)->dgain );
}

TEST_F(pdJointPIDTrqTest, FromZTK)
{
  char filename[] = "model/joint_pid.ztk";
  _pdJointPIDTrq *pid;
  ZTK ztk;

  ZTKParse( &ztk, filename );
  pdJointFromZTK( &joint, &ztk );
  pid = (_pdJointPIDTrq*)joint.prp;
  EXPECT_STREQ( "test_joint_pid", zNamePtr( &joint ) );
  EXPECT_EQ( 200, pid->pgain );
  EXPECT_EQ( 50, pid->igain );
  EXPECT_EQ( 10, pid->dgain );
  EXPECT_EQ( 1000, pid->trqmax );
  EXPECT_EQ( -1000, pid->trqmin );
  EXPECT_EQ( &pd_joint_pid_trq_com, joint.com );
  pdJointDestroy( &joint );
  ZTKDestroy( &ztk );
}

TEST_F(pdJointPIDTrqTest, FPrintZTK)
{
  char buf[BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdJointPIDTrqCreate( &joint, 200, 50, 10 );
  pdJointPIDTrqSetLim( &joint, -1000, 1000 );
  zNameSet( &joint, "test_joint_pid" );
  sprintf( expected,
           "name: %s\ntype: PIDtrq\n"
           "pgain: %.10g\nigain: %.10g\ndgain: %.10g\n"
           "min: %.10g\nmax: %.10g\n",
           zName(&joint), 200.0, 50.0, 10.0, -1000.0, 1000.0 );

  pdJointFPrintZTK( fp, &joint );
  fflush( fp );
  EXPECT_STREQ( expected, buf );
  pdJointDestroy( &joint );
  fclose( fp );
}

TEST_F(pdJointPIDTrqTest, FPrintZTK_inf)
{
  char buf[2*BUFSIZ];
  char expected[BUFSIZ];
  FILE *fp;

  fp = fmemopen( buf, sizeof(buf), "r+" );
  pdJointPIDTrqCreate( &joint, 400, 100, 5 );
  zNameSet( &joint, "test_joint_pid" );
  sprintf( expected,
           "name: %s\ntype: PIDtrq\n"
           "pgain: %.10g\nigain: %.10g\ndgain: %.10g\n",
           zName(&joint), 400.0, 100.0, 5.0 );

  pdJointFPrintZTK( fp, &joint );
  fprintf( fp, "%c", '\0' );
  fflush( fp );
  EXPECT_STREQ( expected, buf );
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
    pdJointPDTrqCreate( zArrayElem(&arr,0), pgain, dgain );
    pdJointPDTrqCreate( zArrayElem(&arr,1), pgain, dgain );
    pdJointPDTrqCreate( zArrayElem(&arr,2), pgain, dgain );
    pdJointPDTrqCreate( zArrayElem(&arr,3), pgain, dgain );
    zNameSet( zArrayElem(&arr,0), "joint01" );
    zNameSet( zArrayElem(&arr,1), "joint02" );
    zNameSet( zArrayElem(&arr,2), "joint03" );
    zNameSet( zArrayElem(&arr,3), "joint04" );
  };

  void CheckAllJointNames() {
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
  };

  double pgain, igain, dgain;
  pdJointArray arr;
  RandomInitializer ri;
};

TEST_F(pdJointArrayTest, ArrayAlloc)
{
  ASSERT_TRUE( pdJointArrayAlloc( &arr, 5 ) );
  EXPECT_EQ( 5, zArraySize( &arr ) );
  for( int i=0; i<(int)zArraySize(&arr); i++ ){
    EXPECT_EQ( i, pdJointArrayOffset( &arr, i ) );
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
  pdJointPDTrqCreate( zArrayElem(&arr,0), pgain, dgain );
  pdJointPIDTrqCreate( zArrayElem(&arr,1), pgain, igain, dgain );
  zNameSet( zArrayElem(&arr,0), "joint_pi" );
  zNameSet( zArrayElem(&arr,1), "joint_pid" );
  EXPECT_EQ( zArrayElem(&arr,0), pdJointArrayNameFind( &arr, "joint_pi" ) );
  EXPECT_EQ( zArrayElem(&arr,1), pdJointArrayNameFind( &arr, "joint_pid" ) );
  pdJointArrayDestroy( &arr );
}

TEST_F(pdJointArrayTest, FromZTK)
{
  char filename[] = "model/joint.ztk";
  ZTK ztk;

  ZTKParse( &ztk, filename );
  pdJointArrayFromZTK( &arr, NULL, &ztk );
  CheckAllJointNames();
  pdJointArrayDestroy( &arr );
  ZTKDestroy( &ztk );
}

TEST_F(pdJointArrayTest, FromZTK_CheckOffset)
{
  char modelfile[] = "model/hydra.ztk";
  char jointfile[] = "model/joint.ztk";
  rkChain chain;
  pdJointArray joint;
  ZTK ztk;
  int i;

  rkChainReadZTK( &chain, modelfile );
  ZTKParse( &ztk, jointfile );
  pdJointArrayFromZTK( &joint, &chain, &ztk );
  for( i=0; i<(int)zArraySize(&joint); i++ ){
    EXPECT_EQ( 6+i, pdJointArrayOffset( &joint, i ) );
  }
  pdJointArrayDestroy( &joint );
  rkChainDestroy( &chain );
  ZTKDestroy( &ztk );
}

TEST_F(pdJointArrayTest, FromZTK_NoNameErr)
{
  char modelfile[] = "model/hydra.ztk";
  char jointfile[] = "model/joint_noname_err.ztk";
  rkChain chain;
  pdJointArray joint;

  testing::internal::CaptureStderr();
  rkChainReadZTK( &chain, modelfile );
  pdJointArrayReadZTK( &joint, &chain, jointfile );
  std::string msg = testing::internal::GetCapturedStderr();
  std::string expected = \
    "run-time error: joint neck_y cannot be found in robot model (pdJointArraySetOffsetMapping).\n";
  EXPECT_EQ( expected, msg );
  pdJointArrayDestroy( &joint );
  rkChainDestroy( &chain );
}

TEST_F(pdJointArrayTest, FromZTK_CheckSort)
{
  char modelfile[] = "model/hydra.ztk";
  char jointfile[] = "model/joint_unsorted.ztk";
  rkChain chain;

  rkChainReadZTK( &chain, modelfile );
  pdJointArrayReadZTK( &arr, &chain, jointfile );
  CheckAllJointNames();
  pdJointArrayDestroy( &arr );
  rkChainDestroy( &chain );
}

TEST_F(pdJointArrayTest, CreateIndex)
{
  char modelfile[] = "model/hydra.ztk";
  char jointfile[] = "model/joint.ztk";
  rkChain chain;
  pdJointArray joint;
  zIndex index;
  int i;

  rkChainReadZTK( &chain, modelfile );
  pdJointArrayReadZTK( &joint, &chain, jointfile );
  index = pdJointArrayCreateIndex( &joint );
  EXPECT_EQ( 31, zArraySize( index ) );
  for( i=0; i<(int)zArraySize(index); i++ )
    EXPECT_EQ( 6+i, zIndexElem( index, i ) );
  zIndexFree( index );
  pdJointArrayDestroy( &joint );
  rkChainDestroy( &chain );
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
