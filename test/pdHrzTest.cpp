#include "gtest/gtest.h"
#include <pedi2/pd_vrt.h>
#include <pedi2/pd_hrz.h>

class pdHrzTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {
    pdHrzDestroy( &hrz );
  };

  pdVrt vrt;
  pdHrz hrz;
};

TEST_F(pdHrzTest, Init)
{
  pdHrz _hrz;

  pdHrzInit( &_hrz );
  EXPECT_EQ( -1, _hrz.dir );
  EXPECT_EQ( NULL, _hrz.prm );
  EXPECT_EQ( NULL, _hrz.com );
  EXPECT_EQ( NULL, _hrz.vrt );
}

TEST_F(pdHrzTest, SetupOutOfRange)
{
  pdVrt _vrt;
  pdHrz _hrz;
  pdHrz *ret;

  ret = pdHrzSetup( &_hrz, &_vrt, -1 );
  EXPECT_EQ( NULL, ret );
  ret = pdHrzSetup( &_hrz, &_vrt, 9 );
  EXPECT_EQ( NULL, ret );
}

TEST_F(pdHrzTest, SetupHrzTan)
{
  pdVrt _vrt;
  pdHrz _hrz;
  pdHrzPrmTan *prm;

  pdHrzSetup( &_hrz, &_vrt, PD_HRZ_TAN );
  prm = (pdHrzPrmTan *)_hrz.prm;
  EXPECT_EQ( PD_HRZ_TAN, _hrz.dir );
  EXPECT_EQ( 0, prm->vd );
  EXPECT_EQ( 0, prm->q1 );
  EXPECT_EQ( 0, prm->q2 );
  EXPECT_EQ( 0, prm->dist );
  EXPECT_EQ( &_vrt, _hrz.vrt );
  EXPECT_EQ( 0, _vrt.z );
}

TEST_F(pdHrzTest, SetupHrzRad)
{
  pdVrt _vrt;
  pdHrz _hrz;
  pdHrzPrmRad *prm;

  pdHrzSetup( &_hrz, &_vrt, PD_HRZ_RAD );
  prm = (pdHrzPrmRad *)_hrz.prm;
  EXPECT_EQ( PD_HRZ_RAD, _hrz.dir );
  EXPECT_EQ( 0, prm->vd );
  EXPECT_EQ( 0, prm->q1 );
  EXPECT_EQ( 0, prm->q2 );
  EXPECT_EQ( 0, prm->kappa );
  EXPECT_EQ( 0, prm->rho );
  EXPECT_EQ( 0, prm->kr );
  EXPECT_EQ( 0, prm->dist );
  EXPECT_EQ( &_vrt, _hrz.vrt );
  EXPECT_EQ( 0, _vrt.z );
}

TEST_F(pdHrzTest, Destroy)
{
  pdVrt _vrt;
  pdHrz _hrz;

  pdHrzSetup( &_hrz, &_vrt, PD_HRZ_TAN );
  pdHrzDestroy( &_hrz );
  EXPECT_EQ( -1, _hrz.dir );
  EXPECT_EQ( NULL, _hrz.prm );
  EXPECT_EQ( NULL, _hrz.com );
  EXPECT_EQ( NULL, _hrz.vrt );
}

// TEST_F(pdHrzTest, SetPrm)
// {
//   pdHrzSetup( &hrz, &vrt, PD_HRZ_TAN );
// }

// TEST_F(pdHrzTest, ComputeK1)
// {
//   hrz.q1 = 1.0;
//   hrz.q2 = 0.5;
//   EXPECT_EQ( 0.5, pdHrzK1( &hrz ) );
//   hrz.q1 = 0.8;
//   hrz.q2 = 1.3;
//   EXPECT_EQ( 0.8*1.3, pdHrzK1( &hrz ) );
// }

// TEST_F(pdHrzTest, ComputeK2)
// {
//   hrz.q1 = 1.0;
//   hrz.q2 = 0.5;
//   EXPECT_EQ( 1.5/vrt.zeta, pdHrzK2( &hrz ) );
//   hrz.q1 = 0.8;
//   hrz.q2 = 1.3;
//   EXPECT_EQ( 2.1/vrt.zeta, pdHrzK2( &hrz ) );
// }
