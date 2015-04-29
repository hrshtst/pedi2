#include "gtest/gtest.h"
#include <pedi2/pd_foot_uw.h>

class pdFootUWTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzUWInit( &czuw, &vrt );
    pdFootUWInit( &lf, &czuw, PD_FOOT_LEFT );
    pdFootUWInit( &rf, &czuw, PD_FOOT_RIGHT );
  };
  virtual void TearDown() {
    pdFootUWDestroy( &lf );
    pdFootUWDestroy( &rf );
    pdCZHrzUWDestroy( &czuw );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdFootUWCZPtr( &lf ) = NULL;
    pdFootUWSign( &lf ) = 0.1;
    pdFootUWPhi( &lf ) = 1;
    pdFootUWRegZMPU( &lf ) = 2;
    pdFootUWRegZMPW( &lf ) = 3;
    pdFootUWRegZMPU( &lf ) = 4;
    pdFootUWRegZMPW( &lf ) = 5;

    pdFootUWCZPtr( &rf ) = NULL;
    pdFootUWSign( &rf ) = 0.1;
    pdFootUWPhi( &rf ) = 1;
    pdFootUWRegZMPU( &rf ) = 2;
    pdFootUWRegZMPW( &rf ) = 3;
    pdFootUWRegZMPU( &rf ) = 4;
    pdFootUWRegZMPW( &rf ) = 5;
}

  pdCZVrt vrt;
  pdCZHrzUW czuw;
  pdFootUW lf, rf;
};

TEST_F(pdFootUWTest, Init)
{
  SetVacuousPrm();
  pdFootUWInit( &lf, &czuw, PD_FOOT_LEFT );
  EXPECT_EQ( &czuw, pdFootUWCZPtr( &lf ) );
  EXPECT_EQ( 1, pdFootUWSign( &lf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &lf ) );

  pdFootUWInit( &rf, &czuw, PD_FOOT_RIGHT );
  EXPECT_EQ( &czuw, pdFootUWCZPtr( &rf ) );
  EXPECT_EQ( -1, pdFootUWSign( &rf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &rf ) );
}

TEST_F(pdFootUWTest, Destroy)
{
  SetVacuousPrm();
  pdFootUWDestroy( &lf );
  EXPECT_EQ( NULL, pdFootUWCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootUWSign( &lf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &lf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &lf ) );

  pdFootUWDestroy( &rf );
  EXPECT_EQ( NULL, pdFootUWCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootUWSign( &rf ) );
  EXPECT_EQ( 0, pdFootUWPhi( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRegZMPW( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosU( &rf ) );
  EXPECT_EQ( 0, pdFootUWRefPosW( &rf ) );
}

TEST_F(pdFootUWTest, ReferKappa)
{
  EXPECT_EQ( 0, pdFootUWKappa( &lf ) );
  pdCZHrzUWSetKappa( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWKappa( &lf ) );
  pdCZHrzUWSetKappa( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWKappa( &lf ) );
}

TEST_F(pdFootUWTest, ReferDist)
{
  EXPECT_EQ( 0, pdFootUWDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 1 );
  EXPECT_EQ( 1, pdFootUWDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootUWDist( &lf ) );
}
