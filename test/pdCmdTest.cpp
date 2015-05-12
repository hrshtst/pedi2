#include "gtest/gtest.h"
#include <pedi2/pd_cmd.h>

class pdCmdTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCmdInit( &cmd );
  };
  virtual void TearDown() {
    pdCmdDestroy( &cmd );
  };

  pdCmd cmd;
};

TEST_F(pdCmdTest, Init)
{
  EXPECT_EQ( 0, cmd.qu1 );
  EXPECT_EQ( 0, cmd.qu2 );
  EXPECT_EQ( 0, cmd.qw1 );
  EXPECT_EQ( 0, cmd.qw2 );
  EXPECT_EQ( 0, cmd.qz1 );
  EXPECT_EQ( 0, cmd.qz2 );
  EXPECT_EQ( 0, cmd.kappa );
  EXPECT_EQ( 0, cmd.rho );
  EXPECT_EQ( 0, cmd.kr );
  EXPECT_EQ( 0, cmd.xd );
  EXPECT_EQ( 0, cmd.yd );
  EXPECT_EQ( 0, cmd.zd );
  EXPECT_EQ( 0, cmd.thetad );
  EXPECT_EQ( 0, cmd.vud );
  EXPECT_EQ( 0, cmd.vwd );
  EXPECT_EQ( 0, cmd.dist );
  EXPECT_EQ( 0, cmd.lfkx );
  EXPECT_EQ( 0, cmd.lfky );
  EXPECT_EQ( 0, cmd.lfkz );
  EXPECT_EQ( 0, cmd.lfcx );
  EXPECT_EQ( 0, cmd.lfcy );
  EXPECT_EQ( 0, cmd.lfcz );
  EXPECT_EQ( 0, cmd.rfkx );
  EXPECT_EQ( 0, cmd.rfky );
  EXPECT_EQ( 0, cmd.rfkz );
  EXPECT_EQ( 0, cmd.rfcx );
  EXPECT_EQ( 0, cmd.rfcy );
  EXPECT_EQ( 0, cmd.rfcz );
  EXPECT_EQ( 0, cmd.lfh );
  EXPECT_EQ( 0, cmd.rfh );
}

TEST_F(pdCmdTest, Destroy)
{
  EXPECT_EQ( 0, cmd.qu1 );
  EXPECT_EQ( 0, cmd.qu2 );
  EXPECT_EQ( 0, cmd.qw1 );
  EXPECT_EQ( 0, cmd.qw2 );
  EXPECT_EQ( 0, cmd.qz1 );
  EXPECT_EQ( 0, cmd.qz2 );
  EXPECT_EQ( 0, cmd.kappa );
  EXPECT_EQ( 0, cmd.rho );
  EXPECT_EQ( 0, cmd.kr );
  EXPECT_EQ( 0, cmd.xd );
  EXPECT_EQ( 0, cmd.yd );
  EXPECT_EQ( 0, cmd.zd );
  EXPECT_EQ( 0, cmd.thetad );
  EXPECT_EQ( 0, cmd.vud );
  EXPECT_EQ( 0, cmd.vwd );
  EXPECT_EQ( 0, cmd.dist );
  EXPECT_EQ( 0, cmd.lfkx );
  EXPECT_EQ( 0, cmd.lfky );
  EXPECT_EQ( 0, cmd.lfkz );
  EXPECT_EQ( 0, cmd.lfcx );
  EXPECT_EQ( 0, cmd.lfcy );
  EXPECT_EQ( 0, cmd.lfcz );
  EXPECT_EQ( 0, cmd.rfkx );
  EXPECT_EQ( 0, cmd.rfky );
  EXPECT_EQ( 0, cmd.rfkz );
  EXPECT_EQ( 0, cmd.rfcx );
  EXPECT_EQ( 0, cmd.rfcy );
  EXPECT_EQ( 0, cmd.rfcz );
  EXPECT_EQ( 0, cmd.lfh );
  EXPECT_EQ( 0, cmd.rfh );
}
