#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_cmd.h>

class pdCmdTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCmdInit( &cmd );
  };
  virtual void TearDown() {
    pdCmdDestroy( &cmd );
  };

  void SetRandomValues() {
    for(int i=0; i<PD_CMD_ENTRY_NUM; ++i)
      cmd.entry[i] = ri.rand();
  }

  RandomInitializer ri;
  pdCmd cmd;
};

TEST_F(pdCmdTest, EntryNum)
{
  int n_entry;

  n_entry = sizeof(pdCmd) / sizeof(double);
  EXPECT_EQ( PD_CMD_ENTRY_NUM, n_entry );
}

TEST_F(pdCmdTest, Init)
{
  SetRandomValues();
  pdCmdInit( &cmd );
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
  EXPECT_EQ( 0, cmd.kx );
  EXPECT_EQ( 0, cmd.ky );
  EXPECT_EQ( 0, cmd.kz );
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
  SetRandomValues();
  pdCmdDestroy( &cmd );
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
  EXPECT_EQ( 0, cmd.kx );
  EXPECT_EQ( 0, cmd.ky );
  EXPECT_EQ( 0, cmd.kz );
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
