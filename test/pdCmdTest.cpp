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

  void Stop() {
    cmd.rho = 0;
    cmd.vud = 0;
    cmd.vwd = 0;
  };

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
  EXPECT_EQ( 0, cmd.lambda );
  EXPECT_EQ( 0, cmd.rho );
  EXPECT_EQ( 0, cmd.kr );
  EXPECT_EQ( 0, cmd.xd );
  EXPECT_EQ( 0, cmd.yd );
  EXPECT_EQ( 0, cmd.zd );
  EXPECT_EQ( 0, cmd.xdd );
  EXPECT_EQ( 0, cmd.ydd );
  EXPECT_EQ( 0, cmd.zdd );
  EXPECT_EQ( 0, cmd.thetad );
  EXPECT_EQ( 0, cmd.vud );
  EXPECT_EQ( 0, cmd.vwd );
  EXPECT_EQ( 0, cmd.dist );
  EXPECT_EQ( 0, cmd.kx );
  EXPECT_EQ( 0, cmd.ky );
  EXPECT_EQ( 0, cmd.kz );
  EXPECT_EQ( 0, cmd.bx );
  EXPECT_EQ( 0, cmd.by );
  EXPECT_EQ( 0, cmd.bz );
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
  EXPECT_EQ( 0, cmd.lambda );
  EXPECT_EQ( 0, cmd.rho );
  EXPECT_EQ( 0, cmd.kr );
  EXPECT_EQ( 0, cmd.xd );
  EXPECT_EQ( 0, cmd.yd );
  EXPECT_EQ( 0, cmd.zd );
  EXPECT_EQ( 0, cmd.xdd );
  EXPECT_EQ( 0, cmd.ydd );
  EXPECT_EQ( 0, cmd.zdd );
  EXPECT_EQ( 0, cmd.thetad );
  EXPECT_EQ( 0, cmd.vud );
  EXPECT_EQ( 0, cmd.vwd );
  EXPECT_EQ( 0, cmd.dist );
  EXPECT_EQ( 0, cmd.kx );
  EXPECT_EQ( 0, cmd.ky );
  EXPECT_EQ( 0, cmd.kz );
  EXPECT_EQ( 0, cmd.bx );
  EXPECT_EQ( 0, cmd.by );
  EXPECT_EQ( 0, cmd.bz );
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

TEST_F(pdCmdTest, TryStop)
{
  Stop();
  EXPECT_TRUE( pdCmdTryStop( &cmd ) );

  Stop();
  cmd.rho = 1;
  EXPECT_FALSE( pdCmdTryStop( &cmd ) );

  Stop();
  cmd.vud = 1;
  EXPECT_FALSE( pdCmdTryStop( &cmd ) );

  Stop();
  cmd.vwd = 1;
  EXPECT_FALSE( pdCmdTryStop( &cmd ) );
}

TEST_F(pdCmdTest, TryStep)
{
  Stop();
  EXPECT_FALSE( pdCmdTryStep( &cmd ) );

  cmd.rho = 1;
  EXPECT_TRUE( pdCmdTryStep( &cmd ) );
}

TEST_F(pdCmdTest, TryWalk)
{
  Stop();
  EXPECT_FALSE( pdCmdTryWalk( &cmd ) );

  cmd.vud = 1;
  EXPECT_TRUE( pdCmdTryWalk( &cmd ) );
}

TEST_F(pdCmdTest, TryWalkSideways)
{
  Stop();
  EXPECT_FALSE( pdCmdTryWalkSideways( &cmd ) );

  cmd.vwd = 1;
  EXPECT_TRUE( pdCmdTryWalkSideways( &cmd ) );
}

TEST_F(pdCmdTest, TryWalkCrabWalk)
{
  Stop();
  EXPECT_FALSE( pdCmdTryWalk( &cmd ) );
  EXPECT_FALSE( pdCmdTryWalkSideways( &cmd ) );
  EXPECT_FALSE( pdCmdTryCrabWalk( &cmd ) );

  cmd.vud = 0;
  cmd.vwd = 1;
  cmd.lambda = 0;
  EXPECT_FALSE( pdCmdTryWalk( &cmd ) );
  EXPECT_TRUE( pdCmdTryWalkSideways( &cmd ) );
  EXPECT_FALSE( pdCmdTryCrabWalk( &cmd ) );

  cmd.vud = 0;
  cmd.vwd = 1;
  cmd.lambda = 0.1;
  EXPECT_FALSE( pdCmdTryWalk( &cmd ) );
  EXPECT_TRUE( pdCmdTryWalkSideways( &cmd ) );
  EXPECT_TRUE( pdCmdTryCrabWalk( &cmd ) );

  cmd.vud = 1;
  cmd.vwd = 1;
  cmd.lambda = 0.1;
  EXPECT_TRUE( pdCmdTryWalk( &cmd ) );
  EXPECT_TRUE( pdCmdTryWalkSideways( &cmd ) );
  EXPECT_FALSE( pdCmdTryCrabWalk( &cmd ) );

  cmd.vud = 0;
  cmd.vwd = 0;
  cmd.lambda = 0.1;
  EXPECT_FALSE( pdCmdTryWalk( &cmd ) );
  EXPECT_FALSE( pdCmdTryWalkSideways( &cmd ) );
  EXPECT_FALSE( pdCmdTryCrabWalk( &cmd ) );
}

TEST_F(pdCmdTest, TryWarp)
{
  Stop();
  EXPECT_FALSE( pdCmdTryWarp( &cmd ) );

  cmd.xdd = 1;
  EXPECT_TRUE( pdCmdTryWarp( &cmd ) );
}
