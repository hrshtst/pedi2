#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_mode.h>

class pdModeTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdModeInit( &mode );
  };
  virtual void TearDown() {
    pdModeDestroy( &mode );
  };

  void SetRandomValues() {
    ri.SetRandBool( mode.standing );
    ri.SetRandBool( mode.trywalk );
    ri.SetRandBool( mode.stepping );
    ri.SetRandBool( mode.walking );
    ri.SetRandBool( mode.sideways );
    ri.SetRandBool( mode.following );
    ri.SetRandBool( mode.braking );
    ri.SetRandBool( mode.rotating );
  };

  RandomInitializer ri;
  pdMode mode;
};

TEST_F(pdModeTest, Init)
{
  SetRandomValues();
  pdModeInit( &mode );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.trywalk );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  EXPECT_FALSE( mode.rotating );
}

TEST_F(pdModeTest, Destroy)
{
  SetRandomValues();
  pdModeDestroy( &mode );
  EXPECT_TRUE( mode.standing );
  EXPECT_FALSE( mode.trywalk );
  EXPECT_FALSE( mode.stepping );
  EXPECT_FALSE( mode.walking );
  EXPECT_FALSE( mode.sideways );
  EXPECT_FALSE( mode.following );
  EXPECT_FALSE( mode.braking );
  EXPECT_FALSE( mode.rotating );
}
