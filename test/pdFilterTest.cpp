#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_filter.h>

const double TIME_STEP = 0.01;

class pdFilterTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterInit( &filter, TIME_STEP );
  };
  virtual void TearDown() {
    pdFilterDestroy( &filter );
  }

  void SetRandomValues() {
    filter._t = ri.rand();
    filter._dt = ri.rand();
  }

  RandomInitializer ri;
  pdFilter filter;
};

TEST_F(pdFilterTest, Init)
{
  SetRandomValues();
  pdFilterInit( &filter, 0.02 );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  EXPECT_EQ( 0.02, pdFilterTimeStep( &filter ) );
}

TEST_F(pdFilterTest, Destroy)
{
  SetRandomValues();
  pdFilterDestroy( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  EXPECT_EQ( 0, pdFilterTimeStep( &filter ) );
}

TEST_F(pdFilterTest, SetTime)
{
  SetRandomValues();
  pdFilterSetTime( &filter, 0.5 );
  EXPECT_EQ( 0.5, pdFilterTime( &filter ) );
}


TEST_F(pdFilterTest, SetTimeStep)
{
  SetRandomValues();
  pdFilterSetTimeStep( &filter, 0.001 );
  EXPECT_EQ( 0.001, pdFilterTimeStep( &filter ) );
}

// TEST_F(pdFilterTest, )
// {}

