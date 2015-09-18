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
  };

  void SetRandomValues() {
    filter._t = ri.rand();
    filter._dt = ri.rand();
    filter._input = ri.rand();
    filter._output = ri.rand();
  };

  void* GetVFPtr(pdFilterVFTableTag tag) {
    return (&filter)->vftable[tag];
  };

  RandomInitializer ri;
  pdFilter filter;
};

TEST_F(pdFilterTest, SetTime_Imp)
{
  pdFilterSetTimeType *func;

  func = (pdFilterSetTimeType*)GetVFPtr(pdFilterSetTimeTag);
  EXPECT_EQ( pdFilterSetTime_Imp, func );
}

TEST_F(pdFilterTest, SetTimeStep_Imp)
{
  pdFilterSetTimeStepType *func;

  func = (pdFilterSetTimeStepType*)GetVFPtr(pdFilterSetTimeStepTag);
  EXPECT_EQ( pdFilterSetTimeStep_Imp, func );
}

TEST_F(pdFilterTest, SetInput_Imp)
{
  pdFilterSetInputType *func;

  func = (pdFilterSetInputType*)GetVFPtr(pdFilterSetInputTag);
  EXPECT_EQ( pdFilterSetInput_Imp, func );
}

TEST_F(pdFilterTest, Update_Imp)
{
  pdFilterUpdateType *func;

  func = (pdFilterUpdateType*)GetVFPtr(pdFilterUpdateTag);
  EXPECT_EQ( pdFilterUpdate_Imp, func );
}

TEST_F(pdFilterTest, Init)
{
  SetRandomValues();
  pdFilterInit( &filter, 0.02 );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  EXPECT_EQ( 0.02, pdFilterTimeStep( &filter ) );
  EXPECT_EQ( 0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0, pdFilterOutput( &filter ) );
}

TEST_F(pdFilterTest, Destroy)
{
  SetRandomValues();
  pdFilterDestroy( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  EXPECT_EQ( 0, pdFilterTimeStep( &filter ) );
  EXPECT_EQ( 0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0, pdFilterOutput( &filter ) );
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

TEST_F(pdFilterTest, ResetTime)
{
  SetRandomValues();
  EXPECT_NE( 0, pdFilterTime( &filter ) );
  pdFilterResetTime( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
}

TEST_F(pdFilterTest, SetInput)
{
  SetRandomValues();
  pdFilterSetInput( &filter, 1 );
  EXPECT_EQ( 1, pdFilterInput( &filter ) );
  pdFilterSetInput( &filter, 2 );
  EXPECT_EQ( 2, pdFilterInput( &filter ) );
}

class pdFilterNoneTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterNoneInit( &filter, TIME_STEP );
  };
  virtual void TearDown() {
    pdFilterNoneDestroy( &filter );
  };

  void SetRandomValues() {
    pdFilter *base = (pdFilter*)&filter;
    base->_t = ri.rand();
    base->_dt = ri.rand();
    base->_input = ri.rand();
    base->_output = ri.rand();
  };

  void* GetVFPtr(pdFilterVFTableTag tag) {
    return ((pdFilter*)&filter)->vftable[tag];
  };

  RandomInitializer ri;
  pdFilterNone filter;
};

TEST_F(pdFilterNoneTest, Init)
{
  SetRandomValues();
  pdFilterNoneInit( &filter, 0.02 );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  EXPECT_EQ( 0.02, pdFilterTimeStep( &filter ) );
  EXPECT_EQ( 0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0, pdFilterOutput( &filter ) );
}

TEST_F(pdFilterNoneTest, SetTime)
{
  SetRandomValues();
  pdFilterSetTime( &filter, 0.5 );
  EXPECT_EQ( 0.5, pdFilterTime( &filter ) );
}

TEST_F(pdFilterNoneTest, ResetTime)
{
  SetRandomValues();
  EXPECT_NE( 0, pdFilterTime( &filter ) );
  pdFilterResetTime( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
}

TEST_F(pdFilterNoneTest, SetTimeStep)
{
  SetRandomValues();
  pdFilterSetTimeStep( &filter, 0.001 );
  EXPECT_EQ( 0.001, pdFilterTimeStep( &filter ) );
}

TEST_F(pdFilterNoneTest, SetInput)
{
  SetRandomValues();
  pdFilterSetInput( &filter, 1 );
  EXPECT_EQ( 1, pdFilterInput( &filter ) );
  pdFilterSetInput( &filter, 2 );
  EXPECT_EQ( 2, pdFilterInput( &filter ) );
}

TEST_F(pdFilterNoneTest, Update_Imp)
{
  pdFilterUpdateType *func;

  func = (pdFilterUpdateType*)GetVFPtr(pdFilterUpdateTag);
  EXPECT_EQ( (pdFilterUpdateType*)pdFilterNoneUpdate_Imp, func );
}

TEST_F(pdFilterNoneTest, Alloc)
{
  pdFilterNone *new_filter;

  new_filter = pdFilterNoneAlloc();
  EXPECT_TRUE( new_filter );
  zFree( new_filter );
  EXPECT_FALSE( new_filter );
}

TEST_F(pdFilterNoneTest, Update)
{
  pdFilterSetInput( &filter, 1 );
  pdFilterUpdate( &filter );
  EXPECT_EQ( 1, pdFilterOutput( &filter ) );
  pdFilterSetInput( &filter, 2 );
  pdFilterUpdate( &filter );
  EXPECT_EQ( 2, pdFilterOutput( &filter ) );
  double val = ri.rand();
  pdFilterSetInput( &filter, val );
  pdFilterUpdate( &filter );
  EXPECT_EQ( val, pdFilterOutput( &filter ) );
}

const double BWF_CF = 50;
const int BWF_DIM = 2;

class pdFilterBWTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterBWInit( &filter, TIME_STEP, BWF_CF, BWF_DIM );
  };
  virtual void TearDown() {
    pdFilterBWDestroy( &filter );
  };

  void SetRandomValues() {
    pdFilter *base = (pdFilter*)&filter;
    base->_t = ri.rand();
    base->_dt = ri.rand();
    base->_input = ri.rand();
    base->_output = ri.rand();
  };

  void* GetVFPtr(pdFilterVFTableTag tag) {
    return ((pdFilter*)&filter)->vftable[tag];
  };

  RandomInitializer ri;
  pdFilterBW filter;
};

TEST_F(pdFilterBWTest, Init)
{
  SetRandomValues();
  pdFilterBWInit( &filter, 0.02, 50, 2 );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  EXPECT_EQ( 0.02, pdFilterTimeStep( &filter ) );
  EXPECT_EQ( 0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0, pdFilterOutput( &filter ) );
}

TEST_F(pdFilterBWTest, SetTime)
{
  SetRandomValues();
  pdFilterSetTime( &filter, 0.5 );
  EXPECT_EQ( 0.5, pdFilterTime( &filter ) );
}

TEST_F(pdFilterBWTest, ResetTime)
{
  SetRandomValues();
  EXPECT_NE( 0, pdFilterTime( &filter ) );
  pdFilterResetTime( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
}

TEST_F(pdFilterBWTest, SetTimeStep)
{
  SetRandomValues();
  pdFilterSetTimeStep( &filter, 0.001 );
  EXPECT_EQ( 0.001, pdFilterTimeStep( &filter ) );
}

TEST_F(pdFilterBWTest, SetInput)
{
  SetRandomValues();
  pdFilterSetInput( &filter, 1 );
  EXPECT_EQ( 1, pdFilterInput( &filter ) );
  pdFilterSetInput( &filter, 2 );
  EXPECT_EQ( 2, pdFilterInput( &filter ) );
}

TEST_F(pdFilterBWTest, Update_Imp)
{
  pdFilterUpdateType *func;

  func = (pdFilterUpdateType*)GetVFPtr(pdFilterUpdateTag);
  EXPECT_EQ( (pdFilterUpdateType*)pdFilterBWUpdate_Imp, func );
}

TEST_F(pdFilterBWTest, Alloc)
{
  pdFilterBW *new_filter;

  new_filter = pdFilterBWAlloc();
  EXPECT_TRUE( new_filter );
  zFree( new_filter );
  EXPECT_FALSE( new_filter );
}

// TEST_F(pdFilterTest, )
// {}

