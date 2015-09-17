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
    filter._input = ri.rand();
    filter._output = ri.rand();
  }

  RandomInitializer ri;
  pdFilter filter;
};

TEST_F(pdFilterTest, Time_Imp)
{
  pdFilterTimeType *func;

  func = (pdFilterTimeType*)filter.vftable[pdFilterTimeTag];
  EXPECT_EQ( pdFilterTime_Imp, func );
}

TEST_F(pdFilterTest, TimeStep_Imp)
{
  pdFilterTimeStepType *func;

  func = (pdFilterTimeStepType*)filter.vftable[pdFilterTimeStepTag];
  EXPECT_EQ( pdFilterTimeStep_Imp, func );
}

TEST_F(pdFilterTest, Input_Imp)
{
  pdFilterInputType *func;

  func = (pdFilterInputType*)filter.vftable[pdFilterInputTag];
  EXPECT_EQ( pdFilterInput_Imp, func );
}

TEST_F(pdFilterTest, Output_Imp)
{
  pdFilterOutputType *func;

  func = (pdFilterOutputType*)filter.vftable[pdFilterOutputTag];
  EXPECT_EQ( pdFilterOutput_Imp, func );
}

TEST_F(pdFilterTest, SetTime_Imp)
{
  pdFilterSetTimeType *func;

  func = (pdFilterSetTimeType*)filter.vftable[pdFilterSetTimeTag];
  EXPECT_EQ( pdFilterSetTime_Imp, func );
}

TEST_F(pdFilterTest, SetTimeStep_Imp)
{
  pdFilterSetTimeStepType *func;

  func = (pdFilterSetTimeStepType*)filter.vftable[pdFilterSetTimeStepTag];
  EXPECT_EQ( pdFilterSetTimeStep_Imp, func );
}

TEST_F(pdFilterTest, SetInput_Imp)
{
  pdFilterSetInputType *func;

  func = (pdFilterSetInputType*)filter.vftable[pdFilterSetInputTag];
  EXPECT_EQ( pdFilterSetInput_Imp, func );
}

TEST_F(pdFilterTest, Alloc_Imp)
{
  pdFilterAllocType *func;

  func = (pdFilterAllocType*)filter.vftable[pdFilterAllocTag];
  EXPECT_EQ( pdFilterAlloc_Imp, func );
}

TEST_F(pdFilterTest, Update_Imp)
{
  pdFilterUpdateType *func;

  func = (pdFilterUpdateType*)filter.vftable[pdFilterUpdateTag];
  EXPECT_EQ( pdFilterUpdate_Imp, func );
}

TEST_F(pdFilterTest, Free_Imp)
{
  pdFilterFreeType *func;

  func = (pdFilterFreeType*)filter.vftable[pdFilterFreeTag];
  EXPECT_EQ( pdFilterFree_Imp, func );
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

TEST_F(pdFilterTest, SetInput)
{
  SetRandomValues();
  pdFilterSetInput( &filter, 1 );
  EXPECT_EQ( 1, pdFilterInput( &filter ) );
  pdFilterSetInput( &filter, 2 );
  EXPECT_EQ( 2, pdFilterInput( &filter ) );
}

const double BWF_CF = 50;
const int BWF_DIM = 2;

class pdFilterBWTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterBWInit( &bwf, TIME_STEP, BWF_CF, BWF_DIM );
  };
  virtual void TearDown() {
    pdFilterBWDestroy( &bwf );
  }

  void SetRandomValues() {
    pdFilter *base = (pdFilter*)&bwf;
    base->_t = ri.rand();
    base->_dt = ri.rand();
    base->_input = ri.rand();
    base->_output = ri.rand();
  }

  RandomInitializer ri;
  pdFilterBW bwf;
};

TEST_F(pdFilterBWTest, Init)
{
  SetRandomValues();
  pdFilterBWInit( &bwf, 0.02, 50, 2 );
  EXPECT_EQ( 0, pdFilterTime( &bwf ) );
  EXPECT_EQ( 0.02, pdFilterTimeStep( &bwf ) );
  EXPECT_EQ( 0, pdFilterInput( &bwf ) );
  EXPECT_EQ( 0, pdFilterOutput( &bwf ) );
}

// TEST_F(pdFilterTest, )
// {}

