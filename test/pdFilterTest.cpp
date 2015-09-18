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
    pdFilterNoneInit( &flt, TIME_STEP );
  };
  virtual void TearDown() {
    pdFilterNoneDestroy( &flt );
  };

  void SetRandomValues() {
    pdFilter *base = (pdFilter*)&flt;
    base->_t = ri.rand();
    base->_dt = ri.rand();
    base->_input = ri.rand();
    base->_output = ri.rand();
  };

  void* GetVFPtr(pdFilterVFTableTag tag) {
    return ((pdFilter*)&flt)->vftable[tag];
  };

  RandomInitializer ri;
  pdFilterNone flt;
};

TEST_F(pdFilterNoneTest, Init)
{
  SetRandomValues();
  pdFilterNoneInit( &flt, 0.02 );
  EXPECT_EQ( 0, pdFilterTime( &flt ) );
  EXPECT_EQ( 0.02, pdFilterTimeStep( &flt ) );
  EXPECT_EQ( 0, pdFilterInput( &flt ) );
  EXPECT_EQ( 0, pdFilterOutput( &flt ) );
}

TEST_F(pdFilterNoneTest, SetTime)
{
  SetRandomValues();
  pdFilterSetTime( &flt, 1.0 );
  EXPECT_EQ( 1.0, pdFilterTime( &flt ) );
}

TEST_F(pdFilterNoneTest, SetTimeStep)
{
  SetRandomValues();
  pdFilterSetTimeStep( &flt, 0.001 );
  EXPECT_EQ( 0.001, pdFilterTimeStep( &flt ) );
}

TEST_F(pdFilterNoneTest, Update_Imp)
{
  pdFilterUpdateType *func;

  func = (pdFilterUpdateType*)GetVFPtr(pdFilterUpdateTag);
  EXPECT_EQ( (pdFilterUpdateType*)pdFilterNoneUpdate_Imp, func );
}

TEST_F(pdFilterNoneTest, Alloc)
{
  pdFilterNone *new_flt;

  new_flt = pdFilterNoneAlloc();
  EXPECT_TRUE( new_flt );
  zFree( new_flt );
  EXPECT_FALSE( new_flt );
}

TEST_F(pdFilterNoneTest, Update)
{
  pdFilterSetInput( &flt, 1 );
  pdFilterUpdate( &flt );
  EXPECT_EQ( 1, pdFilterOutput( &flt ) );
  pdFilterSetInput( &flt, 2 );
  pdFilterUpdate( &flt );
  EXPECT_EQ( 2, pdFilterOutput( &flt ) );
  double val = ri.rand();
  pdFilterSetInput( &flt, val );
  pdFilterUpdate( &flt );
  EXPECT_EQ( val, pdFilterOutput( &flt ) );
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
  };

  void SetRandomValues() {
    pdFilter *base = (pdFilter*)&bwf;
    base->_t = ri.rand();
    base->_dt = ri.rand();
    base->_input = ri.rand();
    base->_output = ri.rand();
  };

  void* GetVFPtr(pdFilterVFTableTag tag) {
    return ((pdFilter*)&bwf)->vftable[tag];
  };

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

TEST_F(pdFilterBWTest, SetTime)
{
  SetRandomValues();
  pdFilterSetTime( &bwf, 1.0 );
  EXPECT_EQ( 1.0, pdFilterTime( &bwf ) );
}

TEST_F(pdFilterBWTest, SetTimeStep)
{
  SetRandomValues();
  pdFilterSetTimeStep( &bwf, 0.001 );
  EXPECT_EQ( 0.001, pdFilterTimeStep( &bwf ) );
}

TEST_F(pdFilterBWTest, Update_Imp)
{
  pdFilterUpdateType *func;

  func = (pdFilterUpdateType*)GetVFPtr(pdFilterUpdateTag);
  EXPECT_EQ( (pdFilterUpdateType*)pdFilterBWUpdate_Imp, func );
}

TEST_F(pdFilterBWTest, Alloc)
{
  pdFilterBW *new_bwf;

  new_bwf = pdFilterBWAlloc();
  EXPECT_TRUE( new_bwf );
  zFree( new_bwf );
  EXPECT_FALSE( new_bwf );
}

// TEST_F(pdFilterTest, )
// {}

