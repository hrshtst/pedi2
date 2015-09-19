#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_filter.h>

const double TIME_STEP = 0.01;

class pdFilterTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterTest, Init)
{
  pdFilterInit( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}

TEST_F(pdFilterTest, DestroyDefault)
{
  pdFilterInit( &filter );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroyDefault( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}


class pdFilterNoneTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterNoneTest, Create)
{
  EXPECT_TRUE( pdFilterCreateNone( &filter ) );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( &pd_filter_none_met, filter._met );
}

TEST_F(pdFilterNoneTest, Destroy)
{
  pdFilterCreateNone( &filter );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroy( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}

TEST_F(pdFilterNoneTest, Update)
{
  double input, output;

  pdFilterCreateNone( &filter );
  input = 0.1;
  pdFilterInput( &filter ) = input;
  output = pdFilterUpdate( &filter, TIME_STEP );
  EXPECT_EQ( input, output );
  input = ri.rand();
  pdFilterInput( &filter ) = input;
  output = pdFilterUpdate( &filter, TIME_STEP );
  EXPECT_EQ( input, output );
}


const double BWF_CF = 50;
const int BWF_DIM = 2;
class pdFilterBWTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
  RandomInitializer ri;
};

TEST_F(pdFilterBWTest, Create)
{
  EXPECT_TRUE( pdFilterCreateBW( &filter, BWF_CF, BWF_DIM ) );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_TRUE( filter._prm );
  EXPECT_EQ( &pd_filter_bw_met, filter._met );
}

TEST_F(pdFilterBWTest, Destroy)
{
  pdFilterCreateBW( &filter, BWF_CF, BWF_DIM );
  zNameSet( &filter, (char*)ZNONAME );
  pdFilterInput( &filter ) = ri.rand();
  pdFilterOutput( &filter ) = ri.rand();
  EXPECT_TRUE( zNamePtr(&filter) );
  EXPECT_NE( 0.0, pdFilterInput( &filter ) );
  EXPECT_NE( 0.0, pdFilterOutput( &filter ) );
  pdFilterDestroy( &filter );
  EXPECT_EQ( NULL, zNamePtr( &filter ) );
  EXPECT_EQ( 0.0, pdFilterInput( &filter ) );
  EXPECT_EQ( 0.0, pdFilterOutput( &filter ) );
  EXPECT_EQ( NULL, filter._prm );
  EXPECT_EQ( NULL, filter._met );
}

// TEST_F(pdFilterBWTest, Update)
// {
//   double input, output;

//   pdFilterCreateBW( &filter, BWF_CF, BWF_DIM );
//   input = 0.1;
//   pdFilterInput( &filter ) = input;
//   output = pdFilterUpdate( &filter, TIME_STEP );
//   EXPECT_EQ( input, output );
//   input = ri.rand();
//   pdFilterInput( &filter ) = input;
//   output = pdFilterUpdate( &filter, TIME_STEP );
//   EXPECT_EQ( input, output );
// }






#if 0
class pdFilterTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdFilter filter;
};

TEST_F(pdFilterTest, TypeExpr)
{
  EXPECT_STREQ( "none", pdFilterTypeExpr( 0 ) );
  EXPECT_STREQ( "bw", pdFilterTypeExpr( 1 ) );
  // exceptions
  EXPECT_STREQ( "none", pdFilterTypeExpr( -1 ) );
  EXPECT_STREQ( "bw", pdFilterTypeExpr( 100 ) );
}

TEST_F(pdFilterTest, TypeFromStr)
{
  EXPECT_EQ( PD_FILTER_NONE, pdFilterTypeFromStr( (char*)"none" ) );
  EXPECT_EQ( PD_FILTER_BW, pdFilterTypeFromStr( (char*)"bw" ) );
  EXPECT_EQ( PD_FILTER_NONE, pdFilterTypeFromStr( (char*)"hoge" ) );
}

TEST_F(pdFilterTest, Init)
{
  pdFilterInit( &filter );
  EXPECT_EQ( PD_FILTER_INVALID, filter.type );
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( NULL, filter.com );
}

TEST_F(pdFilterTest, Create)
{
  pdFilterCreate( &filter, PD_FILTER_NONE );
  EXPECT_EQ( PD_FILTER_NONE, filter.type );
  pdFilterCreate( &filter, PD_FILTER_BW );
  EXPECT_EQ( PD_FILTER_BW, filter.type );
  // exceptions
  pdFilter *ret;
  zEchoOff();
  ret = pdFilterCreate( &filter, -1 );
  EXPECT_EQ( NULL, ret );
  ret = pdFilterCreate( &filter, 100 );
  EXPECT_EQ( NULL, ret );
  zEchoOn();
}

TEST_F(pdFilterTest, Destroy)
{
  pdFilterCreate( &filter, PD_FILTER_NONE );
  zNameSet( &filter, (char*)ZNONAME );
  EXPECT_EQ( PD_FILTER_NONE, filter.type );
  pdFilterDestroy( &filter );
  EXPECT_EQ( PD_FILTER_INVALID, filter.type );
  EXPECT_EQ( NULL, filter.prp );
  EXPECT_EQ( NULL, filter.com );
}

TEST_F(pdFilterTest, Clone)
{
  pdFilter src, dst;

  pdFilterCreate( &src, PD_FILTER_NONE );
  pdFilterInit( &dst );
  zNameSet( &src, (char*)ZNONAME );
  pdFilterClone( &src, &dst );
  EXPECT_EQ( PD_FILTER_NONE, src.type );
  pdFilterDestroy( &src );
  pdFilterDestroy( &dst );

  pdFilterCreate( &src, PD_FILTER_BW );
  zNameSet( &src, (char*)ZNONAME );
  pdFilterClone( &src, &dst );
  EXPECT_EQ( PD_FILTER_BW, src.type );
  pdFilterDestroy( &src );
  pdFilterDestroy( &dst );

  // exception
  pdFilter *ret;
  pdFilterCreate( &src, PD_FILTER_NONE );
  pdFilterCreate( &dst, PD_FILTER_BW );
  zNameSet( &src, (char*)ZNONAME );
  zNameSet( &dst, (char*)ZNONAME );
  ret = pdFilterClone( &src, &dst );
  EXPECT_EQ( NULL, ret );
  pdFilterDestroy( &src );
  pdFilterDestroy( &dst );
}


class pdFilterNoneTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdFilterCreate( &none, PD_FILTER_NONE );
    zNameSet( &none, (char*)"none" );
  };
  virtual void TearDown() {
    pdFilterDestroy( &none );
  };

  pdFilter none;
  RandomInitializer ri;
};

TEST_F(pdFilterNoneTest, Init)
{
  EXPECT_EQ( PD_FILTER_NONE, none.type );
  EXPECT_EQ( 0, ((pdFilterPrpNone*)none.prp)->input );
}

TEST_F(pdFilterNoneTest, SetInput)
{
  pdFilterPrpNone *prp;
  double input;

  prp = (pdFilterPrpNone*)none.prp;
  EXPECT_EQ( 0, prp->input );
  input = 0.1;
  pdFilterSetInput( &none, &input );
  EXPECT_EQ( input, prp->input );
  input = ri.rand();
  pdFilterSetInput( &none, &input );
  EXPECT_EQ( input, prp->input );
}

TEST_F(pdFilterNoneTest, Update)
{
  double input, output;

  input = 0.1;
  pdFilterSetInput( &none, &input );
  pdFilterUpdate( &none, &output, TIME_STEP );
  EXPECT_EQ( input, output );
  input = ri.rand();
  pdFilterSetInput( &none, &input );
  pdFilterUpdate( &none, &output, TIME_STEP );
  EXPECT_EQ( input, output );
}

const double BWF_CF = 50;
const int BWF_DIM = 2;
class pdFilterBWTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdFilterCreate( &bwf, PD_FILTER_BW );
    zNameSet( &bwf, (char*)"bwf" );
    pdFilterBWInit( &bwf, BWF_CF, BWF_DIM );
  };
  virtual void TearDown() {
    pdFilterBWDestroy( &bwf );
    pdFilterDestroy( &bwf );
  };

  pdFilter bwf;
  RandomInitializer ri;
};

TEST_F(pdFilterBWTest, Init)
{
  EXPECT_EQ( PD_FILTER_BW, bwf.type );
  EXPECT_EQ( 0, ((pdFilterPrpBW*)bwf.prp)->input );
}

TEST_F(pdFilterBWTest, SetInput)
{
  pdFilterPrpBW *prp;
  double input;

  prp = (pdFilterPrpBW*)bwf.prp;
  EXPECT_EQ( 0, prp->input );
  input = 0.1;
  pdFilterSetInput( &bwf, &input );
  EXPECT_EQ( input, prp->input );
  input = ri.rand();
  pdFilterSetInput( &bwf, &input );
  EXPECT_EQ( input, prp->input );
}

TEST_F(pdFilterBWTest, Init)
{
  
}


// TEST_F(pdFilterBWTest, Update)
// {
//   double input, output;

//   input = 0.1;
//   pdFilterSetInput( &bwf, &input );
//   pdFilterUpdate( &bwf, &output, TIME_STEP );
//   EXPECT_EQ( input, output );
//   input = ri.rand();
//   pdFilterSetInput( &bwf, &input );
//   pdFilterUpdate( &bwf, &output, TIME_STEP );
//   EXPECT_EQ( input, output );
// }

#endif


#if 0

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

TEST_F(pdFilterTest, IncrTime)
{
  pdFilterSetTimeStep( &filter, 0.001 );
  pdFilterResetTime( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  pdFilterIncrTime( &filter );
  EXPECT_EQ( 0.001, pdFilterTime( &filter ) );
  pdFilterIncrTime( &filter );
  EXPECT_EQ( 0.002, pdFilterTime( &filter ) );
}

TEST_F(pdFilterTest, SetInput)
{
  SetRandomValues();
  pdFilterSetInput( &filter, 1 );
  EXPECT_EQ( 1, pdFilterInput( &filter ) );
  pdFilterSetInput( &filter, 2 );
  EXPECT_EQ( 2, pdFilterInput( &filter ) );
}

class pdFilterBWTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdFilterBWInit( &filter, TIME_STEP );
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

TEST_F(pdFilterNoneTest, IncrTime)
{
  pdFilterSetTimeStep( &filter, 0.001 );
  pdFilterResetTime( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  pdFilterIncrTime( &filter );
  EXPECT_EQ( 0.001, pdFilterTime( &filter ) );
  pdFilterIncrTime( &filter );
  EXPECT_EQ( 0.002, pdFilterTime( &filter ) );
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

TEST_F(pdFilterNoneTest, Update_IncrTime)
{
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  pdFilterUpdate( &filter );
  EXPECT_EQ( TIME_STEP, pdFilterTime( &filter ) );
  pdFilterUpdate( &filter );
  EXPECT_EQ( 2*TIME_STEP, pdFilterTime( &filter ) );
  pdFilterUpdate( &filter );
  EXPECT_EQ( 3*TIME_STEP, pdFilterTime( &filter ) );
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

TEST_F(pdFilterBWTest, IncrTime)
{
  pdFilterSetTimeStep( &filter, 0.001 );
  pdFilterResetTime( &filter );
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  pdFilterIncrTime( &filter );
  EXPECT_EQ( 0.001, pdFilterTime( &filter ) );
  pdFilterIncrTime( &filter );
  EXPECT_EQ( 0.002, pdFilterTime( &filter ) );
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

TEST_F(pdFilterBWTest, Update_IncrTime)
{
  EXPECT_EQ( 0, pdFilterTime( &filter ) );
  pdFilterUpdate( &filter );
  EXPECT_EQ( TIME_STEP, pdFilterTime( &filter ) );
  pdFilterUpdate( &filter );
  EXPECT_EQ( 2*TIME_STEP, pdFilterTime( &filter ) );
  pdFilterUpdate( &filter );
  EXPECT_EQ( 3*TIME_STEP, pdFilterTime( &filter ) );
}

#endif

// TEST_F(pdFilterTest, )
// {}

