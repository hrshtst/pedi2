#include "gtest/gtest.h"
#include <pedi2/pd_cz_vrt.h>

class pdCZVrtTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
  };

  virtual void TearDown() {
    pdCZVrtDestroy( &vrt );
  };

  void SetRandPrm() {
    pdCZVrtSetRef( &vrt, 1 );
    pdCZVrtSetQ1( &vrt, 2 );
    pdCZVrtSetQ2( &vrt, 3 );
    vrt.zeta = 4;
  };

  pdCZVrt vrt;
};

TEST_F(pdCZVrtTest, Init)
{
  SetRandPrm();
  pdCZVrtInit( &vrt );
  EXPECT_EQ( 0, pdCZVrtRef( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtQ1( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtQ2( &vrt ) );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, Destroy)
{
  SetRandPrm();
  pdCZVrtDestroy( &vrt );
  EXPECT_EQ( 0, pdCZVrtRef( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtQ1( &vrt ) );
  EXPECT_EQ( 0, pdCZVrtQ2( &vrt ) );
  EXPECT_EQ( 0, vrt.zeta );
}

TEST_F(pdCZVrtTest, SetRef)
{
  pdCZVrtSetRef( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtRef( &vrt ) );
  pdCZVrtSetRef( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtRef( &vrt ) );
}

TEST_F(pdCZVrtTest, SetQ1)
{
  pdCZVrtSetQ1( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtQ1( &vrt ) );
  pdCZVrtSetQ1( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtQ1( &vrt ) );
}

TEST_F(pdCZVrtTest, SetQ2)
{
  pdCZVrtSetQ2( &vrt, 1 );
  EXPECT_EQ( 1, pdCZVrtQ2( &vrt ) );
  pdCZVrtSetQ2( &vrt, 10 );
  EXPECT_EQ( 10, pdCZVrtQ2( &vrt ) );
}

TEST_F(pdCZVrtTest, SetPrm)
{
  pdCZVrtSetPrm( &vrt, 0.26, 1.0, 0.5 );
  EXPECT_EQ( 0.26, pdCZVrtRef( &vrt ) );
  EXPECT_EQ( 1.0, pdCZVrtQ1( &vrt ) );
  EXPECT_EQ( 0.5, pdCZVrtQ2( &vrt ) );
}

TEST_F(pdCZVrtTest, CalcReactForce)
{
  pdCZVrtSetPrm( &vrt, 0.2, 1.0, 1.0 );
  EXPECT_DOUBLE_EQ( 9.806652, pdCZVrtCalcReactForce( &vrt, 0.2, 0 ) );
  EXPECT_DOUBLE_EQ( 14.709978, pdCZVrtCalcReactForce( &vrt, 0.1, 0 ) );
  EXPECT_DOUBLE_EQ( 13.309502937744337, pdCZVrtCalcReactForce( &vrt, 0.1, 0.1 ) );
  EXPECT_DOUBLE_EQ( 0, pdCZVrtCalcReactForce( &vrt, 0.4, 0.1 ) );
}

TEST_F(pdCZVrtTest, CalcAcc)
{
  pdCZVrtSetPrm( &vrt, 0.2, 1.0, 1.0 );
  EXPECT_DOUBLE_EQ( 0, pdCZVrtCalcAcc( &vrt, 0.2, 0 ) );
  EXPECT_DOUBLE_EQ( 4.903326, pdCZVrtCalcAcc( &vrt, 0.1, 0 ) );
  EXPECT_DOUBLE_EQ( 3.5028509377443386, pdCZVrtCalcAcc( &vrt, 0.1, 0.1 ) );
  EXPECT_DOUBLE_EQ( -9.806652, pdCZVrtCalcAcc( &vrt, 0.4, 0.1 ) );
}

TEST_F(pdCZVrtTest, ClacZMP)
{
  EXPECT_DOUBLE_EQ( 0, pdCZVrtCalcZMP( &vrt ) );
}

TEST_F(pdCZVrtTest, CalcZeta)
{
  EXPECT_DOUBLE_EQ( 7.0023753112783096, pdCZVrtCalcZeta( &vrt, 0.2, 0, 0 ) );
  EXPECT_DOUBLE_EQ( 7.037986928092435, pdCZVrtCalcZeta( &vrt, 0.2, 0.1, 0 ) );
  EXPECT_DOUBLE_EQ( 9.9532165655128786, pdCZVrtCalcZeta( &vrt, 0.2, 0.1, 0.1 ) );
}
