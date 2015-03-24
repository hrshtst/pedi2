#include "gtest/gtest.h"
#include <zm/zm_complex.h>
#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>

class pdFootTest : public testing::Test {
protected:
  virtual void SetUp() {
    pdCZInit( &ctrl );
    // left foot
    lf.dy = 1;
    lf.sole_w = 0.07;
    // right foot
    rf.dy = -1;
    rf.sole_w = 0.07;
  };
  virtual void TearDown() {};

  pdCZ ctrl;
  pdFoot lf;
  pdFoot rf;
  zComplex pz;
};

TEST_F(pdFootTest, CalcFootPhaseDoubleSupport)
{
  zComplexCreate(&pz, 0, 0 );
  pdFootPhase( &lf, &rf, 0, 0, 0, &pz );
  pdFootPhase( &rf, &lf, 0, 0, 0, &pz );
  EXPECT_EQ( 0, lf.phase );
  EXPECT_EQ( 0, rf.phase );
}
