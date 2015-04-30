#include "gtest/gtest.h"
#include <pedi2/pd_foot_z.h>

class pdFootZTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZVrtInit( &vrt );
    pdCZHrzUWInit( &czuw, &vrt );
    pdFootZInit( &lf, &czuw );
    pdFootZInit( &rf, &czuw );
    pdFootZSign( &lf ) = 1;
    pdFootZSign( &rf ) = -1;
  };
  virtual void TearDown() {
    pdFootZDestroy( &lf );
    pdFootZDestroy( &rf );
    pdCZHrzUWDestroy( &czuw );
    pdCZVrtDestroy( &vrt );
  };

  void SetVacuousPrm() {
    pdFootZCZPtr( &lf ) = NULL;
    pdFootZSetMaxHeight( &lf, 100 );
    pdFootZSetSR( &lf, &rsr ); // purposely set pointer to rsr
    pdFootZSign( &lf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &lf ), 10, 20 );
    pdFootZFootPhase( &lf ) = 30;
    pdFootZRefHeight( &lf ) = 40;

    pdFootZCZPtr( &rf ) = NULL;
    pdFootZSetMaxHeight( &rf, 100 );
    pdFootZSetSR( &rf, &lsr ); // purposely set pointer to lsr
    pdFootZSign( &rf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &rf ), 10, 20 );
    pdFootZFootPhase( &rf ) = 30;
    pdFootZRefHeight( &rf ) = 40;
}

  zVec3DList lsr, rsr;
  pdCZVrt vrt;
  pdCZHrzUW czuw;
  pdFootZ lf, rf;
};

TEST_F(pdFootZTest, Init)
{
  SetVacuousPrm();
  pdFootZInit( &lf, &czuw );
  EXPECT_EQ( &czuw, pdFootZCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &lf ) );
  EXPECT_EQ( NULL, pdFootZSR( &lf ) );
  EXPECT_EQ( 0, pdFootZSign( &lf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &lf ) );

  pdFootZInit( &rf, &czuw );
  EXPECT_EQ( &czuw, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_EQ( NULL, pdFootZSR( &rf ) );
  EXPECT_EQ( 0, pdFootZSign( &rf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &rf ) );
}

TEST_F(pdFootZTest, Destroy)
{
  SetVacuousPrm();
  pdFootZDestroy( &lf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &lf ) );
  EXPECT_EQ( NULL, pdFootZSR( &lf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &lf ) );

  pdFootZDestroy( &rf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_EQ( NULL, pdFootZSR( &rf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefHeight( &rf ) );
}
