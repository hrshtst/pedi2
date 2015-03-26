#include "gtest/gtest.h"
#include <pedi2/pd_core.h>

class pdCoreTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdCore core;
};

TEST_F(pdCoreTest, InitProcess)
{
  char model[] = "model/mighty.zkc";
  char conf[]  = "model/mighty_ik.conf";

  pdCoreInit( &core );
  pdCoreLoad( &core, model, conf );
  SUCCEED();
}
