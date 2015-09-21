#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_sensor.h>

const double TIME_STEP = 0.01;

class pdSensorTest : public testing::Test {
protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdSensor sensor;
  RandomInitializer ri;
};

TEST_F(pdSensorTest, Init)
{
  pdSensorInit( &sensor );
  EXPECT_EQ( NULL, zNamePtr( &sensor ) );
  EXPECT_EQ( NULL, sensor._prm );
  EXPECT_EQ( NULL, sensor._met );
}
