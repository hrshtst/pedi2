#include "gtest/gtest.h"
#include <pedi2/pd_state.h>

class pdStateTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdStateInit( &state );
  };
  virtual void TearDown() {
    pdStateDestroy( &state );
  };

  bool zMat3DIsTol(zMat3D *m, double tol) {
    return zIsTol( m->c[0], tol ) &&
        zIsTol( m->c[1], tol ) &&
        zIsTol( m->c[2], tol ) &&
        zIsTol( m->c[3], tol ) &&
        zIsTol( m->c[4], tol ) &&
        zIsTol( m->c[5], tol ) &&
        zIsTol( m->c[6], tol ) &&
        zIsTol( m->c[7], tol ) &&
        zIsTol( m->c[8], tol );
  }

  bool zMat3DIsTiny(zMat3D *m) {
    return zMat3DIsTol( m, zTOL );
  }

  pdState state;
};

TEST_F(pdStateTest, Init)
{
  EXPECT_TRUE( zVec3DIsTiny( &state.com_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.com_vel ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lh_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rh_pos ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.base_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.lf_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.rf_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.lh_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.rh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.zmp ) );
  EXPECT_EQ( 0, state.fz );
  EXPECT_EQ( NULL, state.sr);
}

TEST_F(pdStateTest, Destroy)
{
  EXPECT_TRUE( zVec3DIsTiny( &state.com_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.com_vel ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rf_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.lh_pos ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.rh_pos ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.base_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.lf_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.rf_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.lh_att ) );
  EXPECT_TRUE( zMat3DIsTiny( &state.rh_att ) );
  EXPECT_TRUE( zVec3DIsTiny( &state.zmp ) );
  EXPECT_EQ( 0, state.fz );
  EXPECT_EQ( NULL, state.sr);
}
