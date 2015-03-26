#include "gtest/gtest.h"
#include <pedi2/pd_core.h>

__BEGIN_DECLS

void _pdCoreLoad(pdCore *core, const char* model_file, const char* conf_file);
void _pdCorePoseInit(pdCore *core);

__END_DECLS

class pdCoreTest : public testing::Test {
 protected:
  virtual void SetUp() {};
  virtual void TearDown() {};

  pdCore core;
  pdCommand com;
};

TEST_F(pdCoreTest, InitProcess)
{
  char model[] = "model/mighty.zkc";
  char conf[]  = "model/mighty_ik.conf";

  pdCoreInit( &core, &com );
  _pdCoreLoad( &core, model, conf );

  // feet position and attitude
  EXPECT_NEAR( 0.034, zVec3DElem(&core.lf.p,zX), 1e-04 );
  EXPECT_NEAR( 0.042, zVec3DElem(&core.lf.p,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.p,zZ), 1e-04 );
  EXPECT_NEAR( 0.034, zVec3DElem(&core.rf.p,zX), 1e-04 );
  EXPECT_NEAR( -0.042, zVec3DElem(&core.rf.p,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.p,zZ), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.a,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.a,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.a,zZ), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.a,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.a,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.a,zZ), 1e-04 );
  // sole width
  EXPECT_NEAR( 0.072, core.lf.sole_w, 1e-04 );
  EXPECT_NEAR( 0.072, core.rf.sole_w, 1e-04 );
  // default distance
  EXPECT_NEAR( 0.084, pdCZPrmRad(&core.cz)->dist, 1e-04 );
  // COM height
  // EXPECT_NEAR( 0.29286402589651961, core.cz.vrt.zd, 1e-04 );
  EXPECT_NEAR( 0.26357762330686768, core.cz.vrt.zd, 1e-04 );

  _pdCorePoseInit( &core );
  // feet position and attitude
  EXPECT_NEAR( -0.042, zVec3DElem(&core.lf.p,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.p,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.p,zZ), 1e-04 );
  EXPECT_NEAR( 0.042, zVec3DElem(&core.rf.p,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.p,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.p,zZ), 1e-04 );
  EXPECT_NEAR( zPI_2, zVec3DElem(&core.lf.a,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.a,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.lf.a,zZ), 1e-04 );
  EXPECT_NEAR( zPI_2, zVec3DElem(&core.rf.a,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.a,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&core.rf.a,zZ), 1e-04 );
  // COM height
  zVec3D com_pos, body_att;
  zVec3DCopy( rkChainWldCOM(&core.robot.chain), &com_pos );
  zMat3DToZYX( rkChainLinkWldAtt(&core.robot.chain,core.robot.body_id), &body_att );
  EXPECT_NEAR( 0, zVec3DElem(&com_pos,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&com_pos,zY), 1e-04 );
  // EXPECT_NEAR( 0.29286402589651961, zVec3DElem(&com_pos,zZ), 1e-04 );
  EXPECT_NEAR( 0.26357762330686768, zVec3DElem(&com_pos,zZ), 1e-04 );
  EXPECT_NEAR( zPI_2, zVec3DElem(&body_att,zX), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&body_att,zY), 1e-04 );
  EXPECT_NEAR( 0, zVec3DElem(&body_att,zZ), 1e-04 );
  // spring-damper tracking
  EXPECT_NEAR( -0.042, core.lf.track_old[0], 1e-04 );
  EXPECT_NEAR( 0     , core.lf.track_old[1], 1e-04 );
  EXPECT_NEAR( 0     , core.lf.track_old[2], 1e-04 );
  EXPECT_NEAR( 0.042 , core.rf.track_old[0], 1e-04 );
  EXPECT_NEAR( 0     , core.rf.track_old[1], 1e-04 );
  EXPECT_NEAR( 0     , core.rf.track_old[2], 1e-04 );
}
