#include "gtest/gtest.h"
#include "utility/random_initializer.h"
#include <pedi2/pd_cz.h>

const double TIME_STEP = 0.01;

class pdCZTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZInit( &cz, TIME_STEP );
    destroyed_flag = false;
  };
  virtual void TearDown() {
    if( !destroyed_flag )
      pdCZDestroy( &cz );
  };

  void SetRandomValues() {
    pdCZSetCmdCOMX( &cz,  ri.rand() );
    pdCZSetCmdCOMY( &cz,  ri.rand() );
    pdCZSetCmdCOMZ( &cz,  ri.rand() );
    pdCZSetCmdTheta( &cz, ri.rand() );
    pdCZSetCOMX( &cz, ri.rand() );
    pdCZSetCOMY( &cz, ri.rand() );
    pdCZSetCOMZ( &cz, ri.rand() );
    pdCZSetVelX( &cz, ri.rand() );
    pdCZSetVelY( &cz, ri.rand() );
    pdCZSetVelZ( &cz, ri.rand() );
    pdCZSetAccX( &cz, ri.rand() );
    pdCZSetAccY( &cz, ri.rand() );
    pdCZSetAccZ( &cz, ri.rand() );
    pdCZSetZMPX( &cz, ri.rand() );
    pdCZSetZMPY( &cz, ri.rand() );
    pdCZSetZMPZ( &cz, ri.rand() );
    pdCZSetFZ( &cz, ri.rand() );
    pdCZSetExtFX( &cz, ri.rand() );
    pdCZSetExtFY( &cz, ri.rand() );
    pdCZSetExtFZ( &cz, ri.rand() );
    pdCZSetTheta( &cz, ri.rand() );
    pdCZSetSR( &cz, &sr );
    pdCZSetQ1Z( &cz, ri.rand() );
    pdCZSetQ2Z( &cz, ri.rand() );
    pdCZSetRefVelU( &cz, ri.rand() );
    pdCZSetQ1U( &cz, ri.rand() );
    pdCZSetQ2U( &cz, ri.rand() );
    pdCZSetRefVelW( &cz, ri.rand() );
    pdCZSetQ1W( &cz, ri.rand() );
    pdCZSetQ2W( &cz, ri.rand() );
    pdCZSetRho( &cz, ri.rand() );
    pdCZSetKr( &cz, ri.rand() );
    pdCZSetDist( &cz, ri.rand() );
    pdCZSetKappa( &cz, ri.rand() );
    pdCZSetRefPosU( &cz, ri.rand() );
    pdCZSetRefPosW( &cz, ri.rand() );
    pdCZSetDeltaU( &cz, ri.rand() );
    pdCZSetDeltaW( &cz, ri.rand() );
    pdCZSetVelU( &cz, ri.rand() );
    pdCZSetVelW( &cz, ri.rand() );
    pdCZSetErrCompKX( &cz, ri.rand() );
    pdCZSetErrCompKY( &cz, ri.rand() );
    pdCZSetErrCompKZ( &cz, ri.rand() );
    pdCZSetErrCompBX( &cz, ri.rand() );
    pdCZSetErrCompBY( &cz, ri.rand() );
    pdCZSetErrCompBZ( &cz, ri.rand() );
    pdCZAlphaX( &cz )  = ri.rand();
    pdCZAlphaY( &cz )  = ri.rand();
    pdCZAlphaZ( &cz )  = ri.rand();
    pdCZBetaX( &cz )   = ri.rand();
    pdCZBetaY( &cz )   = ri.rand();
    pdCZBetaZ( &cz )   = ri.rand();
    pdCZRefCOMX( &cz ) = ri.rand();
    pdCZRefCOMY( &cz ) = ri.rand();
    pdCZRefCOMZ( &cz ) = ri.rand();
    pdCZRefVelX( &cz ) = ri.rand();
    pdCZRefVelY( &cz ) = ri.rand();
    pdCZRefVelZ( &cz ) = ri.rand();
    pdCZRefAccX( &cz ) = ri.rand();
    pdCZRefAccY( &cz ) = ri.rand();
    pdCZRefAccZ( &cz ) = ri.rand();
    pdCZRefZMPX( &cz ) = ri.rand();
    pdCZRefZMPY( &cz ) = ri.rand();
    pdCZRefZMPZ( &cz ) = ri.rand();
    pdCZRefFZ( &cz )   = ri.rand();
  };

  RandomInitializer ri;
  zLoop3D sr;
  pdCZ cz;
  bool destroyed_flag;
};

TEST_F(pdCZTest, Init)
{
  SetRandomValues();
  pdCZInit( &cz, TIME_STEP );
  EXPECT_EQ( 0, pdCZCmdCOMX( &cz )  );
  EXPECT_EQ( 0, pdCZCmdCOMY( &cz )  );
  EXPECT_EQ( 0, pdCZCmdCOMZ( &cz )  );
  EXPECT_EQ( 0, pdCZCmdTheta( &cz ) );
  EXPECT_EQ( 0, pdCZCOMX( &cz )     );
  EXPECT_EQ( 0, pdCZCOMY( &cz )     );
  EXPECT_EQ( 0, pdCZCOMZ( &cz )     );
  EXPECT_EQ( 0, pdCZVelX( &cz )     );
  EXPECT_EQ( 0, pdCZVelY( &cz )     );
  EXPECT_EQ( 0, pdCZVelZ( &cz )     );
  EXPECT_EQ( 0, pdCZAccX( &cz )     );
  EXPECT_EQ( 0, pdCZAccY( &cz )     );
  EXPECT_EQ( 0, pdCZAccZ( &cz )     );
  EXPECT_EQ( 0, pdCZZMPX( &cz )     );
  EXPECT_EQ( 0, pdCZZMPY( &cz )     );
  EXPECT_EQ( 0, pdCZZMPZ( &cz )     );
  EXPECT_EQ( 0, pdCZFZ( &cz )       );
  EXPECT_EQ( 0, pdCZExtFX( &cz )    );
  EXPECT_EQ( 0, pdCZExtFY( &cz )    );
  EXPECT_EQ( 0, pdCZExtFZ( &cz )    );
  EXPECT_EQ( 0, pdCZTheta( &cz )    );
  EXPECT_EQ( NULL, pdCZSR( &cz )    );
  EXPECT_EQ( 0, pdCZQ1Z( &cz )      );
  EXPECT_EQ( 0, pdCZQ2Z( &cz )      );
  EXPECT_EQ( 0, pdCZRefVelU( &cz )  );
  EXPECT_EQ( 0, pdCZQ1U( &cz )      );
  EXPECT_EQ( 0, pdCZQ2U( &cz )      );
  EXPECT_EQ( 0, pdCZRefVelW( &cz )  );
  EXPECT_EQ( 0, pdCZQ1W( &cz )      );
  EXPECT_EQ( 0, pdCZQ2W( &cz )      );
  EXPECT_EQ( 0, pdCZRho( &cz )      );
  EXPECT_EQ( 0, pdCZKr( &cz )       );
  EXPECT_EQ( 0, pdCZDist( &cz )     );
  EXPECT_EQ( 0, pdCZKappa( &cz )    );
  EXPECT_EQ( 0, pdCZRefPosU( &cz )  );
  EXPECT_EQ( 0, pdCZRefPosW( &cz )  );
  EXPECT_EQ( 0, pdCZDeltaU( &cz )  );
  EXPECT_EQ( 0, pdCZDeltaW( &cz )  );
  EXPECT_EQ( 0, pdCZVelU( &cz )  );
  EXPECT_EQ( 0, pdCZVelW( &cz )  );
  EXPECT_EQ( 0, pdCZAlphaX( &cz )   );
  EXPECT_EQ( 0, pdCZAlphaY( &cz )   );
  EXPECT_EQ( 0, pdCZAlphaZ( &cz )   );
  EXPECT_EQ( 0, pdCZBetaX( &cz )   );
  EXPECT_EQ( 0, pdCZBetaY( &cz )   );
  EXPECT_EQ( 0, pdCZBetaZ( &cz )   );
  EXPECT_EQ( 0, pdCZErrCompKX( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompKY( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompKZ( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompBX( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompBY( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompBZ( &cz ) );
  EXPECT_EQ( 0, pdCZRefCOMX( &cz )  );
  EXPECT_EQ( 0, pdCZRefCOMY( &cz )  );
  EXPECT_EQ( 0, pdCZRefCOMZ( &cz )  );
  EXPECT_EQ( 0, pdCZRefVelX( &cz )  );
  EXPECT_EQ( 0, pdCZRefVelY( &cz )  );
  EXPECT_EQ( 0, pdCZRefVelZ( &cz )  );
  EXPECT_EQ( 0, pdCZRefAccX( &cz )  );
  EXPECT_EQ( 0, pdCZRefAccY( &cz )  );
  EXPECT_EQ( 0, pdCZRefAccZ( &cz )  );
  EXPECT_EQ( 0, pdCZRefZMPX( &cz )  );
  EXPECT_EQ( 0, pdCZRefZMPY( &cz )  );
  EXPECT_EQ( 0, pdCZRefZMPZ( &cz )  );
  EXPECT_EQ( 0, pdCZRefFZ( &cz ) );
}

TEST_F(pdCZTest, Destroy)
{
  SetRandomValues();
  pdCZDestroy( &cz );
  EXPECT_EQ( 0, pdCZCmdCOMX( &cz )  );
  EXPECT_EQ( 0, pdCZCmdCOMY( &cz )  );
  EXPECT_EQ( 0, pdCZCmdCOMZ( &cz )  );
  EXPECT_EQ( 0, pdCZCmdTheta( &cz ) );
  EXPECT_EQ( 0, pdCZCOMX( &cz )     );
  EXPECT_EQ( 0, pdCZCOMY( &cz )     );
  EXPECT_EQ( 0, pdCZCOMZ( &cz )     );
  EXPECT_EQ( 0, pdCZVelX( &cz )     );
  EXPECT_EQ( 0, pdCZVelY( &cz )     );
  EXPECT_EQ( 0, pdCZVelZ( &cz )     );
  EXPECT_EQ( 0, pdCZAccX( &cz )     );
  EXPECT_EQ( 0, pdCZAccY( &cz )     );
  EXPECT_EQ( 0, pdCZAccZ( &cz )     );
  EXPECT_EQ( 0, pdCZZMPX( &cz )     );
  EXPECT_EQ( 0, pdCZZMPY( &cz )     );
  EXPECT_EQ( 0, pdCZZMPZ( &cz )     );
  EXPECT_EQ( 0, pdCZFZ( &cz )       );
  EXPECT_EQ( 0, pdCZExtFX( &cz )    );
  EXPECT_EQ( 0, pdCZExtFY( &cz )    );
  EXPECT_EQ( 0, pdCZExtFZ( &cz )    );
  EXPECT_EQ( 0, pdCZTheta( &cz )    );
  EXPECT_EQ( NULL, pdCZSR( &cz )    );
  EXPECT_EQ( 0, pdCZQ1Z( &cz )      );
  EXPECT_EQ( 0, pdCZQ2Z( &cz )      );
  EXPECT_EQ( 0, pdCZRefVelU( &cz )  );
  EXPECT_EQ( 0, pdCZQ1U( &cz )      );
  EXPECT_EQ( 0, pdCZQ2U( &cz )      );
  EXPECT_EQ( 0, pdCZRefVelW( &cz )  );
  EXPECT_EQ( 0, pdCZQ1W( &cz )      );
  EXPECT_EQ( 0, pdCZQ2W( &cz )      );
  EXPECT_EQ( 0, pdCZRho( &cz )      );
  EXPECT_EQ( 0, pdCZKr( &cz )       );
  EXPECT_EQ( 0, pdCZDist( &cz )     );
  EXPECT_EQ( 0, pdCZKappa( &cz )    );
  EXPECT_EQ( 0, pdCZRefPosU( &cz )  );
  EXPECT_EQ( 0, pdCZRefPosW( &cz )  );
  EXPECT_EQ( 0, pdCZDeltaU( &cz )  );
  EXPECT_EQ( 0, pdCZDeltaW( &cz )  );
  EXPECT_EQ( 0, pdCZVelU( &cz )  );
  EXPECT_EQ( 0, pdCZVelW( &cz )  );
  EXPECT_EQ( 0, pdCZErrCompKX( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompKY( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompKZ( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompBX( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompBY( &cz ) );
  EXPECT_EQ( 0, pdCZErrCompBZ( &cz ) );
  destroyed_flag = true;
}

TEST_F(pdCZTest, SetCmdCOM)
{
  SetRandomValues();
  pdCZSetCmdCOM( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCmdCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCmdCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCmdCOMVec)
{
  zVec3D v;

  SetRandomValues();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetCmdCOMVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCmdCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCmdCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCmdCOMXYZ)
{
  SetRandomValues();
  pdCZSetCmdCOMX( &cz, 0.1 );
  pdCZSetCmdCOMY( &cz, 0.2 );
  pdCZSetCmdCOMZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCmdCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCmdCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCmdTheta)
{
  SetRandomValues();
  pdCZSetCmdTheta( &cz, 0.1 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdTheta(&cz) );
}

TEST_F(pdCZTest, SetCOM)
{
  SetRandomValues();
  pdCZSetCOM( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCOMVec)
{
  zVec3D v;

  SetRandomValues();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetCOMVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCOMXYZ)
{
  SetRandomValues();
  pdCZSetCOMX( &cz, 0.1 );
  pdCZSetCOMY( &cz, 0.2 );
  pdCZSetCOMZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCOMZ(&cz) );
}

TEST_F(pdCZTest, SetVel)
{
  SetRandomValues();
  pdCZSetVel( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZVelX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZVelY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZVelZ(&cz) );
}

TEST_F(pdCZTest, SetVelVec)
{
  zVec3D v;

  SetRandomValues();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetVelVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZVelX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZVelY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZVelZ(&cz) );
}

TEST_F(pdCZTest, SetVelXYZ)
{
  SetRandomValues();
  pdCZSetVelX( &cz, 0.1 );
  pdCZSetVelY( &cz, 0.2 );
  pdCZSetVelZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZVelX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZVelY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZVelZ(&cz) );
}

TEST_F(pdCZTest, SetAcc)
{
  SetRandomValues();
  pdCZSetAcc( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZAccX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZAccY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZAccZ(&cz) );
}

TEST_F(pdCZTest, SetAccVec)
{
  zVec3D v;

  SetRandomValues();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetAccVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZAccX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZAccY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZAccZ(&cz) );
}

TEST_F(pdCZTest, SetAccXYZ)
{
  SetRandomValues();
  pdCZSetAccX( &cz, 0.1 );
  pdCZSetAccY( &cz, 0.2 );
  pdCZSetAccZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZAccX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZAccY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZAccZ(&cz) );
}

TEST_F(pdCZTest, SetZMP)
{
  SetRandomValues();
  pdCZSetZMP( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZZMPX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZZMPY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZZMPZ(&cz) );
}

TEST_F(pdCZTest, SetZMPVec)
{
  zVec3D v;

  SetRandomValues();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetZMPVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZZMPX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZZMPY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZZMPZ(&cz) );
}

TEST_F(pdCZTest, SetZMPXYZ)
{
  SetRandomValues();
  pdCZSetZMPX( &cz, 0.1 );
  pdCZSetZMPY( &cz, 0.2 );
  pdCZSetZMPZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZZMPX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZZMPY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZZMPZ(&cz) );
}

TEST_F(pdCZTest, SetFZ)
{
  SetRandomValues();
  pdCZSetFZ( &cz, 0.1 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZFZ(&cz) );
}

TEST_F(pdCZTest, SetTheta)
{
  SetRandomValues();
  pdCZSetTheta( &cz, 0.1 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZTheta(&cz) );
}

TEST_F(pdCZTest, SetRefPosUW)
{
  SetRandomValues();
  pdCZSetRefPosUW( &cz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZRefPosU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZRefPosW(&cz) );
}

TEST_F(pdCZTest, SetRefPosUWVec)
{
  zVec2D v;

  SetRandomValues();
  zVec2DCreate( &v, 0.1, 0.2 );
  pdCZSetRefPosUWVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZRefPosU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZRefPosW(&cz) );
}

TEST_F(pdCZTest, SetRefPosUPosW)
{
  SetRandomValues();
  pdCZSetRefPosU( &cz, 0.1 );
  pdCZSetRefPosW( &cz, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZRefPosU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZRefPosW(&cz) );
}

TEST_F(pdCZTest, SetDelta)
{
  SetRandomValues();
  pdCZSetDelta( &cz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZDeltaU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZDeltaW(&cz) );
}

TEST_F(pdCZTest, SetDeltaVec)
{
  zVec2D v;

  SetRandomValues();
  zVec2DCreate( &v, 0.1, 0.2 );
  pdCZSetDeltaVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZDeltaU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZDeltaW(&cz) );
}

TEST_F(pdCZTest, SetDeltaUW)
{
  SetRandomValues();
  pdCZSetDeltaU( &cz, 0.1 );
  pdCZSetDeltaW( &cz, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZDeltaU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZDeltaW(&cz) );
}

TEST_F(pdCZTest, SetTime)
{
  SetRandomValues();
  cz._ode._t = 1000;

  pdCZInit( &cz, TIME_STEP );
  EXPECT_DOUBLE_EQ( 0.0, pdCZTime(&cz) );

  pdCZSetTime( &cz, 10 );
  EXPECT_DOUBLE_EQ( 10.0, pdCZTime(&cz) );
}

TEST_F(pdCZTest, SetTimeStep)
{
  SetRandomValues();
  cz._ode._dt = 1000;

  pdCZInit( &cz, TIME_STEP );
  EXPECT_DOUBLE_EQ( TIME_STEP, pdCZTimeStep(&cz) );

  pdCZSetTimeStep( &cz, 0.001 );
  EXPECT_DOUBLE_EQ( 0.001, pdCZTimeStep(&cz) );
}

TEST_F(pdCZTest, SetErrCompKXYZ)
{
  SetRandomValues();
  pdCZSetErrCompKX( &cz, 0.1 );
  pdCZSetErrCompKY( &cz, 0.2 );
  pdCZSetErrCompKZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZErrCompKX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZErrCompKY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZErrCompKZ(&cz) );
}

TEST_F(pdCZTest, SetErrCompBXYZ)
{
  SetRandomValues();
  pdCZSetErrCompBX( &cz, 0.1 );
  pdCZSetErrCompBY( &cz, 0.2 );
  pdCZSetErrCompBZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZErrCompBX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZErrCompBY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZErrCompBZ(&cz) );
}

TEST_F(pdCZTest, CalcDeltaTheta_KappaIsZero)
{
  zVec2D refuw;

  pdCZSetKappa( &cz, 0 );
  pdCZSetDeltaW( &cz, 2*sqrt(3) );
  zVec2DCreate( &refuw, 2, 1 );
  EXPECT_NEAR( 0, pdCZCalcDeltaTheta( &cz, &refuw ), 1e-12 );
}

TEST_F(pdCZTest, CalcDeltaTheta_KappaIsNotZero)
{
  zVec2D refuw;

  pdCZSetKappa( &cz, 1 );
  pdCZSetDeltaW( &cz, 2*sqrt(3) );
  zVec2DCreate( &refuw, 2, 1 );
  EXPECT_NEAR( zPI/6.0, pdCZCalcDeltaTheta( &cz, &refuw ), 1e-12 );
}

TEST_F(pdCZTest, CalcNextUW_KappaIsZero)
{
  zVec2D refuw, nextuwd;

  pdCZSetKappa( &cz, 0 );
  pdCZSetDeltaW( &cz, 2*sqrt(3) );
  zVec2DCreate( &refuw, 2, 1 );
  zVec2DZero( &nextuwd );
  pdCZCalcNextUW( &cz, &refuw, &nextuwd );
  EXPECT_NEAR( 2.0, nextuwd.e[pdU], 1e-12 );
  EXPECT_NEAR( 2*sqrt(3), nextuwd.e[pdW], 1e-12 );
}

TEST_F(pdCZTest, CalcNextUW_KappaIsNotZero)
{
  zVec2D refuw, nextuwd;

  pdCZSetKappa( &cz, 1 );
  pdCZSetDeltaW( &cz, 2*sqrt(3) );
  zVec2DCreate( &refuw, 2, 1 );
  zVec2DZero( &nextuwd );
  pdCZCalcNextUW( &cz, &refuw,  &nextuwd );
  EXPECT_NEAR( 0.5, nextuwd.e[pdU], 1e-12 );
  EXPECT_NEAR( 1.5*sqrt(3)+1, nextuwd.e[pdW], 1e-12 );
}


#if 0
TEST_F(pdCZTest, Init)
{
  pdCZ _cz;

  pdCZInit( &_cz );
  // vertical
  EXPECT_EQ( 0, _cz.vrt.zd );
  EXPECT_EQ( 0, _cz.vrt.z );
  EXPECT_EQ( 0, _cz.vrt.zeta );
  // tangential
  EXPECT_EQ( (byte)PD_CZ_HRZ_TAN, _cz.tan.dir );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->vd );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->q1 );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->q2 );
  EXPECT_EQ( 0, ((pdCZHrzPrmTan *)_cz.tan.prm)->kappa );
  EXPECT_EQ( &_cz.vrt, ((pdCZHrzPrmTan *)_cz.tan.prm)->vrt );
  EXPECT_EQ( 0, _cz.vrt.z );
  // radial
  EXPECT_EQ( (byte)PD_CZ_HRZ_RAD, _cz.rad.dir );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->vd );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->q1 );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->q2 );
  EXPECT_EQ( 0, ((pdCZHrzPrmRad *)_cz.rad.prm)->kappa );
  EXPECT_EQ( &_cz.vrt, ((pdCZHrzPrmRad *)_cz.rad.prm)->vrt );
  EXPECT_EQ( 0, _cz.vrt.z );
}

TEST_F(pdCZTest, Destroy)
{
  pdCZ _cz;

  pdCZInit( &_cz );
  pdCZDestroy( &_cz );
  // vertical
  EXPECT_EQ( 0, _cz.vrt.zd );
  EXPECT_EQ( 0, _cz.vrt.z );
  EXPECT_EQ( 0, _cz.vrt.zeta );
  // tangential
  EXPECT_EQ( -1, _cz.tan.dir );
  EXPECT_EQ( NULL, _cz.tan.prm );
  EXPECT_EQ( NULL, _cz.tan.com );
  // radial
  EXPECT_EQ( -1, _cz.rad.dir );
  EXPECT_EQ( NULL, _cz.rad.prm );
  EXPECT_EQ( NULL, _cz.rad.com );
}

TEST_F(pdCZTest, SetPrm)
{
  pdCZSetPrm( &cz, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 1, pdCZPrmTan(&cz)->q1 );
  EXPECT_EQ( 2, pdCZPrmTan(&cz)->q2 );
  EXPECT_EQ( 5, pdCZPrmTan(&cz)->kappa );
  EXPECT_EQ( 3, pdCZPrmRad(&cz)->q1 );
  EXPECT_EQ( 4, pdCZPrmRad(&cz)->q2 );
  EXPECT_EQ( 5, pdCZPrmRad(&cz)->kappa );
  EXPECT_EQ( 6, pdCZPrmRad(&cz)->rho );
  EXPECT_EQ( 7, pdCZPrmRad(&cz)->kr );
}

TEST_F(pdCZTest, SetRefVrt)
{
  pdCZSetRefVrt( &cz, 1 );
  EXPECT_EQ( 1, cz.vrt.zd );
}

TEST_F(pdCZTest, SetRefHrz)
{
  pdCZSetRefHrz( &cz, 1, 2, 3 );
  EXPECT_EQ( 1, pdCZPrmTan(&cz)->vd );
  EXPECT_EQ( 2, pdCZPrmRad(&cz)->vd );
  EXPECT_EQ( 3, pdCZPrmRad(&cz)->dist );
}

TEST_F(pdCZTest, Kappa)
{
  pdCZSetPrm( &cz, 1, 2, 3, 4, 5, 6, 7 );
  EXPECT_EQ( 5, pdCZKappa( &cz ) );
  EXPECT_EQ( pdCZPrmTan(&cz)->kappa, pdCZKappa( &cz ) );
  EXPECT_EQ( pdCZPrmRad(&cz)->kappa, pdCZKappa( &cz ) );
}

TEST_F(pdCZTest, UpdateAllStateZero)
{
  pdCZSetPrm( &cz, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCZSetRefVrt( &cz, 0.26 );
  pdCZSetRefHrz( &cz, 0.25, 0.0, 0.1 );
  pdCZUpdate( &cz, 0.0, 0.0, 0.0, 0.0 );
  EXPECT_EQ( 0.26, cz.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&cz) );
  EXPECT_DOUBLE_EQ( -0.040706737871602130529602, pdCZZMPTan( &cz ) );
  EXPECT_DOUBLE_EQ( 0.0, pdCZZMPRad( &cz ) );
}

TEST_F(pdCZTest, UpdateVelocityFollow)
{
  pdCZSetPrm( &cz, 1.0, 0.0, 1.0, 1.5, 0.0, 1.0, 1.0 );
  pdCZSetRefVrt( &cz, 0.26 );
  pdCZSetRefHrz( &cz, 0.25, 0.0, 0.1 );
  pdCZUpdate( &cz, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, cz.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&cz) );
  EXPECT_DOUBLE_EQ( -0.008141347574320424718142, pdCZZMPTan( &cz ) );
  //EXPECT_DOUBLE_EQ( 0.000310222468637089826560, pdCZZMPRad( &cz ) );
  EXPECT_DOUBLE_EQ( 0.000310222468637088091836, pdCZZMPRad( &cz ) );
  EXPECT_DOUBLE_EQ( 0.307074471047709740556542, pdCZAccTan( &cz ) );
  EXPECT_DOUBLE_EQ( 0.011700937663480142150729, pdCZAccRad( &cz ) );
}

TEST_F(pdCZTest, UpdateVelocityFollowCurve)
{
  pdCZSetPrm( &cz, 1.0, 0.0, 1.0, 1.5, 2.0, 1.0, 1.0 );
  pdCZSetRefVrt( &cz, 0.26 );
  pdCZSetRefHrz( &cz, 0.25, 0.0, 0.1 );
  pdCZUpdate( &cz, 0.0, 0.2, 0.01, 0.1 );
  EXPECT_EQ( 0.26, cz.vrt.z );
  EXPECT_EQ( sqrt(RK_G/0.26), pdCZZeta(&cz) );
  EXPECT_DOUBLE_EQ( -0.005162917622573504408678, pdCZZMPTan( &cz ) );
  // EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCZZMPRad( &cz ) );
  EXPECT_DOUBLE_EQ( -0.001854072725677788271298, pdCZZMPRad( &cz ) );
  EXPECT_DOUBLE_EQ( 0.276367023942938716540851, pdCZAccTan( &cz ) );
  EXPECT_DOUBLE_EQ( 0.011700937663480154293794, pdCZAccRad( &cz ) );
}

TEST_F(pdCZTest, ZMPPhase)
{
  zComplex pz;

  zComplexCreate( &pz, 0, 0 );
  pdCZPrmRad(&cz)->q1 = 1;
  pdCZPrmRad(&cz)->q2 = 1;
  pdCZZeta(&cz) = 1;

  pdCZPrmRad(&cz)->zmp = 0;
  pdCZZMPPhase( &cz, 0.0, 0.1, &pz );
  EXPECT_EQ( 0, pz.re );
  EXPECT_EQ( -0.2, pz.im );

  pdCZPrmRad(&cz)->zmp = 0;
  pdCZZMPPhase( &cz, 0.1, 0.0, &pz );
  EXPECT_EQ( 0, pz.re );
  EXPECT_EQ( 0, pz.im );

  pdCZPrmRad(&cz)->zmp = 0.3;
  pdCZZMPPhase( &cz, -0.1, -0.15, &pz );
  EXPECT_EQ( 0.3, pz.re );
  EXPECT_EQ( 0.3, pz.im );
}
#endif
