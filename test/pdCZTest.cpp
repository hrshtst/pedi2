#include "gtest/gtest.h"
#include <pedi2/pd_cz.h>

class pdCZTest : public testing::Test {
 protected:
  virtual void SetUp() {
    pdCZInit( &cz );
    destroyed_flag = false;
  };
  virtual void TearDown() {
    if( !destroyed_flag )
      pdCZDestroy( &cz );
  };

  void SetVacuousPrm() {
    pdCZSetCmdCOMX( &cz, 1 );
    pdCZSetCmdCOMY( &cz, 2 );
    pdCZSetCmdCOMZ( &cz, 3 );
    pdCZSetCmdTheta( &cz, 4 );
    pdCZSetCOMX( &cz, 5 );
    pdCZSetCOMY( &cz, 6 );
    pdCZSetCOMZ( &cz, 7 );
    pdCZSetVelX( &cz, 8 );
    pdCZSetVelY( &cz, 9 );
    pdCZSetVelZ( &cz, 10 );
    pdCZSetAccX( &cz, 11 );
    pdCZSetAccY( &cz, 12 );
    pdCZSetAccZ( &cz, 13 );
    pdCZSetZMPX( &cz, 14 );
    pdCZSetZMPY( &cz, 15 );
    pdCZSetZMPZ( &cz, 16 );
    pdCZSetTheta( &cz, 17 );
    pdCZSetSR( &cz, &sr );
    pdCZSetRefZ( &cz, 18 );
    pdCZSetQ1Z( &cz, 19 );
    pdCZSetQ2Z( &cz, 20 );
    pdCZSetRefVelU( &cz, 21 );
    pdCZSetQ1U( &cz, 22 );
    pdCZSetQ2U( &cz, 23 );
    pdCZSetRefVelW( &cz, 24 );
    pdCZSetQ1W( &cz, 25 );
    pdCZSetQ2W( &cz, 26 );
    pdCZSetRho( &cz, 27 );
    pdCZSetKr( &cz, 28 );
    pdCZSetDist( &cz, 29 );
    pdCZSetKappa( &cz, 30 );
    pdCZSetRefPosU( &cz, 31 );
    pdCZSetRefPosW( &cz, 32 );
    pdCZSetDeltaU( &cz, 33 );
    pdCZSetDeltaW( &cz, 34 );
    pdCZSetVelU( &cz, 35 );
    pdCZSetVelW( &cz, 36 );
    pdCZRefCOMX( &cz ) = 37;
    pdCZRefCOMY( &cz ) = 38;
    pdCZRefCOMZ( &cz ) = 39;
    pdCZRefVelX( &cz ) = 40;
    pdCZRefVelY( &cz ) = 41;
    pdCZRefVelZ( &cz ) = 42;
    pdCZRefAccX( &cz ) = 43;
    pdCZRefAccY( &cz ) = 44;
    pdCZRefAccZ( &cz ) = 45;
    pdCZRefZMPX( &cz ) = 46;
    pdCZRefZMPY( &cz ) = 47;
    pdCZRefZMPZ( &cz ) = 48;
  };

  zVec3DList sr;
  pdCZ cz;
  bool destroyed_flag;
};

TEST_F(pdCZTest, Init)
{
  SetVacuousPrm();
  pdCZInit( &cz );
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
  EXPECT_EQ( 0, pdCZTheta( &cz )    );
  EXPECT_EQ( NULL, pdCZSR( &cz )    );
  EXPECT_EQ( 0, pdCZRefZ( &cz )     );
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
}

TEST_F(pdCZTest, Destroy)
{
  SetVacuousPrm();
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
  EXPECT_EQ( 0, pdCZTheta( &cz )    );
  EXPECT_EQ( NULL, pdCZSR( &cz )    );
  EXPECT_EQ( 0, pdCZRefZ( &cz )     );
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
  destroyed_flag = true;
}

TEST_F(pdCZTest, SetCmdCOM)
{
  SetVacuousPrm();
  pdCZSetCmdCOM( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCmdCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCmdCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCmdCOMVec)
{
  zVec3D v;

  SetVacuousPrm();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetCmdCOMVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCmdCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCmdCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCmdCOMXYZ)
{
  SetVacuousPrm();
  pdCZSetCmdCOMX( &cz, 0.1 );
  pdCZSetCmdCOMY( &cz, 0.2 );
  pdCZSetCmdCOMZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCmdCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCmdCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCmdTheta)
{
  SetVacuousPrm();
  pdCZSetCmdTheta( &cz, 0.1 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCmdTheta(&cz) );
}

TEST_F(pdCZTest, SetCOM)
{
  SetVacuousPrm();
  pdCZSetCOM( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCOMVec)
{
  zVec3D v;

  SetVacuousPrm();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetCOMVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCOMZ(&cz) );
}

TEST_F(pdCZTest, SetCOMXYZ)
{
  SetVacuousPrm();
  pdCZSetCOMX( &cz, 0.1 );
  pdCZSetCOMY( &cz, 0.2 );
  pdCZSetCOMZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZCOMX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZCOMY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZCOMZ(&cz) );
}

TEST_F(pdCZTest, SetVel)
{
  SetVacuousPrm();
  pdCZSetVel( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZVelX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZVelY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZVelZ(&cz) );
}

TEST_F(pdCZTest, SetVelVec)
{
  zVec3D v;

  SetVacuousPrm();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetVelVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZVelX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZVelY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZVelZ(&cz) );
}

TEST_F(pdCZTest, SetVelXYZ)
{
  SetVacuousPrm();
  pdCZSetVelX( &cz, 0.1 );
  pdCZSetVelY( &cz, 0.2 );
  pdCZSetVelZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZVelX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZVelY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZVelZ(&cz) );
}

TEST_F(pdCZTest, SetAcc)
{
  SetVacuousPrm();
  pdCZSetAcc( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZAccX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZAccY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZAccZ(&cz) );
}

TEST_F(pdCZTest, SetAccVec)
{
  zVec3D v;

  SetVacuousPrm();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetAccVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZAccX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZAccY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZAccZ(&cz) );
}

TEST_F(pdCZTest, SetAccXYZ)
{
  SetVacuousPrm();
  pdCZSetAccX( &cz, 0.1 );
  pdCZSetAccY( &cz, 0.2 );
  pdCZSetAccZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZAccX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZAccY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZAccZ(&cz) );
}

TEST_F(pdCZTest, SetZMP)
{
  SetVacuousPrm();
  pdCZSetZMP( &cz, 0.1, 0.2, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZZMPX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZZMPY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZZMPZ(&cz) );
}

TEST_F(pdCZTest, SetZMPVec)
{
  zVec3D v;

  SetVacuousPrm();
  zVec3DCreate( &v, 0.1, 0.2, 0.3 );
  pdCZSetZMPVec( &cz, &v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZZMPX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZZMPY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZZMPZ(&cz) );
}

TEST_F(pdCZTest, SetZMPXYZ)
{
  SetVacuousPrm();
  pdCZSetZMPX( &cz, 0.1 );
  pdCZSetZMPY( &cz, 0.2 );
  pdCZSetZMPZ( &cz, 0.3 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZZMPX(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZZMPY(&cz) );
  EXPECT_DOUBLE_EQ( 0.3, pdCZZMPZ(&cz) );
}

TEST_F(pdCZTest, SetTheta)
{
  SetVacuousPrm();
  pdCZSetTheta( &cz, 0.1 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZTheta(&cz) );
}

TEST_F(pdCZTest, SetRefPosUW)
{
  SetVacuousPrm();
  pdCZSetRefPosUW( &cz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZRefPosU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZRefPosW(&cz) );
}

TEST_F(pdCZTest, SetRefPosUWVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.1, 0.2 );
  pdCZSetRefPosUWVec( &cz, v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZRefPosU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZRefPosW(&cz) );
}

TEST_F(pdCZTest, SetRefPosUPosW)
{
  SetVacuousPrm();
  pdCZSetRefPosU( &cz, 0.1 );
  pdCZSetRefPosW( &cz, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZRefPosU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZRefPosW(&cz) );
}

TEST_F(pdCZTest, SetDelta)
{
  SetVacuousPrm();
  pdCZSetDelta( &cz, 0.1, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZDeltaU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZDeltaW(&cz) );
}

TEST_F(pdCZTest, SetDeltaVec)
{
  zVec2D v;

  SetVacuousPrm();
  zVec2DCreate( v, 0.1, 0.2 );
  pdCZSetDeltaVec( &cz, v );
  EXPECT_DOUBLE_EQ( 0.1, pdCZDeltaU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZDeltaW(&cz) );
}

TEST_F(pdCZTest, SetDeltaUW)
{
  SetVacuousPrm();
  pdCZSetDeltaU( &cz, 0.1 );
  pdCZSetDeltaW( &cz, 0.2 );
  EXPECT_DOUBLE_EQ( 0.1, pdCZDeltaU(&cz) );
  EXPECT_DOUBLE_EQ( 0.2, pdCZDeltaW(&cz) );
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
  //EXPECT_DOUBLE_EQ( -0.001854072725677786536574, pdCZZMPRad( &cz ) );
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
