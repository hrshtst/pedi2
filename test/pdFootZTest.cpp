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
    pdFootZSign( &lf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &lf ), 10, 20 );
    pdFootZFootPhase( &lf ) = 30;
    pdFootZRefZ( &lf ) = 40;
    lf._vert_num = 50;

    pdFootZCZPtr( &rf ) = NULL;
    pdFootZSetMaxHeight( &rf, 100 );
    pdFootZSign( &rf ) = 0.5;
    zComplexCreate( pdFootZZMPPhase( &rf ), 10, 20 );
    pdFootZFootPhase( &rf ) = 30;
    pdFootZRefZ( &rf ) = 40;
    rf._vert_num = 50;
  }

  // zVec3DList lsr, rsr;
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
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 0, zArraySize( &pdFootZSRVert(&lf)->data.array ) );
  EXPECT_EQ( NULL, zArrayBuf( &pdFootZSRVert(&lf)->data.array ) );
  EXPECT_EQ( 0, lf._vert_num );
  EXPECT_EQ( 0, pdFootZSign( &lf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &lf ) );

  pdFootZInit( &rf, &czuw );
  EXPECT_EQ( &czuw, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &rf ) );
  EXPECT_EQ( 0, zArraySize( &pdFootZSRVert(&rf)->data.array ) );
  EXPECT_EQ( NULL, zArrayBuf( &pdFootZSRVert(&rf)->data.array ) );
  EXPECT_EQ( 0, rf._vert_num );
  EXPECT_EQ( 0, pdFootZSign( &rf ) );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &rf ) );
}

TEST_F(pdFootZTest, Destroy)
{
  SetVacuousPrm();
  pdFootZInit( &lf, &czuw );
  pdFootZDestroy( &lf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &lf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &lf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 0, zArraySize( &pdFootZSRVert(&lf)->data.array ) );
  EXPECT_EQ( NULL, zArrayBuf( &pdFootZSRVert(&lf)->data.array ) );
  EXPECT_EQ( 0, lf._vert_num );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &lf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &lf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &lf ) );

  pdFootZInit( &rf, &czuw );
  pdFootZDestroy( &rf );
  EXPECT_EQ( NULL, pdFootZCZPtr( &rf ) );
  EXPECT_EQ( 0, pdFootZMaxHeight( &rf ) );
  EXPECT_FALSE( pdFootZIsSRSet( &rf ) );
  EXPECT_EQ( 0, zArraySize( &pdFootZSRVert(&rf)->data.array ) );
  EXPECT_EQ( NULL, zArrayBuf( &pdFootZSRVert(&rf)->data.array ) );
  EXPECT_EQ( 0, rf._vert_num );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->re );
  EXPECT_EQ( 0, pdFootZZMPPhase( &rf )->im );
  EXPECT_EQ( 0, pdFootZFootPhase( &rf ) );
  EXPECT_EQ( 0, pdFootZRefZ( &rf ) );
}

TEST_F(pdFootZTest, ReferRho)
{
  EXPECT_EQ( 0, pdFootZRho( &lf ) );
  pdCZHrzUWSetRho( &czuw, 1 );
  EXPECT_EQ( 1, pdFootZRho( &lf ) );
  pdCZHrzUWSetRho( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootZRho( &lf ) );
}

TEST_F(pdFootZTest, ReferDist)
{
  EXPECT_EQ( 0, pdFootZDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 1 );
  EXPECT_EQ( 1, pdFootZDist( &lf ) );
  pdCZHrzUWSetDist( &czuw, 0.2 );
  EXPECT_EQ( 0.2, pdFootZDist( &lf ) );
}

TEST_F(pdFootZTest, SetSR)
{
  zVec3D v[3];
  zVec3D vv[4];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_EQ( 0, zArrayElem( &pdFootZSRVert(&lf)->data.array, 0 )->c.x );
  EXPECT_EQ( 0, zArrayElem( &pdFootZSRVert(&lf)->data.array, 0 )->c.y );
  EXPECT_EQ( 0, zArrayElem( &pdFootZSRVert(&lf)->data.array, 1 )->c.x );
  EXPECT_EQ( 1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 1 )->c.y );
  EXPECT_EQ( 1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 2 )->c.x );
  EXPECT_EQ( 1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 2 )->c.y );
  EXPECT_EQ( 3, lf._vert_num );

  zVec3DCreate( &v[0], 0.1, 0.1, 0 );
  zVec3DCreate( &v[1], 0.1, 0.2, 0 );
  zVec3DCreate( &v[2], 0.2, 0.2, 0 );
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 0 )->c.x );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 0 )->c.y );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 1 )->c.x );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 1 )->c.y );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 2 )->c.x );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 2 )->c.y );
  EXPECT_EQ( 3, lf._vert_num );

  zVec3DCreate( &vv[0], 0.1, 0.1, 0 );
  zVec3DCreate( &vv[1], 0.1, 0.2, 0 );
  zVec3DCreate( &vv[2], 0.2, 0.2, 0 );
  zVec3DCreate( &vv[3], 0.2, 0.1, 0 );
  pdFootZSetSR( &lf, vv, 4 );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 0 )->c.x );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 0 )->c.y );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 1 )->c.x );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 1 )->c.y );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 2 )->c.x );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 2 )->c.y );
  EXPECT_EQ( 0.2, zArrayElem( &pdFootZSRVert(&lf)->data.array, 3 )->c.x );
  EXPECT_EQ( 0.1, zArrayElem( &pdFootZSRVert(&lf)->data.array, 3 )->c.y );
  EXPECT_EQ( 4, lf._vert_num );
}

TEST_F(pdFootZTest, SetSR_chk_memory)
{
  zVec3D v1[3];
  zVec3D v2[4];
  zVec3D v3[4];
  zVec3D *p1;
  zVec3D *p2;
  zVec3D *p3;

  zVec3DCreate( &v1[0], 0, 0, 0 );
  zVec3DCreate( &v1[1], 0, 1, 0 );
  zVec3DCreate( &v1[2], 1, 1, 0 );
  zVec3DCreate( &v2[0], 0, 0, 0 );
  zVec3DCreate( &v2[1], 0, 1, 0 );
  zVec3DCreate( &v2[2], 1, 1, 0 );
  zVec3DCreate( &v2[3], 1, 0, 0 );
  zVec3DCreate( &v3[0], 0, 0, 0 );
  zVec3DCreate( &v3[1], 0, 1, 0 );
  zVec3DCreate( &v3[2], 1, 1, 0 );
  zVec3DCreate( &v3[3], 1, 0, 0 );

  EXPECT_EQ( 0, zArraySize( &pdFootZSRVert(&lf)->data.array ) );
  EXPECT_EQ( NULL, zArrayBuf( &pdFootZSRVert(&lf)->data.array ) );
  pdFootZSetSR( &lf, v1, 3 );
  p1 = zArrayBuf( &pdFootZSRVert(&lf)->data.array );
  EXPECT_TRUE( NULL != p1 );

  pdFootZSetSR( &lf, v2, 4 );
  p2 = zArrayBuf( &pdFootZSRVert(&lf)->data.array );
  EXPECT_NE( p1, p2 );

  pdFootZSetSR( &lf, v3, 4 );
  p3 = zArrayBuf( &pdFootZSRVert(&lf)->data.array );
  EXPECT_EQ( p2, p3 );

  pdFootZSetSR( &lf, v1, 3 );
  p1 = zArrayBuf( &pdFootZSRVert(&lf)->data.array );
  EXPECT_NE( p3, p1 );
}

TEST_F(pdFootZTest, IsSRSet)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_TRUE( pdFootZIsSRSet( &lf ) );
}

TEST_F(pdFootZTest, SetSR_None)
{
  zVec3D v[3];

  zVec3DCreate( &v[0], 0, 0, 0 );
  zVec3DCreate( &v[1], 0, 1, 0 );
  zVec3DCreate( &v[2], 1, 1, 0 );
  // set supporting region
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_TRUE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 3, lf._vert_num );
  // set None
  pdFootZSetSR( &lf, NULL, 0 );
  EXPECT_FALSE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 0, lf._vert_num );
  // set supporting region again
  pdFootZSetSR( &lf, v, 3 );
  EXPECT_TRUE( pdFootZIsSRSet( &lf ) );
  EXPECT_EQ( 3, lf._vert_num );
}

TEST_F(pdFootZTest, FindIntersection)
{
  zVec3D v[4], ip[2];
  zVec2D zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );

  // no intersections
  zVec2DCreate( &zmp, 0.05, 0.07 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // share an edge
  zVec2DCreate( &zmp, 0.04, 0.07 );
  EXPECT_EQ( 2, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( 0.04, ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.1,  ip[0].c.y, 1e-12 );
  EXPECT_NEAR( 0.04, ip[1].c.x, 1e-12 );
  EXPECT_NEAR( 0.15, ip[1].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // two intersections
  zVec2DCreate( &zmp, 0.01, 0.07 );
  EXPECT_EQ( 2, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( 0.01, ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.1,  ip[0].c.y, 1e-12 );
  EXPECT_NEAR( 0.01, ip[1].c.x, 1e-12 );
  EXPECT_NEAR( 0.15, ip[1].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // two intersections, ZMP is inside SR
  zVec2DCreate( &zmp, -0.01, 0.12 );
  EXPECT_EQ( 2, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( -0.01, ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.1,   ip[0].c.y, 1e-12 );
  EXPECT_NEAR( -0.01, ip[1].c.x, 1e-12 );
  EXPECT_NEAR( 0.15,  ip[1].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );
}

TEST_F(pdFootZTest, FindIntersection2)
{
  zVec3D v[4], ip[2];
  zVec2D zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.0,  0.04, 0.0 );
  zVec3DCreate( &v[1],  0.04, 0.08, 0.0 );
  zVec3DCreate( &v[2],  0.0,  0.12, 0.0 );
  zVec3DCreate( &v[3], -0.04, 0.08, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );

  // no intersections
  zVec2DCreate( &zmp, 0.06, 0.04 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // one intersection
  zVec2DCreate( &zmp, 0.04, 0.04 );
  EXPECT_EQ( 1, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( 0.04, ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.08, ip[0].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // two intersections
  zVec2DCreate( &zmp, 0.02, 0.04 );
  EXPECT_EQ( 2, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( 0.02, ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.06, ip[0].c.y, 1e-12 );
  EXPECT_NEAR( 0.02, ip[1].c.x, 1e-12 );
  EXPECT_NEAR( 0.1,  ip[1].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // two intersections, intersect on vertices
  zVec2DCreate( &zmp, 0.0, 0.02 );
  EXPECT_EQ( 2, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( 0.0,  ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.04, ip[0].c.y, 1e-12 );
  EXPECT_NEAR( 0.0,  ip[1].c.x, 1e-12 );
  EXPECT_NEAR( 0.12, ip[1].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );

  // two intersections, ZMP is on a vertex
  zVec2DCreate( &zmp, 0.0, 0.04 );
  EXPECT_EQ( 2, pdFootZFindIntersection( &lf, &zmp, ip ) );
  EXPECT_NEAR( 0.0,  ip[0].c.x, 1e-12 );
  EXPECT_NEAR( 0.04, ip[0].c.y, 1e-12 );
  EXPECT_NEAR( 0.0,  ip[1].c.x, 1e-12 );
  EXPECT_NEAR( 0.12, ip[1].c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindIntersection( &rf, &zmp, ip ) );
}

TEST_F(pdFootZTest, FindInnerPoint_LeftFoot)
{
  zVec3D v[4], p;
  zVec2D zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );

  // no intersections
  zVec2DCreate( &zmp, 0.05, 0.07 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // share an edge
  zVec2DCreate( &zmp, 0.04, 0.07 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( 0.04, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.1,  p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // two intersections
  zVec2DCreate( &zmp, 0.01, 0.07 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( 0.01, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.1,  p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // two intersections, ZMP is inside SR
  zVec2DCreate( &zmp, -0.01, 0.12 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( -0.01, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.1,   p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
}

TEST_F(pdFootZTest, FindInnerPoint_LeftFoot2)
{
  zVec3D v[4], p;
  zVec2D zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.0,  0.04, 0.0 );
  zVec3DCreate( &v[1],  0.04, 0.08, 0.0 );
  zVec3DCreate( &v[2],  0.0,  0.12, 0.0 );
  zVec3DCreate( &v[3], -0.04, 0.08, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );

  // no intersections
  zVec2DCreate( &zmp, 0.06, 0.04 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // one intersection
  zVec2DCreate( &zmp, 0.04, 0.04 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( 0.04, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.08, p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // two intersections
  zVec2DCreate( &zmp, 0.02, 0.04 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( 0.02, p.c.x, 1e-12 );
  EXPECT_NEAR( 0.06, p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // two intersections, intersect on vertices
  zVec2DCreate( &zmp, 0.0, 0.02 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( 0.0,  p.c.x, 1e-12 );
  EXPECT_NEAR( 0.04, p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // two intersections, ZMP is on a vertex
  zVec2DCreate( &zmp, 0.0, 0.04 );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_NEAR( 0.0,  p.c.x, 1e-12 );
  EXPECT_NEAR( 0.04, p.c.y, 1e-12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
}

TEST_F(pdFootZTest, FindInnerPoint_RightFoot)
{
  zVec3D v[4], p;
  zVec2D zmp;

  // make convex hull of right foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &lf, NULL, 0 );
  pdFootZSetSR( &rf, v, 4 );

  // no intersections
  zVec2DCreate( &zmp, 0.05, 0.07 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // share an edge
  zVec2DCreate( &zmp, 0.04, 0.07 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( 0.04, p.c.x, 1e-12 );
  EXPECT_NEAR( -0.1, p.c.y, 1e-12 );

  // two intersections
  zVec2DCreate( &zmp, 0.01, 0.07 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( 0.01, p.c.x, 1e-12 );
  EXPECT_NEAR( -0.1, p.c.y, 1e-12 );

  // two intersections, ZMP is inside SR
  zVec2DCreate( &zmp, -0.01, 0.12 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( -0.01, p.c.x, 1e-12 );
  EXPECT_NEAR( -0.1,  p.c.y, 1e-12 );
}

TEST_F(pdFootZTest, FindInnerPoint_RightFoot2)
{
  zVec3D v[4], p;
  zVec2D zmp;

  // make convex hull of right foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.0,  -0.04, 0.0 );
  zVec3DCreate( &v[1],  0.04, -0.08, 0.0 );
  zVec3DCreate( &v[2],  0.0,  -0.12, 0.0 );
  zVec3DCreate( &v[3], -0.04, -0.08, 0.0 );
  pdFootZSetSR( &lf, NULL, 0 );
  pdFootZSetSR( &rf, v, 4 );

  // no intersections
  zVec2DCreate( &zmp, 0.06, 0.04 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &rf, &zmp, &p ) );

  // one intersection
  zVec2DCreate( &zmp, 0.04, 0.04 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( 0.04,  p.c.x, 1e-12 );
  EXPECT_NEAR( -0.08, p.c.y, 1e-12 );

  // two intersections
  zVec2DCreate( &zmp, 0.02, 0.04 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( 0.02,  p.c.x, 1e-12 );
  EXPECT_NEAR( -0.06, p.c.y, 1e-12 );

  // two intersections, intersect on vertices
  zVec2DCreate( &zmp, 0.0, 0.02 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( 0.0,   p.c.x, 1e-12 );
  EXPECT_NEAR( -0.04, p.c.y, 1e-12 );

  // two intersections, ZMP is on a vertex
  zVec2DCreate( &zmp, 0.0, 0.04 );
  EXPECT_EQ( 0, pdFootZFindInnerPoint( &lf, &zmp, &p ) );
  EXPECT_EQ( 1, pdFootZFindInnerPoint( &rf, &zmp, &p ) );
  EXPECT_NEAR( 0.0,   p.c.x, 1e-12 );
  EXPECT_NEAR( -0.04, p.c.y, 1e-12 );
}

TEST_F(pdFootZTest, CalcFootPhase_SingleSupportLeft)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );

  // Now ZMP is out of the supporting region,
  zVec2DCreate( &delta, 0, 0.05 );
  zVec2DCreate( &vel,   0, 0.05 );
  zVec2DCreate( &zmp,   0, 0.07 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // comes into the supporting region,
  zVec2DCreate( &delta, 0, 0.02 );
  zVec2DCreate( &vel,   0, 0.1 );
  zVec2DCreate( &zmp,   0, 0.1 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // comes at the most leftward,
  zVec2DCreate( &delta, 0, -0.02 );
  zVec2DCreate( &vel,   0, 0.0 );
  zVec2DCreate( &zmp,   0, 0.125 );
  EXPECT_NEAR( 0.5, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // get back to the edge again,
  zVec2DCreate( &delta, 0, 0.02 );
  zVec2DCreate( &vel,   0, -0.1 );
  zVec2DCreate( &zmp,   0, 0.1 );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // and go far away
  zVec2DCreate( &delta, 0, 0.05 );
  zVec2DCreate( &vel,   0, -0.05 );
  zVec2DCreate( &zmp,   0, 0.07 );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
}

TEST_F(pdFootZTest, CalcFootPhase_OrthogonalSRRight)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of right foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &lf, NULL, 0 );
  pdFootZSetSR( &rf, v, 4 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );

  // Now ZMP is out of the supporting region,
  zVec2DCreate( &delta, 0, -0.05 );
  zVec2DCreate( &vel,   0, -0.05 );
  zVec2DCreate( &zmp,   0, -0.07 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // comes into the supporting region,
  zVec2DCreate( &delta, 0, -0.02 );
  zVec2DCreate( &vel,   0, -0.1 );
  zVec2DCreate( &zmp,   0, -0.1 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // comes at the most righttward,
  zVec2DCreate( &delta, 0, 0.02 );
  zVec2DCreate( &vel,   0, 0.0 );
  zVec2DCreate( &zmp,   0, -0.125 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0.5, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // get back to the edge again,
  zVec2DCreate( &delta, 0, -0.02 );
  zVec2DCreate( &vel,   0, 0.1 );
  zVec2DCreate( &zmp,   0, -0.1 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  // and go far away
  zVec2DCreate( &delta, 0, -0.05 );
  zVec2DCreate( &vel,   0, 0.05 );
  zVec2DCreate( &zmp,   0, -0.07 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 1, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
}

TEST_F(pdFootZTest, CalcFootPhase_DoubleSupport)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &rf, v, 4 );

  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );

  // always 0 or 1 when double support phase
  zVec2DCreate( &delta, 0, -0.05 );
  zVec2DCreate( &vel,   0, -0.02 );
  zVec2DCreate( &zmp,   0, 0.07 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
  //
  zVec2DCreate( &delta, 0, 0.05 );
  zVec2DCreate( &vel,   0, 0.02 );
  zVec2DCreate( &zmp,   0, -0.07 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &lf, &delta, &vel, &zmp ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZCalcFootPhase( &rf, &delta, &vel, &zmp ), 1e-12 );
}

TEST_F(pdFootZTest, CalcRefZ_RhoIsOne)
{
  double phase;
  zComplex pz;

  pdCZHrzUWSetRho( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 1 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  zComplexCreate( &pz, 0.5, 0 );

  phase = 0;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 0.5;
  EXPECT_NEAR( 0.8, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 1;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcRefZ_RhoIsLowerThanOne)
{
  double phase;
  zComplex pz;

  // e^-1 = 0.36787944117144233
  pdCZHrzUWSetRho( &czuw, 0.8 );
  pdCZHrzUWSetDist( &czuw, 1 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  zComplexCreate( &pz, 0.5, 0 );

  phase = 0;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 0.5;
  EXPECT_GT( pdFootZCalcRefZ( &lf, phase, &pz ), 0 );
  EXPECT_LT( pdFootZCalcRefZ( &lf, phase, &pz ), 0.8 );
  phase = 1;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, CalcRefZ_RhoIsNearlyZero)
{
  double phase;
  zComplex pz;

  // e^-1 = 0.36787944117144233
  pdCZHrzUWSetRho( &czuw, 0.2 );
  pdCZHrzUWSetDist( &czuw, 1 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  zComplexCreate( &pz, 0.5, 0 );

  phase = 0;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 0.5;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
  phase = 1;
  EXPECT_NEAR( 0, pdFootZCalcRefZ( &lf, phase, &pz ), 1e-12 );
}

TEST_F(pdFootZTest, Update_SingleSupportLeftIsOn)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of left foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, 0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, 0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, 0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, 0.15, 0.0 );
  pdFootZSetSR( &lf, v, 4 );
  pdFootZSetSR( &rf, NULL, 0 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );
  pdCZHrzUWSetRho( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 0.25 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  pdFootZSetMaxHeight( &rf, 0.8 );

  // Now ZMP is out of the left foot
  zVec2DCreate( &delta, 0, 0.05 );
  zVec2DCreate( &vel,   0, 0.05 );
  zVec2DCreate( &zmp,   0, 0.07 );
  pdFootZUpdate( &lf, &rf, &delta, &vel, &zmp );
  pdFootZUpdate( &rf, &lf, &delta, &vel, &zmp );
  EXPECT_NEAR( 0, pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &rf ), 1e-12 );

  // right foot can be lifted
  zVec2DCreate( &delta, 0, 0.0 );
  zVec2DCreate( &vel,   0, 0.0 );
  zVec2DCreate( &zmp,   0, 0.125 );
  pdFootZUpdate( &lf, &rf, &delta, &vel, &zmp );
  pdFootZUpdate( &rf, &lf, &delta, &vel, &zmp );
  EXPECT_NEAR( 0,   pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0,   pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0.5, pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0.8, pdFootZRefZ( &rf ), 1e-12 );
}

TEST_F(pdFootZTest, Update_SingleSupportRightIsOn)
{
  zVec3D v[4];
  zVec2D delta, vel, zmp;

  // make convex hull of right foot
  // orthogonal to the moving frame
  zVec3DCreate( &v[0],  0.04, -0.1,  0.0 );
  zVec3DCreate( &v[1], -0.04, -0.1,  0.0 );
  zVec3DCreate( &v[2], -0.04, -0.15, 0.0 );
  zVec3DCreate( &v[3],  0.04, -0.15, 0.0 );
  pdFootZSetSR( &lf, NULL, 0 );
  pdFootZSetSR( &rf, v, 4 );
  pdCZVrtZeta( &vrt ) = 2;
  pdCZHrzUWSetQ1W( &czuw, 1 );
  pdCZHrzUWSetQ2W( &czuw, 1 );
  pdCZHrzUWSetRho( &czuw, 1 );
  pdCZHrzUWSetDist( &czuw, 0.25 );
  pdFootZSetMaxHeight( &lf, 0.8 );
  pdFootZSetMaxHeight( &rf, 0.8 );

  // Now ZMP is out of the right foot
  zVec2DCreate( &delta, 0, -0.05 );
  zVec2DCreate( &vel,   0, -0.05 );
  zVec2DCreate( &zmp,   0, -0.07 );
  pdFootZUpdate( &lf, &rf, &delta, &vel, &zmp );
  pdFootZUpdate( &rf, &lf, &delta, &vel, &zmp );
  EXPECT_NEAR( 0, pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0, pdFootZRefZ( &rf ), 1e-12 );
  // left foot can be lifted
  zVec2DCreate( &delta, 0, 0.0 );
  zVec2DCreate( &vel,   0, 0.0 );
  zVec2DCreate( &zmp,   0, -0.125 );
  pdFootZUpdate( &lf, &rf, &delta, &vel, &zmp );
  pdFootZUpdate( &rf, &lf, &delta, &vel, &zmp );
  EXPECT_NEAR( 0.5, pdFootZFootPhase( &lf ), 1e-12 );
  EXPECT_NEAR( 0.8, pdFootZRefZ( &lf ), 1e-12 );
  EXPECT_NEAR( 0,   pdFootZFootPhase( &rf ), 1e-12 );
  EXPECT_NEAR( 0,   pdFootZRefZ( &rf ), 1e-12 );
}
