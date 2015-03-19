#include "dm_robot.h"

/* robot kinematics/dynamics model */
rkChain dm_robot;

/* IK solver */
static rkIK dm_ik;
static rkIKCell *dm_cell[6];
static zVec dm_dis;

/* IK seeds */
zVec3D dm_d_com, dm_d_lf, dm_d_rf;
zVec3D dm_d_att_body, dm_d_att_lf, dm_d_att_rf;

/* supporting region */

#define DM_TOL (1.0e-3)

#define DM_ROBOT_BODY       0
#define DM_ROBOT_LEFT_FOOT  9
#define DM_ROBOT_RIGHT_FOOT 15

zVec3DList dm_sr_lf; /* supporting region on the left sole */
zVec3DList dm_sr_rf; /* supporting region on the right sole */
zVec3DList dm_sr; /* supporting region */
/* contact points */
static zVec3D dm_sr_lf_vert[4], dm_sr_rf_vert[4], dm_sr_vert[8];

/* scene stuff */
static glrkChain dm_gl_robot;
static int dm_gl_gauge;
static zOpticalInfo dm_gl_sr_oi;

void dmRobotInit(void)
{
  GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
  register int i;

  /* robot model */
  if( !rkChainReadFile( &dm_robot, (char *)"hydra.zkc" ) )
    exit( EXIT_FAILURE );
  zVec3DCopy( rkChainWldCOM(&dm_robot), &dm_d_com );
  zVec3DCopy( rkChainLinkWldPos(&dm_robot,DM_ROBOT_LEFT_FOOT), &dm_d_lf );
  zVec3DCopy( rkChainLinkWldPos(&dm_robot,DM_ROBOT_RIGHT_FOOT), &dm_d_rf );
  zMat3DToZYX( rkChainLinkWldAtt(&dm_robot,DM_ROBOT_BODY), &dm_d_att_body );
  zMat3DToZYX( rkChainLinkWldAtt(&dm_robot,DM_ROBOT_LEFT_FOOT), &dm_d_att_lf );
  zMat3DToZYX( rkChainLinkWldAtt(&dm_robot,DM_ROBOT_RIGHT_FOOT), &dm_d_att_rf );

  /* supporting region */
  zListInit( &dm_sr_lf );
  zListInit( &dm_sr_rf );
  zListInit( &dm_sr );

  /* IK solver */
  if( !rkIKCreate( &dm_ik, &dm_robot ) )
    exit( EXIT_FAILURE );
  rkIKConfReadFile( &dm_ik, (char *)"hydra_ik.conf" );
  for( i=0; i<6; i++ )
    dm_cell[i] = rkIKFindCell( &dm_ik, i );

  /* joint displacement vector */
  if( !( dm_dis = zVecAlloc( rkChainJointSize(&dm_robot) ) ) )
    exit( EXIT_FAILURE );

  /* scene stuff */
  if( !glrkChainLoad( &dm_gl_robot, &dm_robot, NULL ) ){
    ZRUNERROR( "unable to load robot model" );
    exit( EXIT_FAILURE );
  }
  dm_gl_gauge = glrkGauge( zX, 6.0, zY, 6.0, 1.0, 0.2, white );
  zOpticalInfoCreateSimple( &dm_gl_sr_oi, 1.0, 0.4, 0.8, NULL );
}

void dmRobotExit(void)
{
  glrkChainUnload( &dm_gl_robot );
  glDeleteLists( dm_gl_gauge, 1 );
  zVecFree( dm_dis );
  rkIKDestroy( &dm_ik );
  zVec3DListDestroy( &dm_sr_lf, false );
  zVec3DListDestroy( &dm_sr_rf, false );
  zVec3DListDestroy( &dm_sr, false );
  rkChainDestroy( &dm_robot );
}

void dmRobotSolveIK(void)
{
  rkIKDeactivate( &dm_ik );
  rkIKCellSetRefVec( dm_cell[0], &dm_d_com );
  rkIKCellSetRefVec( dm_cell[1], &dm_d_att_body );
  rkIKCellSetRefVec( dm_cell[2], &dm_d_lf );
  rkIKCellSetRefVec( dm_cell[3], &dm_d_att_lf );
  rkIKCellSetRefVec( dm_cell[4], &dm_d_rf );
  rkIKCellSetRefVec( dm_cell[5], &dm_d_att_rf );
  rkIKSolve( &dm_ik, dm_dis, zTOL, 0 );
}

void dmSupportRegion(void)
{
  int i, nl, nr, n;
  rkLink *foot;
  zShape3D *sole;
  zVec3D v;

  nl = nr = n = 0;
  /* left foot */
  foot = rkChainLink( &dm_robot, DM_ROBOT_LEFT_FOOT );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < DM_TOL ){
      zVec3DCopy( &v, &dm_sr_lf_vert[nl++] );
      zVec3DCopy( &v, &dm_sr_vert[n++] );
    }
  }
  /* right foot */
  foot = rkChainLink( &dm_robot, DM_ROBOT_RIGHT_FOOT );
  sole = zListHead( rkLinkShapeList(foot) )->data;
  for( i=0; i<4; i++ ){
    zXfer3D( rkLinkWldFrame(foot), zShape3DVert(sole,i), &v );
    if( zVec3DElem(&v,zZ) < DM_TOL ){
      zVec3DCopy( &v, &dm_sr_rf_vert[nr++] );
      zVec3DCopy( &v, &dm_sr_vert[n++] );
    }
  }
  /* supporting region */
  zVec3DListDestroy( &dm_sr_lf, false );
  zVec3DListDestroy( &dm_sr_rf, false );
  zVec3DListDestroy( &dm_sr, false );
  if( nl > 0 ) zCH2D( &dm_sr_lf, dm_sr_lf_vert, nl );
  if( nr > 0 ) zCH2D( &dm_sr_rf, dm_sr_rf_vert, nr );
  if( n  > 0 ) zCH2D( &dm_sr, dm_sr_vert, n );
}

void dmSupportRegionBorder(double *xmin, double *xmax, double *ymin, double *ymax)
{
  zVec3DListCell *vc;
  double x, y;

  if( zListIsEmpty(&dm_sr) ) return;
  *xmin = *xmax = zVec3DInnerProd( Z_UNITXVEC3D, zListTail(&dm_sr)->data );
  *ymin = *ymax = zVec3DInnerProd( Z_UNITYVEC3D, zListTail(&dm_sr)->data );
  zListForEach( &dm_sr, vc ){
    x = zVec3DInnerProd( Z_UNITXVEC3D, vc->data );
    if( x < *xmin ) *xmin = x;
    if( x > *xmax ) *xmax = x;
    y = zVec3DInnerProd( Z_UNITYVEC3D, vc->data );
    if( y < *ymin ) *ymin = y;
    if( y > *ymax ) *ymax = y;
  }
}

/* dummy --- should handle 2D phase estimation */
void dmRobotFootPos(zVec3D *lf, zVec3D *rf)
{
  zVec3DCopy( rkChainLinkWldPos(&dm_robot,DM_ROBOT_LEFT_FOOT), lf );
  zVec3DCopy( rkChainLinkWldPos(&dm_robot,DM_ROBOT_RIGHT_FOOT), rf );
}

void dmRobotFootAtt(zVec3D *lf, zVec3D *rf)
{
  zMat3DToZYX( rkChainLinkWldAtt(&dm_robot,DM_ROBOT_LEFT_FOOT), lf );
  zMat3DToZYX( rkChainLinkWldAtt(&dm_robot,DM_ROBOT_RIGHT_FOOT), rf );
}

void dmRobotFootRegion(double *ylout, double *ylin, double *dyl, double *yrout, double *yrin, double *dyr)
{
  *ylout = zVec3DElem(rkChainLinkWldPos(&dm_robot,DM_ROBOT_LEFT_FOOT),zY) + 0.035;
  *ylin  = zVec3DElem(rkChainLinkWldPos(&dm_robot,DM_ROBOT_LEFT_FOOT),zY) - 0.035;
  *yrout = zVec3DElem(rkChainLinkWldPos(&dm_robot,DM_ROBOT_RIGHT_FOOT),zY) - 0.035;
  *yrin  = zVec3DElem(rkChainLinkWldPos(&dm_robot,DM_ROBOT_RIGHT_FOOT),zY) + 0.035;
  *dyl = *ylout > *ylin ? 1 : -1;
  *dyr = *yrout > *yrin ? 1 : -1;
}

/* scene */

void dmGLGauge(void)
{
  glDisable( GL_LIGHTING );
  glCallList( dm_gl_gauge );
  glEnable( GL_LIGHTING );
}

void dmGLRobot(void)
{
  glrkChainDraw( &dm_gl_robot );
}

void dmGLSupportRegion(void)
{
  zVec3DListCell *vc;

  glBegin( GL_POLYGON );
    glShadeModel( GL_FLAT );
    glrkMaterial( &dm_gl_sr_oi );
    glrkNormal( Z_UNITZVEC3D );
    zListForEachRew( &dm_sr, vc )
      glrkVertex( vc->data );
  glEnd();
}
