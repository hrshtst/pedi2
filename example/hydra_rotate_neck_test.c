#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_robot.h>

#define NECK_P_ID 15
#define NECK_P_MIN 0
#define NECK_P_MAX 120
void move_neck(int step, zVec v)
{
  double deg;
  double amp;

  amp = 0.5 * ( NECK_P_MAX - NECK_P_MIN );
  deg = - amp * cos(0.005*zPIx2*step) + amp;
  zVecSetElem( v, 0, zDeg2Rad(deg) );
}

#define DT   0.01
#define STEP 1000
int main(void)
{
  pdCmd cmd;      /* user-defined command values */
  pdState state;  /* robot state */
  pdBiped biped;  /* bipdal locomotion controller */
  pdRobot robot;  /* robot model instance */
  zVec dis;       /* joint displacement vector */

  zIndex idx;     /* for neck movement */
  zVec v;         /* for neck movement */
  register int i;

  /* initialization */
  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );

  /* load kinematics/dynamics model file (robot.zkc) */
  if( !pdRobotLoad( &robot, "model/hydra.zkc" ) )
    exit( EXIT_FAILURE );

  /* prepare joint displacement vector */
  dis = zVecAlloc( pdRobotJointSize(&robot) );

  /* set referential values */
  cmd.zd = 0.95;           /* COM height */
  cmd.dist = 0.25;         /* distance of both feet */
  cmd.lfh = cmd.rfh = 0.1; /* (maximal) foot lifting height */
  cmd.vud = 0.0;           /* referential velocity (longitudinal) */
  cmd.kappa = 0.0;         /* referential curvature for rotation */

  /* initialize robot state */
  pdRobotDefaultBipedInit( &robot, &biped, &state );

  /* for neck movement */
  idx = zIndexCreate( 1 );
  v   = zVecAlloc( 1 );

  /* unregister neck joint */
  zIndexSetElem( idx, 0, NECK_P_ID );
  pdRobotJointUnregIndex( &robot, idx );

  /* main loop */
  for( i=0; i<STEP; i++ ){
    /* modify command values */
    cmd.rho = 1.0;
    cmd.vud = 0.1;
    cmd.kappa = 0.0;

    /* neck movement */
    move_neck( i, v );
    pdRobotSetJointDis( &robot, idx, v );

    /* update controller */
    pdBipedUpdate( &biped, &state );
    pdRobotSetBipedRefVec( &robot, &biped );
    pdRobotSolveIK( &robot, 0 );

    /* output */
    /* obtain desired joint displacement as a zVec instance */
    zVecCopy( pdRobotJointDis( &robot ), dis );
    /* you can visualize the motion by executing the following command, e.g. */
    /*   $ rk_anim model/hydra.zkc motion.zvs -pan -- -90 -x 0 -y -- -15 -z 1 */
    printf( "%f ", DT );zVecWrite( dis );

    /* update state */
    pdBipedUpdateState( &biped, &state );
    pdRobotUpdateState( &robot, &state );
  }

  /* destroy */
  zIndexFree( idx );
  zVecFree( v );
  pdRobotDestroy( &robot );
  pdBipedDestroy( &biped );
  pdStateDestroy( &state );
  pdCmdDestroy( &cmd );
  return 0;
}
