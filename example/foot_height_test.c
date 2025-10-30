#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_robot.h>

#define DT   0.01
#define STEP 1000
int main(void)
{
  pdCmd cmd;      /* user-defined command values */
  pdState state;  /* robot state */
  pdBiped biped;  /* bipedal locomotion controller */
  pdRobot robot;  /* robot model instance */
  zVec dis;       /* joint displacement vector */
  register int i;

  /* initialization */
  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );

  /* load kinematics/dynamics model file (robot.ztk) */
  if( !pdRobotLoad( &robot, "model/hydra.ztk" ) )
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
  pdRobotBipedDefaultInit( &robot, &biped, &state );

  /* main loop */
  for( i=0; i<STEP; i++ ){
    /* modify command values */
    /* cmd.rho = 1.0; */
    if( i > 0 && i < 530 )
      cmd.vud = 0.2;
    else if( i > 750 && i < 900 )
      cmd.vud = 0.1;
    else
      cmd.vud = 0.0;
    cmd.kappa = 0.0;

    /* update controller */
    pdBipedUpdate( &biped, &state );
    pdRobotBipedSetRefVec( &robot, &biped );
    pdRobotSolveIK( &robot, 0 );

    /* output */
    /* obtain desired joint displacement as a zVec instance */
    zVecCopy( pdRobotJointDis( &robot ), dis );
    /* you can visualize the motion by executing the following command, e.g. */
    /*   $ rk_anim model/hydra.ztk motion.zvs -pan -- -90 -x 0 -y -- -15 -z 1 */

    printf( "%f ", DT );zVecPrint( dis );
    /* if( i > 750 ){ */
    /*   pdFootFWrite( stderr, pdBipedLFPtr(&biped), pdBipedRFPtr(&biped) ); */
    /*   getchar(); */
    /* } */
    /* pdFootDataWrite( pdBipedLFPtr(&biped), pdBipedRFPtr(&biped) ); */

    /* update state */
    pdBipedUpdateState( &biped, &state );
    pdRobotUpdateState( &robot, &state );
  }

  /* destroy */
  pdRobotDestroy( &robot );
  pdBipedDestroy( &biped );
  pdStateDestroy( &state );
  pdCmdDestroy( &cmd );
  return 0;
}
