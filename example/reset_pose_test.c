#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_robot.h>

#define DT   0.01
#define STEP 1000
void init(pdCmd *cmd, pdBiped *biped, pdState *state, pdRobot *robot)
{
  pdCmdDefaultInit( cmd );
  pdStateInit( state );
  pdBipedInit( biped, cmd, DT );
  pdRobotInit( robot );

  /* load kinematics/dynamics model file (robot.ztk) */
  if( !pdRobotLoad( robot, "model/hydra.ztk" ) )
    exit( EXIT_FAILURE );

  /* initialize robot state */
  pdRobotBipedDefaultInit( robot, biped, state );
}

void default_cmd(pdCmd *cmd)
{
  /* set referential values */
  cmd->zd = 0.95;           /* COM height */
  cmd->dist = 0.25;         /* distance of both feet */
  cmd->lfh = cmd->rfh = 0.1; /* (maximal) foot lifting height */
  cmd->vud = 0.0;           /* referential velocity (longitudinal) */
  cmd->kappa = 0.0;         /* referential curvature for rotation */
}

void update_cmd_walk(pdCmd *cmd, int step)
{
  /* modify command values */
  if( step > 200 )
    cmd->vud = 0.1;
}

void update_controller(pdBiped *biped, pdState *state, pdRobot *robot)
{
  pdBipedUpdate( biped, state );
  pdRobotBipedSetRefVec( robot, biped );
  pdRobotSolveIK( robot, 0 );
}

void update_state(pdBiped *biped, pdState *state, pdRobot *robot)
{
  pdBipedUpdateState( biped, state );
  pdRobotUpdateState( robot, state );
}

void destroy(pdCmd *cmd, pdBiped *biped, pdState *state, pdRobot *robot)
{
  pdRobotDestroy( robot );
  pdBipedDestroy( biped );
  pdStateDestroy( state );
  pdCmdDestroy( cmd );
}


int main(void)
{
  pdCmd cmd;      /* user-defined command values */
  pdState state;  /* robot state */
  pdBiped biped;  /* bipedal locomotion controller */
  pdRobot robot;  /* robot model instance */
  zVec dis;       /* joint displacement vector */
  register int i;

  /* initialization */
  init( &cmd, &biped, &state, &robot );
  /* prepare joint displacement vector */
  dis = zVecAlloc( pdRobotJointSize(&robot) );
  /* set default command */
  default_cmd( &cmd );

  zEchoOff();
  /* main loop */
  for( i=0; i<STEP; i++ ){
    /* update command */
    update_cmd_walk( &cmd, i );
    /* update controller */
    update_controller( &biped, &state, &robot );
    /* output */
    zVecCopy( pdRobotJointDis( &robot ), dis );
    /* you can visualize the motion by executing the following command, e.g. */
    /*   $ rk_anim model/hydra.ztk motion.zvs -pan -- -90 -x 0 -y -- -15 -z 1 */
    printf( "%f ", DT );zVecPrint( dis );
    /* update state */
    update_state( &biped, &state, &robot );
  }

  zVec v;
  FILE *fp;
  const char init_pose_file[] = "data/init.pose.zv";
  if( !( fp = fopen( init_pose_file, "r" ) ) ){
    ZOPENERROR( init_pose_file );
    return EXIT_FAILURE;
  }
  if( !( v = zVecFScan( fp ) ) ){
    ZALLOCERROR();
    return EXIT_FAILURE;
  }
  if( fp ) fclose( fp );
  pdRobotBipedResetPose( &robot, &biped, &state, v );
  zVecFree( v );

  for( i=0; i<STEP; i++ ){
    /* update command */
    update_cmd_walk( &cmd, i );
    /* update controller */
    update_controller( &biped, &state, &robot );
    /* output */
    zVecCopy( pdRobotJointDis( &robot ), dis );
    /* you can visualize the motion by executing the following command, e.g. */
    /*   $ rk_anim model/hydra.ztk motion.zvs -pan -- -90 -x 0 -y -- -15 -z 1 */
    printf( "%f ", DT );zVecPrint( dis );
    /* update state */
    update_state( &biped, &state, &robot );
  }

  /* destroy */
  destroy( &cmd, &biped, &state, &robot );
  return 0;
}
