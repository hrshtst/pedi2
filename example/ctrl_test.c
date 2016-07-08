#include <pedi2/pd_cmd.h>
#include <pedi2/pd_state.h>
#include <pedi2/pd_biped.h>
#include <pedi2/pd_robot.h>

/* #define DEBUG_MODE */
#define OUTPUT_PLOT_DATA

void init_mighty(pdRobot *robot, pdCmd *cmd)
{
  /* load mighty */
  if( !pdRobotLoad( robot, "model/mighty.zkc" ) )
    exit( EXIT_FAILURE );
  /* set referential values */
  cmd->zd = 0.26;
  cmd->dist = 0.084;
  cmd->lfh = 0.02;
  cmd->rfh = 0.02;
  cmd->vud = 0.0;
  cmd->kappa = 0.0;
  /* visualize the motion by executing the following command */
  /*   rk_anim model/mighty.zkc motion.zvs -pan -- -90 -x 0.4 -y -- -6 -z 0.3 */
}

void init_hydra(pdRobot *robot, pdCmd *cmd)
{
  /* load hydra */
  if( !pdRobotLoad( robot, "model/hydra.zkc" ) )
    exit( EXIT_FAILURE );
  /* set referential values */
  cmd->zd = 0.95;
  cmd->dist = 0.25;
  cmd->lfh = 0.1;
  cmd->rfh = 0.1;
  cmd->vud = 0.0;
  cmd->kappa = 0.0;
  /* visualize the motion by executing the following command */
  /*   $ rk_anim model/hydra.zkc motion.zvs -pan -- -90 -x 0 -y -- -15 -z 1 */
}

#define DT   0.01
#define STEP 1000
int main(int argc, char *argv[])
{
  pdCmd cmd;
  pdState state;
  pdBiped biped;
  pdRobot robot;
  zVec dis;
  FILE *fp;
  register int i;

  /* initialization */
  pdCmdDefaultInit( &cmd );
  pdStateInit( &state );
  pdBipedInit( &biped, &cmd, DT );
  pdRobotInit( &robot );

  /* parse arguments */
  if( argc > 1 ){
    if( !strcmp( argv[1], "mighty" ) )
      init_mighty( &robot, &cmd );
    else if( !strcmp( argv[1], "hydra" ) )
      init_hydra( &robot, &cmd );
    else{
      ZRUNERROR( "invalid argument: %s", argv[1] );
      exit( EXIT_FAILURE );
    }
  } else
    init_mighty( &robot, &cmd );

  /* allocate displacement vector */
  dis = zVecAlloc( pdRobotJointSize(&robot) );

  /* initialize robot state */
  pdRobotDefaultBipedInit( &robot, &biped, &state );

  /* output data for plotting */
  fp = fopen( "data.log", "w" );

  /* main loop */
  for( i=0; i<STEP; i++ ){
    cmd.vud = 0.1;
    cmd.kappa = 0.0;

    /* update */
    pdBipedUpdate( &biped, &state );
    pdRobotSetBipedRefVec( &robot, &biped );
    pdRobotSolveIK( &robot, 0 );

#ifdef DEBUG_MODE
    pdCZVrtWrite( pdCZVrtPtr( pdBipedCZPtr(&biped) ) );
    pdCZHrzUWWrite( pdCZHrzUWPtr( pdCZHrzPtr( pdBipedCZPtr(&biped) ) ) );
    pdCZHrzWrite( pdCZHrzPtr( pdBipedCZPtr(&biped) ) );
    pdCZWrite( pdBipedCZPtr(&biped) );
    pdFootWrite( pdBipedLFPtr(&biped), pdBipedRFPtr(&biped) );
    pdRobotWrite( &robot );
    getchar();
#endif

    /* output */
    zVecCopy( pdRobotJointDis( &robot ), dis );
#ifndef DEBUG_MODE
    printf( "%f ", DT );zVecWrite( dis );
#endif
#ifdef OUTPUT_PLOT_DATA
    pdBipedDataFWrite( fp, &biped );
#endif

    /* update state */
    pdBipedUpdateState( &biped, &state );
    pdRobotUpdateState( &robot, &state );
  }

  /* destroy */
  fclose( fp );
  pdRobotDestroy( &robot );
  pdBipedDestroy( &biped );
  pdStateDestroy( &state );
  pdCmdDestroy( &cmd );
  return 0;
}
