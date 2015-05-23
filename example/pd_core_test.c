#include <pedi2/pd_cmd.h>
#include <pedi2/pd_core.h>

/* #define DEBUG_MODE */

void init_mighty(pdCore *ctrl, pdCmd *cmd)
{
  /* load mighty */
  if( !pdCoreLoad( ctrl, "model/mighty.zkc" ) )
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

void init_hydra(pdCore *ctrl, pdCmd *cmd)
{
  /* load hydra */
  if( !pdCoreLoad( ctrl, "model/hydra.zkc" ) )
    exit( EXIT_FAILURE );
  /* set referential values */
  cmd->zd = 0.85;
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
  pdCore ctrl;
  zVec dis;
  register int i;

  /* initialization */
  pdCmdDefaultInit( &cmd );
  pdCoreInit( &ctrl, &cmd, DT );

  /* parse arguments */
  if( argc > 1 ){
    if( !strcmp( argv[1], "mighty" ) )
      init_mighty( &ctrl, &cmd );
    else if( !strcmp( argv[1], "hydra" ) )
      init_hydra( &ctrl, &cmd );
    else{
      ZRUNERROR( "invalid argument: %s", argv[1] );
      exit( EXIT_FAILURE );
    }
  } else
    init_mighty( &ctrl, &cmd );

  /* allocate displacement vector */
  dis = zVecAlloc( pdCoreJointSize(&ctrl) );

  /* main loop */
  for( i=0; i<STEP; i++ ){
    cmd.vud = 0.1;
    cmd.kappa = 0.0;

    /* update */
    pdCoreUpdate( &ctrl );

#ifdef DEBUG_MODE
    pdCZVrtWrite( pdCZVrtPtr( pdCoreCZPtr(&ctrl) ) );
    pdCZHrzUWWrite( pdCZHrzUWPtr( pdCZHrzPtr( pdCoreCZPtr(&ctrl) ) ) );
    pdCZHrzWrite( pdCZHrzPtr( pdCoreCZPtr(&ctrl) ) );
    pdCZWrite( pdCoreCZPtr(&ctrl) );
    pdFootWrite( pdCoreLFPtr(&ctrl), pdCoreRFPtr(&ctrl) );
    pdRobotWrite( pdCoreRobotPtr(&ctrl) );
    getchar();
#endif

    /* output */
    zVecCopy( pdCoreJointDis( &ctrl ), dis );
#ifndef DEBUG_MODE
    printf( "%f ", DT );zVecWrite( dis );
#endif
  }

  /* destroy */
  pdCoreDestroy( &ctrl );
  pdCmdDestroy( &cmd );
  return 0;
}
