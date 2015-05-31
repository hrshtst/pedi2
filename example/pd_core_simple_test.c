#include <pedi2/pd_cmd.h>
#include <pedi2/pd_core.h>

#define DT   0.01
#define STEP 1000
int main(void)
{
  pdCmd cmd;    /* user-defined command values */
  pdCore ctrl;  /* core controller instance */
  zVec dis;     /* joint displacement vector */
  register int i;

  /* initialization */
  pdCmdDefaultInit( &cmd );
  pdCoreInit( &ctrl, &cmd, DT );

  /* load kinematics/dynamics model file (robot.zkc) */
  if( !pdCoreLoad( &ctrl, "model/hydra.zkc" ) )
    exit( EXIT_FAILURE );

  /* prepare joint displacement vector */
  dis = zVecAlloc( pdCoreJointSize(&ctrl) );

  /* set referential values */
  cmd.zd = 0.95;           /* COM height */
  cmd.dist = 0.25;         /* distance of both feet */
  cmd.lfh = cmd.rfh = 0.1; /* (maximal) foot lifting height */
  cmd.vud = 0.0;           /* referential velocity (longitudinal) */
  cmd.kappa = 0.0;         /* referential curvature for rotation */

  /* main loop */
  for( i=0; i<STEP; i++ ){
    /* modify command values */
    cmd.rho = 1.0;
    if( i > 200 )
      cmd.vud = 0.1;
    cmd.kappa = 0.0;

    /* feedback the current state */
    /* if not given, the controller will automatically update it */
    /* pdCoreSetState( &ctrl, x, y ); */

    /* update */
    pdCoreUpdate( &ctrl );

    /* output */
    /* obtain desired joint displacement as a zVec instance */
    zVecCopy( pdCoreJointDis( &ctrl ), dis );
    /* you can visualize the motion by executing the following command, e.g. */
    /*   $ rk_anim model/hydra.zkc motion.zvs -pan -- -90 -x 0 -y -- -15 -z 1 */
    printf( "%f ", DT );zVecWrite( dis );
  }

  /* destroy */
  pdCoreDestroy( &ctrl );
  pdCmdDestroy( &cmd );
  return 0;
}
