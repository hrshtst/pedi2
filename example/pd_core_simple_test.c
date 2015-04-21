#include <pedi2/pd_core.h>

#define DT   0.01
#define STEP 1000
int main(void)
{
  pdCommand com; /* user-defined command values */
  pdCore ctrl;   /* core controller instance */
  zVec dis;      /* joint displacement vector */
  register int i;

  /* initialization */
  pdCommandDefaultInit( &com );
  pdCoreInit( &ctrl, &com );

  /* load kinematics/dynamics model file (robot.zkc) */
  pdCoreLoad( &ctrl, "model/hydra.zkc" );

  /* prepare joint displacement vector */
  dis = zVecAlloc( pdCoreGetJointSize(&ctrl) );

  /* set referential values */
  com.zd = 0.85;             /* COM height */
  com.dist = 0.25;           /* distance of both feet */
  com.lfh = com.rfh = 0.2;  /* (maximal) foot lifting height */
  com.vud = 0.0;    /* referential velocity (longitudinal) */
  com.kappa = 0.0;  /* referential curvature for rotation */

  /* main loop */
  for( i=0; i<STEP; i++ ){
    /* modify command values */
    com.vud = 0.1;
    com.kappa = 0.0;

    /* feedback the current state */
    /* if not given, the controller will automatically update it */
    /* pdCoreSetState( &ctrl, x, y ); */

    /* update */
    pdCoreUpdate( &ctrl, DT );

    /* output */
    /* obtain desired joint displacement as a zVec instance */
    pdCoreGetJointDis( &ctrl, dis );
    /* you can visualize the motion by executing the following command, e.g. */
    /*   $ rk_anim model/hydra.zkc motion.zvs -x 13 -y 0.5 -z 0.8            */
    printf( "%f ", DT );zVecWrite( dis );
  }

  /* destroy */
  pdCoreExit( &ctrl );
  pdCommandExit( &com );
  return 0;
}
