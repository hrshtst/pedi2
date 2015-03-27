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
  /* and inverse kinematics configuration file (robot_ik.conf) */
  pdCoreLoad( &ctrl, "model/hydra.zkc", "model/hydra_ik.conf" );

  /* prepare joint displacement vector */
  dis = zVecAlloc( pdCoreGetJointSize(&ctrl) );

  /* set referential values */
  com.zd = 0.8;             /* COM height */
  com.dist = 0.35;           /* distance of both feet */
  com.lfh = com.rfh = 0.2;  /* (maximal) foot lifting height */
  com.vud = 0.0;    /* referential velocity (longitudinal) */
  com.kappa = 0.0;  /* referential curvature for rotation */

  /* main loop */
  for( i=0; i<STEP; i++ ){
    /* modify command values */
    com.vud = 0.25;
    com.kappa = 0.0;

    /* feedback the current state */
    /* if not given, the controller will automatically update it */
    /* pdCoreSetState( &ctrl, x, y ); */

    /* update */
    pdCoreUpdate( &ctrl, DT );

    /* output */
    /* obtain desired joint displacement as a zVec instance */
    pdCoreGetJointDis( &ctrl, dis );
    printf( "%f ", DT );zVecWrite( dis ); /* output to stdout */
  }

  /* destroy */
  pdCoreExit( &ctrl );
  pdCommandExit( &com );
  return 0;
}
