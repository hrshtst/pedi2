#include <pedi2/pd_cz.h>

void set_ctrl_parameters(pdCZ *cz)
{
  /* vertical motion controller */
  pdCZSetQ1Z( cz, 1.0 );
  pdCZSetQ2Z( cz, 0.5 );
  /* horizontal motion controller */
  pdCZSetRefVelU( cz, 0.2 );
  pdCZSetQ1U( cz, 1.0 );
  pdCZSetQ2U( cz, 0.0 );
  pdCZSetRefVelW( cz, 0.0 );
  pdCZSetQ1W( cz, 1.0 );
  pdCZSetQ2W( cz, 1.5 );
  pdCZSetRho( cz, 1.0 );
  pdCZSetKr( cz, 1.0 );
  pdCZSetDist( cz, 0.2 );
  pdCZSetKappa( cz, 0.0 );
}

#define DT 0.01
#define T 20.0
int main(void)
{
  pdCZ cz;
  zVec3D com, vel, acc, zmp, comd;
  double fz;
  double theta, thetad;
  double t;

  /* init of pdCZ */
  pdCZInit( &cz, DT );
  set_ctrl_parameters( &cz );
  /* init states */
  zVec3DCreate( &com, 0, 0, 0.3 );
  zVec3DCopy( &com, pdCZRefCOM( &cz ) );
  zVec3DZero( &vel );
  zVec3DZero( &acc );
  zVec3DZero( &zmp );
  fz = 0;
  zVec3DCreate( &comd, 0, 0, 0.26 );
  theta = thetad = -zPI_2;
  /* loop */
  for( t=0; t<T; t+=DT ){
    /* command update */
    pdCZSetCmdCOMVec( &cz, &comd );
    pdCZSetCmdTheta( &cz, thetad );
    if( t < 0.4*T )
      pdCZSetKappa( &cz, 1.0 );
    else if( t < 0.6*T )
      pdCZSetKappa( &cz, 0.0 );
    else
      pdCZSetKappa( &cz, -1.0 );
    /* update */
    pdCZUpdate( &cz, &com, &vel, &acc, &zmp, fz, ZVEC3DZERO, theta, NULL );
    /* output */
    pdCZDataWrite( &cz );
    /* auto reference update */
    if( !zIsTiny( pdCZRefVelU(&cz) ) )
      pdCZAutoUpdateRef_old( &cz, &comd, &thetad );
    /* state update */
    zVec3DCopy( pdCZRefCOM(&cz), &com );
    zVec3DCopy( pdCZRefVel(&cz), &vel );
    zVec3DCopy( pdCZRefAcc(&cz), &acc );
    zVec3DCopy( pdCZRefZMP(&cz), &zmp );
    theta = thetad;
  }

  return 0;
}
