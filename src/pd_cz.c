#include <pedi2/pd_cz.h>

void pdCZInit(pdCZ *c, double dt)
{
  pdCZSetCmdCOM( c, 0, 0, 0 );
  pdCZSetCmdTheta( c, 0 );
  pdCZSetCOM( c, 0, 0, 0 );
  pdCZSetVel( c, 0, 0, 0 );
  pdCZSetAcc( c, 0, 0, 0 );
  pdCZSetZMP( c, 0, 0, 0 );
  pdCZSetTheta( c, 0 );
  pdCZSetSR( c, NULL );
  pdCZVrtInit( pdCZVrtPtr(c) );
  pdCZHrzInit( pdCZHrzPtr(c), pdCZVrtPtr(c) );
  pdCZResetTime( c );
  pdCZSetTimeStep( c, dt );
  c->_ode.pos  = zVecCreateList( 3, pdCZCOMX(c), pdCZCOMY(c), pdCZCOMZ(c) );
  c->_ode.vel = zVecCreateList( 3, pdCZVelX(c), pdCZVelY(c), pdCZVelZ(c) );
  zODE2Assign( &c->_ode.solver, Regular, NULL, NULL, NULL, NULL );
  zODE2AssignRegular( &c->_ode.solver, RK4 );
  /* zODE2Init( &c->_ode.solver, 3, 0, NULL ); */
  pdCZRefCOMX( c ) = 0;
  pdCZRefCOMY( c ) = 0;
  pdCZRefCOMZ( c ) = 0;
  pdCZRefVelX( c ) = 0;
  pdCZRefVelY( c ) = 0;
  pdCZRefVelZ( c ) = 0;
  pdCZRefAccX( c ) = 0;
  pdCZRefAccY( c ) = 0;
  pdCZRefAccZ( c ) = 0;
  pdCZRefZMPX( c ) = 0;
  pdCZRefZMPY( c ) = 0;
  pdCZRefZMPZ( c ) = 0;
}

void pdCZDestroy(pdCZ *c)
{
  zVecFree( c->_ode.pos );
  zVecFree( c->_ode.vel );
  /* zODE2Destroy( &c->_ode.solver ); */
  pdCZVrtDestroy( pdCZVrtPtr(c) );
  pdCZHrzDestroy( pdCZHrzPtr(c) );
  pdCZSetCmdCOM( c, 0, 0, 0 );
  pdCZSetCmdTheta( c, 0 );
  pdCZSetCOM( c, 0, 0, 0 );
  pdCZSetVel( c, 0, 0, 0 );
  pdCZSetAcc( c, 0, 0, 0 );
  pdCZSetZMP( c, 0, 0, 0 );
  pdCZSetTheta( c, 0 );
  pdCZSetSR( c, NULL );
}

double pdCZCalcDeltaTheta(pdCZ *cz, zVec2D refuw)
{
  return atan2( pdCZKappa(cz)*refuw[pdU],
                1.0+pdCZKappa(cz)*(pdCZDeltaW(cz)-refuw[pdW]) );
}

void pdCZCalcNextUW(pdCZ *cz, zVec2D refuw, zVec2D nextuwd)
{
  double refdw;
  double delta_theta;

  delta_theta = pdCZCalcDeltaTheta( cz, refuw );
  refdw = ( pdCZDeltaW( cz ) - refuw[pdW] ) / cos( delta_theta );
  if( !zIsTiny( pdCZKappa( cz ) ) )
    refdw += ( 1.0 - cos(delta_theta) ) / ( pdCZKappa(cz) * cos(delta_theta) );
  zVec2DCreate( nextuwd, refuw[pdU] - refdw * sin(delta_theta),
                         refuw[pdW] + refdw * cos(delta_theta) );
}

#if 0
void pdCZZMPPhase(pdCZ *c, double dw, double vw, zComplex *pz)
{
  /* zComplexCreate( pz, pdCZZMPRad(c) - dw, -(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2+1)*vw/(pdCZZeta(c)*sqrt(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2)) ); */
  zComplexCreate( pz, pdCZZMPRad(c), -(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2+1)*vw/(pdCZZeta(c)*sqrt(pdCZPrmRad(c)->q1*pdCZPrmRad(c)->q2)) );
}
#endif
