#include <pedi2/pd_cz.h>

static void _pdCZUpdateState(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, double theta, zVec3DList *sr);
static void _pdCZUpdateVrt(pdCZ *cz);
static void _pdCZUpdateHrz(pdCZ *cz);
static void _pdCZUpdateAlpha(pdCZ *cz);
static zVec _pdCZUpdate(double t, zVec pos, zVec vel, void *util, zVec acc);
static void _pdCZODE2Update(pdCZ *cz, zVec p, zVec v, double dt);
static void _pdCZUpdateRef(pdCZ *cz);

void pdCZInit(pdCZ *c, double dt)
{
  pdCZSetCmdCOM( c, 0, 0, 0 );
  pdCZSetCmdTheta( c, 0 );
  pdCZSetCOM( c, 0, 0, 0 );
  pdCZSetVel( c, 0, 0, 0 );
  pdCZSetAcc( c, 0, 0, 0 );
  pdCZSetZMP( c, 0, 0, 0 );
  pdCZSetFZ( c, 0 );
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
  zODE2Init( &c->_ode.solver, 3, 0, _pdCZUpdate );
  pdCZSetErrCompKX( c, 0 );
  pdCZSetErrCompKY( c, 0 );
  pdCZSetErrCompKZ( c, 0 );
  pdCZSetErrCompBX( c, 0 );
  pdCZSetErrCompBY( c, 0 );
  pdCZSetErrCompBZ( c, 0 );
  pdCZAlphaX( c ) = 0;
  pdCZAlphaY( c ) = 0;
  pdCZAlphaZ( c ) = 0;
  pdCZBetaX( c )  = 0;
  pdCZBetaY( c )  = 0;
  pdCZBetaZ( c )  = 0;
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
  pdCZRefFZ( c ) = 0;
}

void pdCZDestroy(pdCZ *c)
{
  zVecFree( c->_ode.pos );
  zVecFree( c->_ode.vel );
  zODE2Destroy( &c->_ode.solver );
  pdCZVrtDestroy( pdCZVrtPtr(c) );
  pdCZHrzDestroy( pdCZHrzPtr(c) );
  pdCZSetCmdCOM( c, 0, 0, 0 );
  pdCZSetCmdTheta( c, 0 );
  pdCZSetCOM( c, 0, 0, 0 );
  pdCZSetVel( c, 0, 0, 0 );
  pdCZSetAcc( c, 0, 0, 0 );
  pdCZSetZMP( c, 0, 0, 0 );
  pdCZSetFZ( c, 0 );
  pdCZSetTheta( c, 0 );
  pdCZSetSR( c, NULL );
  pdCZSetErrCompKX( c, 0 );
  pdCZSetErrCompKY( c, 0 );
  pdCZSetErrCompKZ( c, 0 );
  pdCZSetErrCompBX( c, 0 );
  pdCZSetErrCompBY( c, 0 );
  pdCZSetErrCompBZ( c, 0 );
}

double pdCZCalcDeltaTheta(pdCZ *cz, zVec2D refuw)
{
  return atan2( pdCZKappa(cz)*refuw[pdU],
                1.0+pdCZKappa(cz)*(pdCZDeltaW(cz)-refuw[pdW]) );
}

double pdCZCalcDeltaW(pdCZ *cz, zVec2D refuw, double delta_theta)
{
  double delta_w;

  delta_w = ( pdCZDeltaW( cz ) - refuw[pdW] ) / cos( delta_theta );
  if( !zIsTiny( pdCZKappa( cz ) ) )
    delta_w += ( 1.0 - cos(delta_theta) ) / ( pdCZKappa(cz) * cos(delta_theta) );
  return delta_w;
}

void pdCZCalcNextUW(pdCZ *cz, zVec2D refuw, zVec2D nextuwd)
{
  double refdw;
  double delta_theta;

  delta_theta = pdCZCalcDeltaTheta( cz, refuw );
  refdw = pdCZCalcDeltaW( cz, refuw, delta_theta );
  zVec2DCreate( nextuwd, refuw[pdU] - refdw * sin(delta_theta),
                         refuw[pdW] + refdw * cos(delta_theta) );
}

void _pdCZUpdateState(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, double theta, zVec3DList *sr)
{
  pdCZSetCOMVec( cz, com );
  pdCZSetVelVec( cz, vel );
  pdCZSetAccVec( cz, acc );
  pdCZSetZMPVec( cz, zmp );
  pdCZSetFZ( cz, fz );
  pdCZSetTheta( cz, theta );
  pdCZSetSR( cz, sr );
  zVecSetElem( cz->_ode.pos, zX, pdCZRefCOMX(cz) );
  zVecSetElem( cz->_ode.pos, zY, pdCZRefCOMY(cz) );
  zVecSetElem( cz->_ode.pos, zZ, pdCZRefCOMZ(cz) );
  zVecSetElem( cz->_ode.vel, zX, pdCZRefVelX(cz) );
  zVecSetElem( cz->_ode.vel, zY, pdCZRefVelY(cz) );
  zVecSetElem( cz->_ode.vel, zZ, pdCZRefVelZ(cz) );
}

void _pdCZUpdateVrt(pdCZ *cz)
{
  pdCZVrtSetRef( pdCZVrtPtr( cz ), pdCZCmdCOMZ( cz ) );
  pdCZVrtUpdateAcc( pdCZVrtPtr(cz), pdCZCOMZ(cz), pdCZVelZ(cz) );
  pdCZVrtUpdateZeta( pdCZVrtPtr(cz), pdCZCOMZ(cz), pdCZVrtAcc(pdCZVrtPtr(cz)), pdCZZMPZ(cz) );
  /* pdCZVrtUpdateAcc( pdCZVrtPtr(cz), pdCZRefCOMZ(cz), pdCZVelZ(cz) ); */
  /* pdCZVrtUpdateZeta( pdCZVrtPtr(cz), pdCZRefCOMZ(cz), pdCZVrtAcc(pdCZVrtPtr(cz)), pdCZZMPZ(cz) ); */
  pdCZVrtUpdateZMP( pdCZVrtPtr(cz) ); /* may not be updated here */
}

void _pdCZUpdateHrz(pdCZ *cz)
{
  zVec2D xy, vxy, xyd;

  zVec2DCreate( xy,  pdCZCOMX(cz), pdCZCOMY(cz) );
  /* zVec2DCreate( xy,  pdCZRefCOMX(cz), pdCZRefCOMY(cz) ); */
  zVec2DCreate( vxy, pdCZVelX(cz), pdCZVelY(cz) );
  zVec2DCreate( xyd, pdCZCmdCOMX(cz), pdCZCmdCOMY(cz) );
  pdCZHrzUpdateAcc( pdCZHrzPtr(cz), xy, pdCZTheta(cz), vxy, xyd, pdCZCmdTheta(cz), pdCZSR(cz) );
}

void _pdCZUpdateAlpha(pdCZ *cz)
{
  pdCZHrz *h = pdCZHrzPtr(cz);
  pdCZVrt *v = pdCZVrtPtr(cz);

  pdCZAlphaX(cz) = pdCZErrCompKX(cz)*zSqr(pdCZZeta(cz))*(pdCZHrzZMPX(h)-pdCZZMPX(cz));
  pdCZAlphaY(cz) = pdCZErrCompKY(cz)*zSqr(pdCZZeta(cz))*(pdCZHrzZMPY(h)-pdCZZMPY(cz));
  pdCZAlphaZ(cz) = pdCZErrCompKZ(cz)*(pdCZFZ(cz)-pdCZVrtRF(v));
}

void _pdCZUpdateBeta(pdCZ *cz, zVec3D *com)
{
  pdCZBetaX(cz) -= pdCZErrCompBX(cz)*(pdCZCOMX(cz)-pdCZRefCOMX(cz));
  pdCZBetaY(cz) -= pdCZErrCompBY(cz)*(pdCZCOMY(cz)-pdCZRefCOMY(cz));
  pdCZBetaZ(cz) -= pdCZErrCompBZ(cz)*(pdCZCOMZ(cz)-pdCZRefCOMZ(cz));
}

zVec _pdCZUpdate(double t, zVec pos, zVec vel, void *util, zVec acc)
{
  pdCZ *cz;

  cz = (pdCZ *)util;
  /* _pdCZUpdateVrt( cz ); */
  /* _pdCZUpdateHrz( cz ); */
  zVecSetElem( acc, 0, pdCZHrzAccX( pdCZHrzPtr(cz) ) + pdCZAlphaX(cz) );
  zVecSetElem( acc, 1, pdCZHrzAccY( pdCZHrzPtr(cz) ) + pdCZAlphaY(cz) );
  zVecSetElem( acc, 2, pdCZVrtAcc( pdCZVrtPtr(cz) ) + pdCZAlphaZ(cz) );
  return acc;
}

void _pdCZODE2Update(pdCZ *cz, zVec p, zVec v, double dt)
{
  double acc[3], vel[3];

  zRawVecCopy( zVecBuf(v), vel, 3 );
  acc[0] = pdCZHrzAccX( pdCZHrzPtr(cz) ) + pdCZAlphaX(cz);
  acc[1] = pdCZHrzAccY( pdCZHrzPtr(cz) ) + pdCZAlphaY(cz);
  acc[2] =  pdCZVrtAcc( pdCZVrtPtr(cz) ) + pdCZAlphaZ(cz);
  cz->_ode._dt = dt;
  zRawVecCat( zVecBuf(p), dt, vel, zVecBuf(p), 3 );
  zRawVecCat( zVecBuf(v), dt, acc, zVecBuf(v), 3 );
}

void _pdCZUpdateRef(pdCZ *cz)
{
  zVec3DSetElem( &cz->refcom, zX, zVecElem( cz->_ode.pos, zX ) );
  zVec3DSetElem( &cz->refcom, zY, zVecElem( cz->_ode.pos, zY ) );
  zVec3DSetElem( &cz->refcom, zZ, zVecElem( cz->_ode.pos, zZ ) );
  zVec3DSetElem( &cz->refvel, zX, zVecElem( cz->_ode.vel, zX ) );
  zVec3DSetElem( &cz->refvel, zY, zVecElem( cz->_ode.vel, zY ) );
  zVec3DSetElem( &cz->refvel, zZ, zVecElem( cz->_ode.vel, zZ ) );
  zVec3DSetElem( &cz->refacc, zX, pdCZHrzAccX( pdCZHrzPtr(cz) ) );
  zVec3DSetElem( &cz->refacc, zY, pdCZHrzAccY( pdCZHrzPtr(cz) ) );
  zVec3DSetElem( &cz->refacc, zZ, pdCZVrtAcc( pdCZVrtPtr(cz) ) );
  zVec3DSetElem( &cz->refzmp, zX, pdCZHrzZMPX( pdCZHrzPtr(cz) ) );
  zVec3DSetElem( &cz->refzmp, zY, pdCZHrzZMPY( pdCZHrzPtr(cz) ) );
  zVec3DSetElem( &cz->refzmp, zZ, pdCZVrtZMP( pdCZVrtPtr(cz) ) );
  cz->reffz = pdCZVrtRF( pdCZVrtPtr(cz) );
}


void pdCZUpdate(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, double theta, zVec3DList *sr)
{
  _pdCZUpdateState( cz, com, vel, acc, zmp, fz, theta, sr );
  _pdCZUpdateAlpha( cz );
  _pdCZUpdateBeta( cz, com );
  _pdCZUpdateVrt( cz );
  _pdCZUpdateHrz( cz );
  /* zODE2Update( &cz->_ode.solver, pdCZTime(cz), cz->_ode.pos, cz->_ode.vel, pdCZTimeStep(cz), cz ); */
  _pdCZODE2Update( cz, cz->_ode.pos, cz->_ode.vel, pdCZTimeStep(cz) );
  _pdCZUpdateRef( cz );
  pdCZIncrTime( cz );
}

void pdCZAutoUpdateRef(pdCZ *cz, zVec3D *comd, double *thetad)
{
  zVec2D refxy, refuw;
  zVec2D nextxyd, nextuwd;
  double delta_theta;

  zVec2DCreate( refxy, pdCZRefCOMX(cz), pdCZRefCOMY(cz) );
  pdCZHrzXformXYtoUW( pdCZHrzPtr(cz), refxy, refuw );
  delta_theta = pdCZCalcDeltaTheta( cz, refuw );
  pdCZCalcNextUW( cz, refuw, nextuwd );
  pdCZHrzXformUWtoXY( pdCZHrzPtr(cz), nextuwd, nextxyd );
  zVec3DCreate( comd, nextxyd[zX], nextxyd[zY], zVec3DElem( comd, zZ ) );
  *thetad = pdCZTheta(cz) + delta_theta;
}

void pdCZFWrite(FILE *fp, pdCZ *cz)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "t:%g, dt:%g\n", pdCZTime(cz), pdCZTimeStep(cz) );
  fprintf( fp, "xd:%g, yd:%g, zd:%g, thetad:%g\n",
           pdCZCmdCOMX(cz), pdCZCmdCOMY(cz), pdCZCmdCOMZ(cz), pdCZCmdTheta(cz) );
  fprintf( fp, "x :%g, y: %g, z: %g, theta :%g\n",
           pdCZCOMX(cz), pdCZCOMY(cz), pdCZCOMZ(cz), pdCZTheta(cz) );
  fprintf( fp, "vx:%g, vy:%g, vz:%g\n", pdCZVelX(cz), pdCZVelY(cz), pdCZVelZ(cz) );
  fprintf( fp, "ax:%g, ay:%g, az:%g\n", pdCZAccX(cz), pdCZAccY(cz), pdCZAccZ(cz) );
  fprintf( fp, "zx:%g, zy:%g, zz:%g\n", pdCZZMPX(cz), pdCZZMPY(cz), pdCZZMPZ(cz) );
  fprintf( fp, "ud:%g, deltau:%g, vu:%g\n",
           pdCZRefPosU(cz), pdCZDeltaU(cz), pdCZVelU(cz) );
  fprintf( fp, "wd:%g, deltaw:%g, vw:%g\n",
           pdCZRefPosW(cz), pdCZDeltaW(cz), pdCZVelW(cz) );
  fprintf( fp, "refx: %g, refy: %g, refz: %g\n",
           pdCZRefCOMX(cz), pdCZRefCOMY(cz), pdCZRefCOMZ(cz) );
  fprintf( fp, "refvx:%g, refvy:%g, refvz:%g\n",
           pdCZRefVelX(cz), pdCZRefVelY(cz), pdCZRefVelZ(cz) );
  fprintf( fp, "refax:%g, refay:%g, refaz:%g\n",
           pdCZRefAccX(cz), pdCZRefAccY(cz), pdCZRefAccZ(cz) );
  fprintf( fp, "refzx:%g, refzy:%g, refzz:%g\n",
           pdCZRefZMPX(cz), pdCZRefZMPY(cz), pdCZRefZMPZ(cz) );
}

void pdCZDataFWrite(FILE *fp, pdCZ *cz)
{
  fprintf( fp, "%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n",
/* 0- 1*/  pdCZTime(cz), pdCZTimeStep(cz),
/* 2- 5*/  pdCZCmdCOMX(cz), pdCZCmdCOMY(cz), pdCZCmdCOMZ(cz), pdCZCmdTheta(cz),
/* 6- 9*/  pdCZCOMX(cz), pdCZCOMY(cz), pdCZCOMZ(cz), pdCZTheta(cz),
/*10-12*/  pdCZVelX(cz), pdCZVelY(cz), pdCZVelZ(cz),
/*13-15*/  pdCZAccX(cz), pdCZAccY(cz), pdCZAccZ(cz),
/*16-18*/  pdCZZMPX(cz), pdCZZMPY(cz), pdCZZMPZ(cz),
/*19-21*/  pdCZRefPosU(cz), pdCZDeltaU(cz), pdCZVelU(cz),
/*22-24*/  pdCZRefPosW(cz), pdCZDeltaW(cz), pdCZVelW(cz),
/*25-27*/  pdCZRefCOMX(cz), pdCZRefCOMY(cz), pdCZRefCOMZ(cz),
/*28-30*/  pdCZRefVelX(cz), pdCZRefVelY(cz), pdCZRefVelZ(cz),
/*31-33*/  pdCZRefAccX(cz), pdCZRefAccY(cz), pdCZRefAccZ(cz),
/*34-36*/  pdCZRefZMPX(cz), pdCZRefZMPY(cz), pdCZRefZMPZ(cz) );
}
