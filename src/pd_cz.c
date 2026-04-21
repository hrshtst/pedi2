#include <pedi2/pd_cz.h>

static void _pdCZUpdateState(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, zVec3D *ef, double theta, zLoop3D *sr);
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
  pdCZSetExtF( c, 0, 0, 0 );
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
  zODE2Create( &c->_ode.solver, 3, 0, _pdCZUpdate );
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
  pdCZSetExtF( c, 0, 0, 0 );
  pdCZSetTheta( c, 0 );
  pdCZSetSR( c, NULL );
  pdCZSetErrCompKX( c, 0 );
  pdCZSetErrCompKY( c, 0 );
  pdCZSetErrCompKZ( c, 0 );
  pdCZSetErrCompBX( c, 0 );
  pdCZSetErrCompBY( c, 0 );
  pdCZSetErrCompBZ( c, 0 );
}

static double _pdCZCalcHypotenuseA(double dist, double lambda);
static double _pdCZCalcHypotenuseR(double dist, double dist0, double lambda);
static void _pdCZCalcPsiSC(double dist, double dist0, double lambda, double *s, double *c);
static double _pdCZCalcSinPhiOverLambda(double dist, double refdist, double dist0, double lambda);
static double _pdCZCalcCosPhi(double dist, double refdist, double dist0, double lambda);

double _pdCZCalcHypotenuseA(double dist, double lambda)
{
  return 1.0 + lambda * lambda * dist * dist;
}

double _pdCZCalcHypotenuseR(double dist, double dist0, double lambda)
{
  double A, A0;

  A = _pdCZCalcHypotenuseA(dist, lambda);
  A0 = _pdCZCalcHypotenuseA(dist0, lambda);
  if( A - A0 > 1.0 ){
    ZRUNWARN( "referential distance cannot exceed square root of (d0^2 + 1/lambda^2)" );
    return 0.0;
  }
  return sqrt( A0 - A + 1.0 );
}

void _pdCZCalcPsiSC(double dist, double dist0, double lambda, double *s, double *c)
{
  double R;

  R = _pdCZCalcHypotenuseR(dist, dist0, lambda);
  *s = lambda * ( dist - dist0 * R );
  *c = ( lambda * lambda * dist * dist0 + R );
}

double _pdCZCalcSinPhiOverLambda(double dist, double refdist, double dist0, double lambda)
{
  double R, nextR;

  R = _pdCZCalcHypotenuseR(dist, dist0, lambda);
  nextR = _pdCZCalcHypotenuseR(refdist, dist0, lambda);
  return ( refdist * R - dist * nextR );
}

double _pdCZCalcCosPhi(double dist, double refdist, double dist0, double lambda)
{
  double R, nextR;

  R = _pdCZCalcHypotenuseR(dist, dist0, lambda);
  nextR = _pdCZCalcHypotenuseR(refdist, dist0, lambda);
  return ( R * nextR + lambda * lambda * dist * refdist );
}

double pdCZCalcDeltaTheta(pdCZ *cz, zVec2D *refuw)
{
  return atan2( pdCZKappa(cz)*refuw->e[pdU],
                1.0+pdCZKappa(cz)*(pdCZDeltaW(cz)-refuw->e[pdW]) );
}

double pdCZCalcDeltaThetaLambda(pdCZ *cz, double dist, double refdist)
{
  return atan2( pdCZLambda(cz) * _pdCZCalcSinPhiOverLambda(dist, refdist, pdCZCanonDist(cz), pdCZLambda(cz)),
                _pdCZCalcCosPhi(dist, refdist, pdCZCanonDist(cz), pdCZLambda(cz)) );
}

double pdCZCalcDeltaW(pdCZ *cz, zVec2D *refuw, double delta_theta)
{
  double delta_w;

  delta_w = ( pdCZDeltaW( cz ) - refuw->e[pdW] ) / cos( delta_theta );
  if( !zIsTiny( pdCZKappa( cz ) ) )
    delta_w += ( 1.0 - cos(delta_theta) ) / ( pdCZKappa(cz) * cos(delta_theta) );
  return delta_w;
}

double _pdCZCalcDeltaW(pdCZ *cz, zVec2D *refuw, double delta_theta)
{
  double delta_w;

  delta_w = refuw->e[pdU] * tan( 0.5 * delta_theta ) + pdCZDeltaW( cz ) - refuw->e[pdW];
  return delta_w;
}

void pdCZCalcNextUW(pdCZ *cz, zVec2D *refuw, zVec2D *nextuwd)
{
  double refdw;
  double delta_theta;

  delta_theta = pdCZCalcDeltaTheta( cz, refuw );
  refdw = pdCZCalcDeltaW( cz, refuw, delta_theta );
  zVec2DCreate( nextuwd, refuw->e[pdU] - refdw * sin(delta_theta),
                         refuw->e[pdW] + refdw * cos(delta_theta) );
}

void _pdCZUpdateState(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, zVec3D *ef, double theta, zLoop3D *sr)
{
  pdCZSetCOMVec( cz, com );
  pdCZSetVelVec( cz, vel );
  pdCZSetAccVec( cz, acc );
  pdCZSetZMPVec( cz, zmp );
  pdCZSetFZ( cz, fz );
  pdCZSetExtFVec( cz, ef );
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

  zVec2DCreate( &xy,  pdCZCOMX(cz), pdCZCOMY(cz) );
  /* zVec2DCreate( &xy,  pdCZRefCOMX(cz), pdCZRefCOMY(cz) ); */
  zVec2DCreate( &vxy, pdCZVelX(cz), pdCZVelY(cz) );
  zVec2DCreate( &xyd, pdCZCmdCOMX(cz), pdCZCmdCOMY(cz) );
  pdCZHrzUpdateAcc( pdCZHrzPtr(cz), &xy, pdCZTheta(cz), &vxy, &xyd, pdCZCmdTheta(cz), pdCZSR(cz) );
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
  zVecSetElem( acc, 0, pdCZHrzAccX( pdCZHrzPtr(cz) ) + pdCZAlphaX(cz) + pdCZExtFX(cz) );
  zVecSetElem( acc, 1, pdCZHrzAccY( pdCZHrzPtr(cz) ) + pdCZAlphaY(cz) + pdCZExtFY(cz) );
  zVecSetElem( acc, 2,  pdCZVrtAcc( pdCZVrtPtr(cz) ) + pdCZAlphaZ(cz) + pdCZExtFZ(cz) );
  return acc;
}

void _pdCZODE2Update(pdCZ *cz, zVec p, zVec v, double dt)
{
  double acc[3], vel[3];

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#elif defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wnonnull"
#endif
  zRawVecCopy( zVecBuf(v), vel, 3 );
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(__clang__)
#pragma clang diagnostic pop
#endif
  acc[0] = pdCZHrzAccX( pdCZHrzPtr(cz) ) + pdCZAlphaX(cz) + pdCZExtFX(cz);
  acc[1] = pdCZHrzAccY( pdCZHrzPtr(cz) ) + pdCZAlphaY(cz) + pdCZExtFY(cz);
  acc[2] =  pdCZVrtAcc( pdCZVrtPtr(cz) ) + pdCZAlphaZ(cz) + pdCZExtFZ(cz);
  cz->_ode._dt = dt;
  zRawVecCat( zVecBuf(p), dt, vel, zVecBuf(p), 3 );
  zRawVecCat( zVecBuf(v), dt, acc, zVecBuf(v), 3 );
}

void _pdCZUpdateRef(pdCZ *cz)
{
  _zVec3DCreate( &cz->refcom, zVecElemNC(cz->_ode.pos,zX), zVecElemNC(cz->_ode.pos,zY), zVecElemNC(cz->_ode.pos,zZ) );
  _zVec3DCreate( &cz->refvel, zVecElemNC(cz->_ode.vel,zX), zVecElemNC(cz->_ode.vel,zY), zVecElemNC(cz->_ode.vel,zZ) );
  _zVec3DCreate( &cz->refacc, pdCZHrzAccX( pdCZHrzPtr(cz) ), pdCZHrzAccY( pdCZHrzPtr(cz) ), pdCZVrtAcc( pdCZVrtPtr(cz) ) );
  _zVec3DCreate( &cz->refzmp, pdCZHrzZMPX( pdCZHrzPtr(cz) ), pdCZHrzZMPY( pdCZHrzPtr(cz) ), pdCZVrtZMP( pdCZVrtPtr(cz) ) );
  cz->reffz = pdCZVrtRF( pdCZVrtPtr(cz) );
}

void pdCZUpdate(pdCZ *cz, zVec3D *com, zVec3D *vel, zVec3D *acc, zVec3D *zmp, double fz, zVec3D *ef, double theta, zLoop3D *sr)
{
  _pdCZUpdateState( cz, com, vel, acc, zmp, fz, ef, theta, sr );
  _pdCZUpdateAlpha( cz );
  _pdCZUpdateBeta( cz, com );
  _pdCZUpdateVrt( cz );
  _pdCZUpdateHrz( cz );
  /* zODE2Update( &cz->_ode.solver, pdCZTime(cz), cz->_ode.pos, cz->_ode.vel, pdCZTimeStep(cz), cz ); */
  _pdCZODE2Update( cz, cz->_ode.pos, cz->_ode.vel, pdCZTimeStep(cz) );
  _pdCZUpdateRef( cz );
  pdCZIncrTime( cz );
}

void pdCZAutoUpdateRef_old(pdCZ *cz, zVec3D *comd, double *thetad)
{
  zVec2D refxy, refuw;
  zVec2D nextxyd, nextuwd;
  double delta_theta;

  zVec2DCreate( &refxy, pdCZRefCOMX(cz), pdCZRefCOMY(cz) );
  pdCZHrzXformXYtoUW( pdCZHrzPtr(cz), &refxy, &refuw );
  delta_theta = pdCZCalcDeltaTheta( cz, &refuw );
  pdCZCalcNextUW( cz, &refuw, &nextuwd );
  pdCZHrzXformUWtoXY( pdCZHrzPtr(cz), &nextuwd, &nextxyd );
  zVec3DCreate( comd, nextxyd.c.x, nextxyd.c.y, comd->c.z );
  *thetad = pdCZTheta(cz) + delta_theta;
}

void pdCZAutoUpdateRef(pdCZ *cz, zVec3D *lfpos, zVec3D *rfpos, zVec3D *comd, double *thetad)
{
  zVec2D refxy, refuw;
  zVec2D lfxy, lfuw;
  zVec2D rfxy, rfuw;
  zVec2D nextxyd, nextuwd;
  double delta_thetalf, delta_thetarf, delta_theta;
  double delta_wlf, delta_wrf, delta_w, next_delta_w;

  /* world frame -> moving frame */
  zVec2DCreate( &refxy, pdCZRefCOMX(cz), pdCZRefCOMY(cz) );
  pdCZHrzXformXYtoUW( pdCZHrzPtr(cz), &refxy, &refuw );
  zVec2DCreate( &lfxy, lfpos->c.x, lfpos->c.y );
  pdCZHrzXformXYtoUW( pdCZHrzPtr(cz), &lfxy, &lfuw );
  zVec2DCreate( &rfxy, rfpos->c.x, rfpos->c.y );
  pdCZHrzXformXYtoUW( pdCZHrzPtr(cz), &rfxy, &rfuw );
  /* calculate desired COM position */
  delta_theta = pdCZCalcDeltaTheta( cz, &refuw );
  delta_thetalf = pdCZCalcDeltaTheta( cz, &lfuw );
  delta_thetarf = pdCZCalcDeltaTheta( cz, &rfuw );
  delta_w = pdCZCalcDeltaW( cz, &refuw, delta_theta );
  delta_wlf = pdCZCalcDeltaW( cz, &lfuw, delta_thetalf );
  delta_wrf = pdCZCalcDeltaW( cz, &rfuw, delta_thetarf );
  next_delta_w = delta_w - 0.5 * ( delta_wlf + delta_wrf );
  zVec2DCreate( &nextuwd, refuw.e[pdU] - next_delta_w * sin(delta_theta),
                          refuw.e[pdW] + next_delta_w * cos(delta_theta) );
  /* moving frame -> world frame */
  pdCZHrzXformUWtoXY( pdCZHrzPtr(cz), &nextuwd, &nextxyd );
  zVec3DCreate( comd, nextxyd.c.x, nextxyd.c.y, comd->c.z );
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
