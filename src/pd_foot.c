#include <pedi2/pd_foot.h>

void _pdFootWritePosAtt(pdFoot *f)
{
  /* for debug */
  char buf[20];

  if( f->dy > 0 ) sprintf( buf, "lf" );
  else sprintf( buf, "rf" );
  printf( "%s.p :", buf ); zVec3DWrite( &f->p );
  printf( "%s.pd:", buf ); zVec3DWrite( &f->pd );
  printf( "%s.ps:", buf ); zVec3DWrite( &f->ps );
  printf( "%s.a :", buf ); zVec3DWrite( &f->a );
  printf( "%s.as:", buf ); zVec3DWrite( &f->as );
}

void pdFootPhase(pdFoot *pf, pdFoot *kf, double xd, double yd, double theta, zComplex *pz)
{
  double dr, d, da, r2, r;
  double s, c;
  zComplex p, penter;

  zSinCos( theta, &s, &c );
  r2 = zComplexSqrAbs(pz);
  r  = sqrt(r2);
  dr = -(zVec3DElem(&pf->p,zX)-xd)*c-(zVec3DElem(&pf->p,zY)-yd)*s - 0.5*pf->dy*pf->sole_w;
  da = acos( fabs(dr) / r );
  if( ( d = r2 - zSqr(dr) ) > 0 ){
    zComplexCreate( &penter, dr, -pf->dy*sqrt(d) );
    zComplexCDiv( pz, &penter, &p );
    kf->phase = zLimit( 0.5*zComplexArg(&p)/da, 0, 1 );
  } else
    kf->phase = 0;
}

void pdFootLift(pdCZ *ctrl, pdFoot *lf, pdFoot *rf, double xd, double yd, double theta, zComplex *pz)
{
  double dh;

  pdFootPhase( rf, lf, xd, yd, theta, pz );
  pdFootPhase( lf, rf, xd, yd, theta, pz );
  dh = 2.0 * zComplexAbs(pz) / pdCZPrmRad(ctrl)->dist * zLimit( (pdCZPrmRad(ctrl)->rho*zE-1)/(zE-1), 0, 1 );
  zVec3DElem(&lf->pd,zZ) = zCycloidY( 0, lf->h * dh, lf->phase );
  zVec3DElem(&rf->pd,zZ) = zCycloidY( 0, rf->h * dh, rf->phase );
}

#define PD_FOOT_TOL (1.0e-3)
bool pdFootIsOff(pdFoot *f)
{
  return zVec3DElem(&f->p,zZ) > PD_FOOT_TOL;
}

bool pdFootDoesAttemptToLift(pdFoot *f)
{
  return zVec3DElem(&f->pd,zZ) > PD_FOOT_TOL;
}

static void _pdFootMoveKick(pdCZ *ctrl, pdFoot *pf, pdFoot *kf, double xd, double yd, double theta);

void _pdFootMoveKick(pdCZ *ctrl, pdFoot *pf, pdFoot *kf, double xd, double yd, double theta)
{
  double d;
  double s, c;

  d = 0.5 * pdCZPrmRad(ctrl)->dist;
  zSinCos( theta, &s, &c );

  zVec3DElem(&kf->pd,zX) = xd - kf->dy * d * c;
  zVec3DElem(&kf->pd,zY) = yd - kf->dy * d * s;
  zVec3DCreate( &kf->as, theta+zPI_2, 0, 0 );
}

void pdFootMove(pdCZ *ctrl, pdFoot *lf, pdFoot *rf, double xd, double yd, double theta)
{
  if( pdFootIsOff( lf ) || pdFootDoesAttemptToLift( lf ) ){
    _pdFootMoveKick( ctrl, rf, lf, xd, yd, theta );
  }
  if( pdFootIsOff( rf ) || pdFootDoesAttemptToLift( rf ) ){
    _pdFootMoveKick( ctrl, lf, rf, xd, yd, theta );
  }
}

static void _pdFootUpdateSOL(pdFoot *f, zAxis i, double dt);

void _pdFootUpdateSOL(pdFoot *f, zAxis i, double dt)
{
  zVec3DElem(&f->ps,i) =
      ( f->track_k[i]*dt*dt * zVec3DElem(&f->pd,i) + ( f->track_c[i]*dt+2 ) * zVec3DElem(&f->p,i) - f->track_old[i] ) / ( 1 + f->track_c[i]*dt + f->track_k[i]*dt*dt );
  f->track_old[i] = zVec3DElem(&f->p,i);
}

void pdFootUpdate(pdFoot *lf, pdFoot *rf, double dt)
{
  if( pdFootIsOff( lf ) || pdFootDoesAttemptToLift( lf ) ){
    _pdFootUpdateSOL( lf, zX, dt );
    _pdFootUpdateSOL( lf, zY, dt );
    _pdFootUpdateSOL( lf, zZ, dt );
  }
  if( pdFootIsOff( rf ) || pdFootDoesAttemptToLift( rf ) ){
    _pdFootUpdateSOL( rf, zX, dt );
    _pdFootUpdateSOL( rf, zY, dt );
    _pdFootUpdateSOL( rf, zZ, dt );
  }
  /* printf("--\n"); */
  /* _pdFootWritePosAtt( lf ); */
  /* _pdFootWritePosAtt( rf ); */
}
