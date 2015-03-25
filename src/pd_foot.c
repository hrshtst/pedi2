#include <pedi2/pd_foot.h>

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
