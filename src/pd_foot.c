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
bool pdFootIsOn(pdFoot *f)
{
  return zVec3DElem(&f->p,zZ) < PD_FOOT_TOL;
}

bool pdFootDoesAttemptToLift(pdFoot *f)
{
  return zVec3DElem(&f->pd,zZ) > PD_FOOT_TOL;
}
