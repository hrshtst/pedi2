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
