#include <pedi2/pd_foot_uw.h>

void pdFootUWInit(pdFootUW *fuw, pdCZHrzUW *czuw)
{
  pdFootUWCZPtr( fuw ) = czuw;
  pdFootUWPhi( fuw ) = 0;
  pdFootUWRegZMPU( fuw ) = 0;
  pdFootUWRegZMPW( fuw ) = 0;
  pdFootUWRefPosU( fuw ) = 0;
  pdFootUWRefPosW( fuw ) = 0;
  pdFootUWSign( fuw ) = 0;
}

void pdFootUWDestroy(pdFootUW *fuw)
{
  pdFootUWCZPtr( fuw ) = NULL;
  pdFootUWSign( fuw ) = 0;
  pdFootUWPhi( fuw ) = 0;
  pdFootUWRegZMPU( fuw ) = 0;
  pdFootUWRegZMPW( fuw ) = 0;
  pdFootUWRefPosU( fuw ) = 0;
  pdFootUWRefPosW( fuw ) = 0;
}

static double _pdFootUWCalcHypotenuseA(double d, double lambda);
static double _pdFootUWCalcHypotenuseR(double d, double d0, double lambda);
static void _pdFootUWCalcPsiSC(double d, double d0, double lambda, double *s, double *c);
static double _pdFootUWCalcSinPhiOverLambda(double d, double nextd, double d0, double lambda);
static double _pdFootUWCalcCosPhi(double d, double nextd, double d0, double lambda);

double _pdFootUWCalcHypotenuseA(double d, double lambda)
{
  return 1.0 + lambda * lambda * d * d;
}

double _pdFootUWCalcHypotenuseR(double d, double d0, double lambda)
{
  double A, A0, r;

  A = _pdFootUWCalcHypotenuseA(d, lambda);
  A0 = _pdFootUWCalcHypotenuseA(d0, lambda);
  r = A0 - A + 1.0;
  if( r < 0.0 ){
    if( !zIsTiny( r*r ) )
      ZRUNWARN( "referential distance cannot exceed square root of (d0^2 + 1/lambda^2)" );
    return 0.0;
  }
  return sqrt( r );
}

void _pdFootUWCalcPsiSC(double d, double d0, double lambda, double *s, double *c)
{
  double R;

  R = _pdFootUWCalcHypotenuseR(d, d0, lambda);
  *s = lambda * ( d - d0 * R );
  *c = ( lambda * lambda * d * d0 + R );
}

double _pdFootUWCalcSinPhiOverLambda(double d, double nextd, double d0, double lambda)
{
  double R, nextR;

  R = _pdFootUWCalcHypotenuseR(d, d0, lambda);
  nextR = _pdFootUWCalcHypotenuseR(nextd, d0, lambda);
  return ( nextd * R - d * nextR );
}

double _pdFootUWCalcCosPhi(double d, double nextd, double d0, double lambda)
{
  double R, nextR;

  R = _pdFootUWCalcHypotenuseR(d, d0, lambda);
  nextR = _pdFootUWCalcHypotenuseR(nextd, d0, lambda);
  return ( R * nextR + lambda * lambda * d * nextd );
}


double pdFootUWCalcPhi(pdFootUW *fuw, zVec2D *delta, zVec2D *regzmp)
{
  return atan2( pdFootUWKappa(fuw)*regzmp->e[pdU],
                1.0+pdFootUWKappa(fuw)*(delta->e[pdW]-regzmp->e[pdW]));
}

double pdFootUWCalcPhiLambda(pdFootUW *fuw, double dist, double refdist)
{
  double d0, d, nextd;

  d0 = 0.5 * pdFootUWCanonDist(fuw);
  d = 0.5 * dist;
  nextd = 0.5 * refdist;
  return atan2( pdFootUWLambda(fuw) * _pdFootUWCalcSinPhiOverLambda( d, nextd, d0, pdFootUWLambda(fuw) ),
                _pdFootUWCalcCosPhi( d, nextd, d0, pdFootUWLambda(fuw) ) );
}

void pdFootUWCalcRefPos_old(pdFootUW *fuw, zVec2D *delta, zVec2D *regzmp, zVec2D *refpos)
{
  double phi;
  double ud, wd;

  phi = pdFootUWCalcPhi( fuw, delta, regzmp );
  if( zIsTiny( pdFootUWKappa(fuw) ) ){
    ud = regzmp->e[pdU];
    wd = delta->e[pdW];
  } else {
    ud = sin( phi ) / pdFootUWKappa( fuw );
    wd = ( 1.0 - cos( phi ) ) / pdFootUWKappa( fuw ) + delta->e[pdW];
  }
  ud -= 0.5 * pdFootUWSign( fuw ) * pdFootUWDist( fuw ) * sin( phi );
  wd += 0.5 * pdFootUWSign( fuw ) * pdFootUWDist( fuw ) * cos( phi );
  zVec2DCreate( refpos, ud, wd );
}

void pdFootUWCalcRefPos(pdFootUW *kf, zVec2D *delta, zVec2D *regzmp, zVec2D *ref_kf_pos)
{
  double phi;
  double dr;
  zVec2D e;

  phi = pdFootUWCalcPhi( kf, delta, regzmp );
  dr = regzmp->e[pdU]*tan(0.5*phi) + delta->e[pdW] - regzmp->e[pdW] + 0.5 * pdFootUWSign( kf ) * pdFootUWDist( kf );
  zVec2DCreate( &e, -sin( phi ), cos( phi ) );
  if( pdFootUWSign( kf ) * dr < 0 )
    dr = 0;
  zVec2DCat( regzmp, dr, &e, ref_kf_pos );
}
}

void pdFootUWCalcCOMRefPos(zVec2D *lf_pos, zVec2D *rf_pos, zVec2D *ref_pos)
{
  ref_pos->e[pdU] = 0.5 * ( lf_pos->e[pdU] + rf_pos->e[pdU] );
  ref_pos->e[pdW] = 0.5 * ( lf_pos->e[pdW] + rf_pos->e[pdW] );
}

void pdFootUWUpdate(pdFootUW *kf, zVec2D *delta, zVec2D *vel)
{
  pdFootUWCalcRegZMP( kf, delta, vel, pdFootUWRegZMP( kf ) );
  pdFootUWPhi( kf ) = pdFootUWCalcPhi( kf, delta, pdFootUWRegZMP( kf ) );
  pdFootUWCalcRefPos( kf, delta, pdFootUWRegZMP( kf ), pdFootUWRefPos( kf ) );
}

void pdFootUWFWrite(FILE *fp, pdFootUW *f)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "sign:%g, kappa:%g, lambda:%g\n",
           pdFootUWSign(f), pdFootUWKappa(f), pdFootUWLambda(f) );
  fprintf( fp, "dist:%g, phi:%g\n",
           pdFootUWDist(f), pdFootUWPhi(f) );
  fprintf( fp, "reguz:%g, regwz:%g\n", pdFootUWRegZMPU(f), pdFootUWRegZMPW(f) );
  fprintf( fp, "refud:%g, refwd:%g\n", pdFootUWRefPosU(f), pdFootUWRefPosW(f) );
}
