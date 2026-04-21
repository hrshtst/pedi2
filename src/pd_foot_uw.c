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

void _pdFootUWCalcNextUWLambda(pdFootUW *fuw, zVec2D *delta, double dist, double refdist, zVec2D *nextuwd)
{
  double d0, d, nextd;
  double A0;
  double sin_psi, cos_psi;
  double sin_phi_over_lambda;
  double lambda_d0;

  d0 = 0.5 * pdFootUWCanonDist(fuw);
  d = 0.5 * dist;
  nextd = 0.5 * refdist;
  A0 = _pdFootUWCalcHypotenuseA( d0, pdFootUWLambda(fuw) );
  _pdFootUWCalcPsiSC( nextd, d0, pdFootUWLambda(fuw), &sin_psi, &cos_psi );
  sin_phi_over_lambda = _pdFootUWCalcSinPhiOverLambda( d, nextd, d0, pdFootUWLambda(fuw) );
  lambda_d0 = pdFootUWLambda(fuw) * d0;
  zVec2DCreate( nextuwd, delta->e[pdU] - ( sin_psi+lambda_d0*cos_psi)*sin_phi_over_lambda/(A0*A0),
                         delta->e[pdW] - (-cos_psi+lambda_d0*sin_psi)*sin_phi_over_lambda/(A0*A0) );
}

void pdFootUWCalcRefPosLambda(pdFootUW *kf, zVec2D *delta, zVec2D *regzmp, double refdist, zVec2D *pf_pos, zVec2D *ref_kf_pos)
{
  double stabilizable_refdist;
  zVec2D pk, next_uwd, next_pk, ps;
  double pk_norm, next_pk_norm, ps_norm;
  double cos_phi, cos_phi1, cos_phi2;
  zVec2D e, stabilizable_next_pk;

  /* Calculate three vectors based on the pivot foot for comparison */
  _pdFootUWCalcNextUWLambda( kf, delta, pdFootUWDist( kf ), refdist, &next_uwd );
  zVec2DSub( delta, pf_pos, &pk );          /* current kick foot position */
  zVec2DSub( &next_uwd, pf_pos, &next_pk ); /* desired kick foot position */
  zVec2DSub( regzmp, pf_pos, &ps );         /* strong-stabilizable point */

  /* calculate stabilizable referential distance between feet */
  pk_norm = zVec2DNorm( &pk );
  next_pk_norm = zVec2DNorm( &next_pk );
  ps_norm = zVec2DNorm( &ps );
  stabilizable_refdist = zMax( 2.0 * zMax( pk_norm, next_pk_norm ), ps_norm );

  /* calculate stabilizable differential angle of kick foot */
  cos_phi = zVec2DInnerProd( &pk, &next_pk ) / ( pk_norm * next_pk_norm );
  cos_phi1 = zVec2DInnerProd( &ps, &pk ) / ( ps_norm * pk_norm );
  cos_phi2 = zVec2DInnerProd( &ps, &next_pk ) / ( ps_norm * next_pk_norm );
  if( cos_phi <= cos_phi1 && cos_phi <= cos_phi2 ){
    /* desired kick foot position can be stabilizable */
    zVec2DNormalize( &next_pk, &e );
  } else {
    /* strong-stabilizable point should be used for kick foot position */
    zVec2DNormalize( &ps, &e );
  }
  zVec2DMul( &e, stabilizable_refdist, &stabilizable_next_pk );

  /* calculate kick foot position with respect to moving frame */
  zVec2DAdd( pf_pos, &stabilizable_next_pk, ref_kf_pos );
}

void pdFootUWCalcCOMRefPos(zVec2D *lf_pos, zVec2D *rf_pos, zVec2D *ref_pos)
{
  ref_pos->e[pdU] = 0.5 * ( lf_pos->e[pdU] + rf_pos->e[pdU] );
  ref_pos->e[pdW] = 0.5 * ( lf_pos->e[pdW] + rf_pos->e[pdW] );
}

void pdFootUWUpdate(pdFootUW *kf, zVec2D *delta, zVec2D *vel, double refdist, zVec2D *pf_pos)
{
  pdFootUWCalcRegZMP( kf, delta, vel, pdFootUWRegZMP( kf ) );
  if( zIsTiny( pdFootUWLambda(kf) ) ){
    pdFootUWPhi( kf ) = pdFootUWCalcPhi( kf, delta, pdFootUWRegZMP( kf ) );
    pdFootUWCalcRefPos( kf, delta, pdFootUWRegZMP( kf ), pdFootUWRefPos( kf ) );
  } else {
    pdFootUWPhi( kf ) = pdFootUWCalcPhiLambda( kf, pdFootUWDist(kf), refdist );
    pdFootUWCalcRefPosLambda( kf, delta, pdFootUWRegZMP( kf ), refdist, pf_pos, pdFootUWRefPos( kf ) );
  }
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
