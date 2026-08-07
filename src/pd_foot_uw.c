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
  pdFootUWLandingFn( fuw ) = NULL;
  pdFootUWLandingUtil( fuw ) = NULL;
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
  pdFootUWLandingFn( fuw ) = NULL;
  pdFootUWLandingUtil( fuw ) = NULL;
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter" /* unused parameter ‘vel’ [-Wunused-parameter] */
#elif defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
double pdFootUWCalcPhi(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp)
{
  return atan2( pdFootUWKappa(fuw)*regzmp->e[pdU],
                1.0+pdFootUWKappa(fuw)*(delta->e[pdW]-regzmp->e[pdW]));
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(__clang__)
#pragma clang diagnostic pop
#endif

void pdFootUWCalcRefPos_old(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos)
{
  double phi;
  double ud, wd;

  phi = pdFootUWCalcPhi( fuw, delta, vel, regzmp );
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

void pdFootUWCalcRefPos(pdFootUW *fuw, zVec2D *delta, zVec2D *vel, zVec2D *regzmp, zVec2D *refpos)
{
  double phi;
  double dr;
  zVec2D e;

  phi = pdFootUWCalcPhi( fuw, delta, vel, regzmp );
  dr = regzmp->e[pdU]*tan(0.5*phi) + delta->e[pdW] - regzmp->e[pdW] + 0.5 * pdFootUWSign( fuw ) * pdFootUWDist( fuw );
  zVec2DCreate( &e, -sin( phi ), cos( phi ) );
  if( pdFootUWSign( fuw ) * dr < 0 )
    dr = 0;
  zVec2DCat( regzmp, dr, &e, refpos );
}

void pdFootUWCalcCOMRefPos(zVec2D *lf_pos, zVec2D *rf_pos, zVec2D *ref_pos)
{
  ref_pos->e[pdU] = 0.5 * ( lf_pos->e[pdU] + rf_pos->e[pdU] );
  ref_pos->e[pdW] = 0.5 * ( lf_pos->e[pdW] + rf_pos->e[pdW] );
}

void pdFootUWUpdate(pdFootUW *kf, zVec2D *delta, zVec2D *vel)
{
  pdFootUWCalcRegZMP( kf, delta, vel, pdFootUWRegZMP( kf ) );
  pdFootUWPhi( kf ) = pdFootUWCalcPhi( kf, delta, vel, pdFootUWRegZMP( kf ) );
  pdFootUWCalcRefPos( kf, delta, vel, pdFootUWRegZMP( kf ), pdFootUWRefPos( kf ) );
  if( pdFootUWLandingFn( kf ) )
    pdFootUWLandingFn( kf )( kf, delta, vel, pdFootUWRegZMP( kf ), pdFootUWRefPos( kf ), pdFootUWLandingUtil( kf ) );
}

void pdFootUWFWrite(FILE *fp, pdFootUW *f)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "sign:%g, kappa:%g, dist:%g, phi:%g\n",
           pdFootUWSign(f), pdFootUWKappa(f), pdFootUWDist(f), pdFootUWPhi(f) );
  fprintf( fp, "reguz:%g, regwz:%g\n", pdFootUWRegZMPU(f), pdFootUWRegZMPW(f) );
  fprintf( fp, "refud:%g, refwd:%g\n", pdFootUWRefPosU(f), pdFootUWRefPosW(f) );
}
