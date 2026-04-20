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

double pdFootUWCalcPhi(pdFootUW *fuw, zVec2D *delta, zVec2D *regzmp, double refdist)
{
  return atan2( pdFootUWKappa(fuw)*regzmp->e[pdU],
                1.0+pdFootUWKappa(fuw)*(delta->e[pdW]-regzmp->e[pdW]));
}

void pdFootUWCalcRefPos_old(pdFootUW *fuw, zVec2D *delta, zVec2D *regzmp, zVec2D *refpos, double refdist)
{
  double phi;
  double ud, wd;

  phi = pdFootUWCalcPhi( fuw, delta, regzmp, refdist );
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

void pdFootUWCalcRefPos(pdFootUW *fuw, zVec2D *delta, zVec2D *regzmp, zVec2D *refpos, double refdist)
{
  double phi;
  double dr;
  zVec2D e;

  phi = pdFootUWCalcPhi( fuw, delta, regzmp, refdist );
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

void pdFootUWUpdate(pdFootUW *kf, zVec2D *delta, zVec2D *vel, double refdist)
{
  pdFootUWCalcRegZMP( kf, delta, vel, pdFootUWRegZMP( kf ) );
  pdFootUWPhi( kf ) = pdFootUWCalcPhi( kf, delta, pdFootUWRegZMP( kf ), refdist );
  pdFootUWCalcRefPos( kf, delta, pdFootUWRegZMP( kf ), pdFootUWRefPos( kf ), refdist );
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
