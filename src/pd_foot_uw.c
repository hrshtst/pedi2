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

double pdFootUWCalcPhi(pdFootUW *fuw, zVec2D delta, zVec2D vel, zVec2D regzmp)
{
  return atan2( pdFootUWKappa(fuw)*regzmp[pdU],
                1.0+pdFootUWKappa(fuw)*(delta[pdW]-regzmp[pdW]));
}

void pdFootUWCalcRefPos(pdFootUW *fuw, zVec2D delta, zVec2D vel, zVec2D regzmp, zVec2D refpos)
{
  double phi;
  double ud, wd;

  phi = pdFootUWCalcPhi( fuw, delta, vel, regzmp );
  if( zIsTiny( pdFootUWKappa(fuw) ) ){
    ud = regzmp[pdU];
    wd = delta[pdW];
  } else {
    ud = sin( phi ) / pdFootUWKappa( fuw );
    wd = ( 1.0 - cos( phi ) ) / pdFootUWKappa( fuw ) + delta[pdW];
  }
  ud -= 0.5 * pdFootUWSign( fuw ) * pdFootUWDist( fuw ) * sin( phi );
  wd += 0.5 * pdFootUWSign( fuw ) * pdFootUWDist( fuw ) * cos( phi );
  zVec2DCreate( refpos, ud, wd );
}

void pdFootUWCalcCOMRefPos(zVec2D lf_pos, zVec2D rf_pos, zVec2D ref_pos)
{
  ref_pos[pdU] = 0.5 * ( lf_pos[pdU] + rf_pos[pdU] );
  ref_pos[pdW] = 0.5 * ( lf_pos[pdW] + rf_pos[pdW] );
}

void pdFootUWUpdate(pdFootUW *kf, zVec2D delta, zVec2D vel)
{
  pdFootUWCalcRegZMP( kf, delta, vel, pdFootUWRegZMP( kf ) );
  pdFootUWPhi( kf ) = pdFootUWCalcPhi( kf, delta, vel, pdFootUWRegZMP( kf ) );
  pdFootUWCalcRefPos( kf, delta, vel, pdFootUWRegZMP( kf ), pdFootUWRefPos( kf ) );
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
