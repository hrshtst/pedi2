#include <pedi2/pd_foot_uw.h>

void pdFootUWInit(pdFootUW *fuw, pdCZHrzUW *czuw, byte dir)
{
  pdFootUWCZPtr( fuw ) = czuw;
  pdFootUWPhi( fuw ) = 0;
  pdFootUWRegZMPU( fuw ) = 0;
  pdFootUWRegZMPW( fuw ) = 0;
  pdFootUWRefPosU( fuw ) = 0;
  pdFootUWRefPosW( fuw ) = 0;
  if( dir == PD_FOOT_LEFT )
    pdFootUWSign( fuw ) = 1.0;
  else if( dir == PD_FOOT_RIGHT )
    pdFootUWSign( fuw ) = -1.0;
  else{
    ZRUNERROR( "invalid foot dir specified - %d", dir );
    pdFootUWDestroy( fuw );
    return;
  }
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

void pdFootUWUpdate(pdFootUW *f, zVec2D delta, zVec2D vel)
{
  pdFootUWCalcRegZMP( f, delta, vel, pdFootUWRegZMP( f ) );
  pdFootUWPhi( f ) = pdFootUWCalcPhi( f, delta, vel, pdFootUWRegZMP( f ) );
  pdFootUWCalcRefPos( f, delta, vel, pdFootUWRegZMP( f ), pdFootUWRefPos( f ) );
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
