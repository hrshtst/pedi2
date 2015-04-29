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
