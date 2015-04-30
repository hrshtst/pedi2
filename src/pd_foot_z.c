#include <pedi2/pd_foot_z.h>

void pdFootZInit(pdFootZ *f, pdCZHrzUW *czuw)
{
  pdFootZCZPtr( f ) = czuw;
  pdFootZSetMaxHeight( f, 0 );
  pdFootZSetSR( f, NULL );
  pdFootZSign( f ) = 0;
  zComplexClear( pdFootZZMPPhase( f ) );
  pdFootZFootPhase( f ) = 0;
  pdFootZRefHeight( f ) = 0;
}

void pdFootZDestroy(pdFootZ *f)
{
  pdFootZCZPtr( f ) = NULL;
  pdFootZSetMaxHeight( f, 0 );
  pdFootZSetSR( f, NULL );
  pdFootZSign( f ) = 0;
  zComplexClear( pdFootZZMPPhase( f ) );
  pdFootZFootPhase( f ) = 0;
  pdFootZRefHeight( f ) = 0;
}
