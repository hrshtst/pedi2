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

static double _pdFootZFindInnerEdgeW(pdFootZ *f);
double _pdFootZFindInnerEdgeW(pdFootZ *f)
{
  double inner_w;
  zVec3DListCell *cp;

  inner_w = zVec3DElem( zListTail( pdFootZSR( f ) )->data, pdW );
  zListForEach( pdFootZSR( f ), cp ){
    if( pdFootZSign(f) > 0 )
      /* left foot */
      inner_w = zMin( inner_w, zVec3DElem( cp->data, pdW ) );
    else
      /* right foot */
      inner_w = zMax( inner_w, zVec3DElem( cp->data, pdW ) );
  }
  return inner_w;
}

double pdFootZCalcFootPhase(pdFootZ *pf, zVec2D delta, zComplex *pz)
{
  double r2, r, dr, da, d;
  double inner_w;
  zComplex p_in, p;

  if( !pdFootZSR( pf ) ) return 0.0; /* pf is floating */
  r2 = zComplexSqrAbs( pz );
  r  = sqrt( r2 );
  inner_w = _pdFootZFindInnerEdgeW( pf );
  dr = inner_w - delta[pdW];
  da = acos( fabs(dr) / r );
  if( ( d = r2 - zSqr(dr) ) > 0 ){
    zComplexCreate( &p_in, dr, -pdFootZSign(pf)*sqrt(d) );
    zComplexCDiv( pz, &p_in, &p );
    return zLimit( 0.5 * zComplexArg(&p)/da, 0, 1 );
  } else
    return 0.0;
}

static double _sigma(double rho);
double _sigma(double rho)
{
  return zLimit( (rho*zE-1)/(zE-1), 0, 1 );
}

double pdFootZCalcRefZ(pdFootZ *kf, double phase, zComplex *pz)
{
  double dh, d;

  d = 0.5 * pdFootZDist( kf );
  dh = zComplexAbs(pz) / d * _sigma( pdFootZRho(kf) );
  return zCycloidY( 0, pdFootZMaxHeight(kf) * dh, phase );
}
