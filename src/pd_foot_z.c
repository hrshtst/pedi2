#include <pedi2/pd_foot_z.h>

void pdFootZInit(pdFootZ *f, pdCZHrzUW *czuw)
{
  pdFootZCZPtr( f ) = czuw;
  pdFootZSetMaxHeight( f, 0 );
  zListInit( pdFootZSR( f ) );
  f->_vert_num = 0;
  f->_sr_vert = NULL;
  pdFootZSign( f ) = 0;
  zComplexClear( pdFootZZMPPhase( f ) );
  pdFootZFootPhase( f ) = 0;
  pdFootZRefZ( f ) = 0;
}

void pdFootZDestroy(pdFootZ *f)
{
  pdFootZCZPtr( f ) = NULL;
  pdFootZSetMaxHeight( f, 0 );
  zVec3DListDestroy( pdFootZSR( f ), false );
  zFree( pdFootZSRVert( f ) );
  f->_vert_num = 0;
  pdFootZSign( f ) = 0;
  zComplexClear( pdFootZZMPPhase( f ) );
  pdFootZFootPhase( f ) = 0;
  pdFootZRefZ( f ) = 0;
}

void pdFootZSetSR(pdFootZ *f, zVec3D p[], int num)
{
  register int i;
  zVec3D *traversep;

  if( num == 0 || !p ){
    zFree( pdFootZSRVert( f ) );
    zVec3DListDestroy( pdFootZSR( f ), false );
    zListInit( pdFootZSR( f ) );
  } else if( num != f->_vert_num ){
    zFree( pdFootZSRVert( f ) );
    if( !( pdFootZSRVert( f ) = zAlloc( zVec3D, num ) ) ){
      ZALLOCERROR();
      zFree( pdFootZSRVert( f ) );
      exit( EXIT_FAILURE );
    }
  }
  if( num > 0 && p ){
    traversep = pdFootZSRVert( f );
    for( i=0; i<num; i++ )
      zVec3DCreate( traversep++,
                    zVec3DElem(&p[i],zX),
                    zVec3DElem(&p[i],zY),
                    zVec3DElem(&p[i],zZ) );
    zCH2D( pdFootZSR( f ), pdFootZSRVert( f ), num );
  }
  f->_vert_num = num;
}

int pdFootZFindIntersection(pdFootZ *f, zVec2D *zmp, zVec3D ip[])
{
  zVec3DList *ch;
  zVec3DListCell *vc, *vcp;
  int n;
  double k;
  zVec3D v;

  ch = pdFootZSR( f );
  if( zListNum( ch ) == 0 ) return 0;
  vcp = zListHead( ch );
  if( zListNum( ch ) == 1 &&
      zIsTiny( zVec3DElem( vcp->data, pdU ) - zmp->e[pdU] )){
    zVec3DCopy( vcp->data, &ip[0] );
    zVec3DCopy( vcp->data, &ip[1] );
    return 1;
  }
  n = 0;
  zListForEach( ch, vc ){
    if( n > 1 ) break;
    if( zIsTiny( zVec3DElem( vc->data,  pdU ) - zmp->e[pdU] ) ){
      /* intersect on a vertex */
      zVec3DCopy( vc->data, &ip[n++] );
    } else if( ( zVec3DElem( vc->data,  pdU ) - zmp->e[pdU] ) *
               ( zVec3DElem( vcp->data, pdU ) - zmp->e[pdU] ) < 0 &&
               !zIsTiny( vcp->data->e[0] - vc->data->e[0] ) ){
      /* intersect on an edge */
      k = ( zmp->e[pdU] - vc->data->e[0] ) / ( vcp->data->e[0] - vc->data->e[0] );
      zVec3DSub( vcp->data, vc->data, &v );
      zVec3DCat( vc->data, k, &v, &ip[n++] );
    }
    vcp = vc;
  }
  return n;
}

int pdFootZFindInnerPoint(pdFootZ *f, zVec2D *zmp, zVec3D *p)
{
  zVec3D ip[2];
  int n_ip;

  n_ip = pdFootZFindIntersection( f, zmp, ip );
  if( n_ip == 0 ) return 0;
  if( n_ip == 1 )
    zVec3DCopy( &ip[0], p );
  else {
    if( pdFootZSign(f) * ( ip[1].e[pdW] - ip[0].e[pdW] ) > 0 )
      zVec3DCopy( &ip[0], p );
    else
      zVec3DCopy( &ip[1], p );
  }
  return 1;
}

double pdFootZCalcFootPhase(pdFootZ *pf, zVec2D *delta, zVec2D *vel, zVec2D *zmp)
{
  double r2, r, dr, da, d;
  zVec3D inner_p;
  zComplex p_in, p;

  if( !pdFootZIsSRSet( pf ) ||
      !pdFootZFindInnerPoint( pf, zmp, &inner_p ) )
    /* pf is floating or ZMP does not lie on SR */
    return 0.0;
  pdFootZCalcZMPPhase( pf, delta, vel, zmp, &pf->pz );
  r2 = zComplexSqrAbs( &pf->pz );
  r  = sqrt( r2 );
  dr = zVec3DElem( &inner_p, pdW ) - delta->e[pdW];
  da = acos( fabs(dr) / r );
  if( ( d = r2 - zSqr(dr) ) > 0 ){
    zComplexCreate( &p_in, dr, -pdFootZSign(pf)*sqrt(d) );
    zComplexCDiv( &pf->pz, &p_in, &p );
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

void pdFootZUpdate(pdFootZ *pf, pdFootZ *kf, zVec2D *delta, zVec2D *vel, zVec2D *zmp)
{
  kf->phase = pdFootZCalcFootPhase( pf, delta, vel, zmp );
  kf->refz = pdFootZCalcRefZ( kf, kf->phase, &pf->pz );
}

void pdFootZFWrite(FILE *fp, pdFootZ *f)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "sign:%g, h:%g, rho:%g, dist:%g\n",
           pdFootZSign(f), pdFootZMaxHeight(f), pdFootZRho(f), pdFootZDist(f) );
  fprintf( fp, "pz:" );zComplexFWrite( fp, pdFootZZMPPhase(f) );
  fprintf( fp, ", phase:%g, refz:%g\n", pdFootZFootPhase(f), pdFootZRefZ(f) );
}
