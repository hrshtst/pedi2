#include <pedi2/pd_cz_hrz_uw.h>

void pdCZHrzUWInit(pdCZHrzUW *hrz, pdCZVrt *vrt)
{
  pdCZHrzUInit( &hrz->_u, &pdCZHrzUWKappa(hrz), &pdCZVrtZeta(vrt) );
  pdCZHrzWInit( &hrz->_w, &pdCZHrzUWKappa(hrz), &pdCZVrtZeta(vrt) );
  hrz->_vrt = vrt;
  pdCZHrzUWSetKappa( hrz, 0 );
  zListInit( pdCZHrzUWSR(hrz) );
  zArrayInit( pdCZHrzUWSRVert(hrz)->data.array );
  hrz->_vert_num = 0;
  zVec2DZero( pdCZHrzUWZMP(hrz) );
  zVec2DZero( pdCZHrzUWAcc(hrz) );
}

void pdCZHrzUWDestroy(pdCZHrzUW *hrz)
{
  pdCZHrzUDestroy( &hrz->_u );
  pdCZHrzWDestroy( &hrz->_w );
  hrz->_vrt = NULL;
  pdCZHrzUWSetKappa( hrz, 0 );
  zLoop3DDestroy( pdCZHrzUWSR(hrz) );
  if( zArrayBuf( pdCZHrzUWSRVert(hrz)->data.array ) )
    zVec3DDataDestroy( pdCZHrzUWSRVert(hrz) );
  hrz->_vert_num = 0;
  zVec2DZero( pdCZHrzUWZMP(hrz) );
  zVec2DZero( pdCZHrzUWAcc(hrz) );
}

void pdCZHrzUWSetSR(pdCZHrzUW *hrz, zVec3D p[], int num)
{
  register int i;

  if( num == 0 || !p ){
    if( zArrayBuf( pdCZHrzUWSRVert(hrz)->data.array ) )
      zVec3DDataDestroy( pdCZHrzUWSRVert(hrz) );
    zLoop3DDestroy( pdCZHrzUWSR(hrz) );
    zListInit( pdCZHrzUWSR(hrz) );
  } else if( num != hrz->_vert_num ){
    if( zArrayBuf( pdCZHrzUWSRVert(hrz)->data.array ) )
      zVec3DDataDestroy( pdCZHrzUWSRVert(hrz) );
    zVec3DDataInitArray( pdCZHrzUWSRVert(hrz), num );
  }
  if( num > 0 && p ){
    zVec3DDataRewind( pdCZHrzUWSRVert(hrz) );
    for( i=0; i<num; i++ )
      zVec3DDataAdd( pdCZHrzUWSRVert(hrz), &p[i] );
    zVec3DDataConvexHull2D( pdCZHrzUWSRVert(hrz), pdCZHrzUWSR(hrz) );
  }
  hrz->_vert_num = num;
}

void pdCZHrzUWCalcZMP(pdCZHrzUW *hrz, zVec2D *delta, zVec2D *vel, zVec2D *zmp)
{
  zVec3D p, cp;

  zVec3DCreate( &p,
                pdCZHrzUWCalcSimZMPU( hrz, delta, vel ),
                pdCZHrzUWCalcSimZMPW( hrz, delta, vel ),
                pdCZVrtCalcZMP( hrz->_vrt ) );
  if( pdCZHrzUWIsSRSet( hrz ) ){
    zConvexHull2DClosest( pdCZHrzUWSR(hrz), &p, &cp );
    zVec2DCreate( zmp, cp.c.x, cp.c.y );
  } else {
    zVec2DCreate( zmp, p.c.x, p.c.y );
  }
}

void pdCZHrzUWCalcAcc(pdCZHrzUW *hrz, zVec2D *zmp, zVec2D *acc)
{
  double ddu, ddw;

  ddu = -zSqr( pdCZHrzUWZeta( hrz ) ) * zmp->e[pdU];
  ddw = -zSqr( pdCZHrzUWZeta( hrz ) ) * zmp->e[pdW];
  zVec2DCreate( acc, ddu, ddw );
}

void pdCZHrzUWCalcZMPPhase(pdCZHrzUW *hrz, zVec2D *delta, zVec2D *vel, zVec2D *zmp, zComplex *pz)
{
  double q1, q2, zeta;

  q1 = pdCZHrzUWQ1W( hrz );
  q2 = pdCZHrzUWQ2W( hrz );
  zeta = pdCZHrzUWZeta( hrz );
  zComplexCreate( pz, zmp->e[pdW]-delta->e[pdW],
                  -( q1*q2 + 1.0 ) * vel->e[pdW] / ( zeta * sqrt( q1*q2 ) ) );
}

void pdCZHrzUWUpdate(pdCZHrzUW *hrz, zVec2D *delta, zVec2D *vel)
{
  pdCZHrzUWCalcZMP( hrz, delta, vel, pdCZHrzUWZMP(hrz) );
  pdCZHrzUWCalcAcc( hrz, pdCZHrzUWZMP(hrz), pdCZHrzUWAcc(hrz));
}

void pdCZHrzUWFWrite(FILE *fp, pdCZHrzUW *hrz)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "vud:%f, vwd:%f\n", pdCZHrzUWRefVelU(hrz), pdCZHrzUWRefVelW(hrz) );
  fprintf( fp, "qu1:%f, qu2:%f\n", pdCZHrzUWQ1U(hrz), pdCZHrzUWQ2U(hrz) );
  fprintf( fp, "qw1:%f, qw2:%f\n", pdCZHrzUWQ1W(hrz), pdCZHrzUWQ2W(hrz) );
  fprintf( fp, "rho:%f, kr:%f, dist:%f\n",
           pdCZHrzUWRho(hrz), pdCZHrzUWKr(hrz), pdCZHrzUWDist(hrz) );
  fprintf( fp, "kappa:%f\n", pdCZHrzUWKappa(hrz) );
  fprintf( fp, "refuz:%f, refwz:%f\n", pdCZHrzUWZMPU(hrz), pdCZHrzUWZMPW(hrz) );
  fprintf( fp, "refddu:%f, refddw:%f\n", pdCZHrzUWAccU(hrz), pdCZHrzUWAccW(hrz) );
}
