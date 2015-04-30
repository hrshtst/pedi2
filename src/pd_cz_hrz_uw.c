#include <pedi2/pd_cz_hrz_uw.h>

void pdCZHrzUWInit(pdCZHrzUW *hrz, pdCZVrt *vrt)
{
  pdCZHrzUInit( &hrz->_u, &pdCZHrzUWKappa(hrz), &pdCZVrtZeta(vrt) );
  pdCZHrzWInit( &hrz->_w, &pdCZHrzUWKappa(hrz), &pdCZVrtZeta(vrt) );
  hrz->_vrt = vrt;
  pdCZHrzUWSetKappa( hrz, 0 );
  zListInit( pdCZHrzUWSR(hrz) );
  hrz->_vert_num = 0;
  zVec2DClear( pdCZHrzUWZMP(hrz) );
  zVec2DClear( pdCZHrzUWAcc(hrz) );
  hrz->_sr_vert = NULL;
}

void pdCZHrzUWDestroy(pdCZHrzUW *hrz)
{
  pdCZHrzUDestroy( &hrz->_u );
  pdCZHrzWDestroy( &hrz->_w );
  hrz->_vrt = NULL;
  pdCZHrzUWSetKappa( hrz, 0 );
  zVec3DListDestroy( pdCZHrzUWSR(hrz), false );
  zFree( pdCZHrzUWSRVert( hrz ) );
  hrz->_vert_num = 0;
  zVec2DClear( pdCZHrzUWZMP(hrz) );
  zVec2DClear( pdCZHrzUWAcc(hrz) );
}

void pdCZHrzUWSetSR(pdCZHrzUW *hrz, zVec3D p[], int num)
{
  register int i;
  zVec3D *traversep;

  if( num == 0 || !p ){
    zFree( pdCZHrzUWSRVert(hrz) );
    zVec3DListDestroy( pdCZHrzUWSR(hrz), false );
    zListInit( pdCZHrzUWSR(hrz) );
  } else if( num != hrz->_vert_num ){
    zFree( pdCZHrzUWSRVert(hrz) );
    if( !( pdCZHrzUWSRVert(hrz) = zAlloc( zVec3D, num ) ) ){
      ZALLOCERROR();
      zFree( pdCZHrzUWSRVert(hrz) );
      exit( EXIT_FAILURE );
    }
  }
  if( num > 0 && p ){
    traversep = pdCZHrzUWSRVert(hrz);
    for( i=0; i<num; i++ )
      zVec3DCreate( traversep++,
                    zVec3DElem(&p[i],zX),
                    zVec3DElem(&p[i],zY),
                    zVec3DElem(&p[i],zZ) );
    zCH2D( pdCZHrzUWSR(hrz), pdCZHrzUWSRVert(hrz), num );
  }
  hrz->_vert_num = num;
}

void pdCZHrzUWCalcZMP(pdCZHrzUW *hrz, zVec2D delta, zVec2D vel, zVec2D zmp)
{
  zVec3D p, cp;

  zVec3DCreate( &p,
                pdCZHrzUWCalcSimZMPU( hrz, delta, vel ),
                pdCZHrzUWCalcSimZMPW( hrz, delta, vel ),
                pdCZVrtCalcZMP( hrz->_vrt ) );
  if( pdCZHrzUWIsSRSet( hrz ) ){
    zCH2DClosest( &hrz->_sr, &p, &cp );
    zVec2DCreate( zmp, zVec3DElem( &cp, zX ), zVec3DElem( &cp, zY ) );
  } else {
    zVec2DCreate( zmp, zVec3DElem( &p, zX ), zVec3DElem( &p, zY ) );
  }
}

void pdCZHrzUWCalcAcc(pdCZHrzUW *hrz, zVec2D zmp, zVec2D acc)
{
  double ddu, ddw;

  ddu = -zSqr( pdCZHrzUWZeta( hrz ) ) * zmp[pdU];
  ddw = -zSqr( pdCZHrzUWZeta( hrz ) ) * zmp[pdW];
  zVec2DCreate( acc, ddu, ddw );
}

void pdCZHrzUWCalcZMPPhase(pdCZHrzUW *hrz, zVec2D delta, zVec2D vel, zVec2D zmp, zComplex *pz)
{
  double q1, q2, zeta;

  q1 = pdCZHrzUWQ1W( hrz );
  q2 = pdCZHrzUWQ2W( hrz );
  zeta = pdCZHrzUWZeta( hrz );
  zComplexCreate( pz, zmp[pdW]-delta[pdW],
                  -( q1*q2 + 1.0 ) * vel[pdW] / ( zeta * sqrt( q1*q2 ) ) );
}

void pdCZHrzUWUpdate(pdCZHrzUW *hrz, zVec2D delta, zVec2D vel)
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
