#include <pedi2/pd_cz_hrz_mov.h>

void pdCZHrzMovInit(pdCZHrzMov *hrz, pdCZVrt *vrt)
{
  pdCZHrzUInit( &hrz->_u, &pdCZHrzMovKappa(hrz), &pdCZVrtZeta(vrt) );
  pdCZHrzWInit( &hrz->_w, &pdCZHrzMovKappa(hrz), &pdCZVrtZeta(vrt) );
  hrz->_vrt = vrt;
  pdCZHrzMovSetKappa( hrz, 0 );
  zListInit( &pdCZHrzMovSR(hrz) );
  hrz->_vert_num = 0;
  zVec2DClear( pdCZHrzMovZMP(hrz) );
  zVec2DClear( pdCZHrzMovAcc(hrz) );
  hrz->_sr_vert = NULL;
}

void pdCZHrzMovDestroy(pdCZHrzMov *hrz)
{
  pdCZHrzUDestroy( &hrz->_u );
  pdCZHrzWDestroy( &hrz->_w );
  hrz->_vrt = NULL;
  pdCZHrzMovSetKappa( hrz, 0 );
  zVec3DListDestroy( &pdCZHrzMovSR(hrz), false );
  zFree( pdCZHrzMovSRVert( hrz ) );
  hrz->_vert_num = 0;
  zVec2DClear( pdCZHrzMovZMP(hrz) );
  zVec2DClear( pdCZHrzMovAcc(hrz) );
}

void pdCZHrzMovSetSR(pdCZHrzMov *hrz, zVec3D p[], int num)
{
  register int i;
  zVec3D *traversep;

  if( num != hrz->_vert_num ){
    zFree( pdCZHrzMovSRVert(hrz) );
    if( !( pdCZHrzMovSRVert(hrz) = zAlloc( zVec3D, num ) ) ){
      ZALLOCERROR();
      zFree( pdCZHrzMovSRVert(hrz) );
      exit( EXIT_FAILURE );
    }
  }
  traversep = pdCZHrzMovSRVert(hrz);
  for( i=0; i<num; i++ )
    zVec3DCreate( traversep++,
                  zVec3DElem(&p[i],zX),
                  zVec3DElem(&p[i],zY),
                  zVec3DElem(&p[i],zZ) );
  zCH2D( &pdCZHrzMovSR(hrz), pdCZHrzMovSRVert(hrz), num );
  hrz->_vert_num = num;
}

void pdCZHrzMovCalcZMP(pdCZHrzMov *hrz, zVec2D delta, zVec2D vel, zVec2D zmp)
{
  zVec3D p, cp;

  zVec3DCreate( &p,
                pdCZHrzMovCalcSimZMPU( hrz, delta, vel ),
                pdCZHrzMovCalcSimZMPW( hrz, delta, vel ),
                pdCZVrtCalcZMP( hrz->_vrt ) );
  if( pdCZHrzMovIsSRSet( hrz ) ){
    zCH2DClosest( &hrz->_sr, &p, &cp );
    zVec2DCreate( zmp, zVec3DElem( &cp, zX ), zVec3DElem( &cp, zY ) );
  } else {
    zVec2DCreate( zmp, zVec3DElem( &p, zX ), zVec3DElem( &p, zY ) );
  }
}

void pdCZHrzMovCalcAcc(pdCZHrzMov *hrz, zVec2D zmp, zVec2D acc)
{
  double ddu, ddw;

  ddu = -zSqr( pdCZHrzMovZeta( hrz ) ) * zmp[pdU];
  ddw = -zSqr( pdCZHrzMovZeta( hrz ) ) * zmp[pdW];
  zVec2DCreate( acc, ddu, ddw );
}

void pdCZHrzMovUpdate(pdCZHrzMov *hrz, zVec2D delta, zVec2D vel)
{
  pdCZHrzMovCalcZMP( hrz, delta, vel, pdCZHrzMovZMP(hrz) );
  pdCZHrzMovCalcAcc( hrz, pdCZHrzMovZMP(hrz), pdCZHrzMovAcc(hrz));
}

void pdCZHrzMovFWrite(FILE *fp, pdCZHrzMov *hrz)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "vud:%f, vwd:%f\n", pdCZHrzMovRefVelU(hrz), pdCZHrzMovRefVelW(hrz) );
  fprintf( fp, "qu1:%f, qu2:%f\n", pdCZHrzMovQ1U(hrz), pdCZHrzMovQ2U(hrz) );
  fprintf( fp, "qw1:%f, qw2:%f\n", pdCZHrzMovQ1W(hrz), pdCZHrzMovQ2W(hrz) );
  fprintf( fp, "rho:%f, kr:%f, dist:%f\n",
           pdCZHrzMovRho(hrz), pdCZHrzMovKr(hrz), pdCZHrzMovDist(hrz) );
  fprintf( fp, "kappa:%f\n", pdCZHrzMovKappa(hrz) );
  fprintf( fp, "refuz:%f, refwz:%f\n", pdCZHrzMovZMPU(hrz), pdCZHrzMovZMPW(hrz) );
  fprintf( fp, "refddu:%f, refddw:%f\n", pdCZHrzMovAccU(hrz), pdCZHrzMovAccW(hrz) );
}
