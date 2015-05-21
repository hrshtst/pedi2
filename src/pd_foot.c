#include <pedi2/pd_foot.h>

void pdFootInit(pdFoot *f, pdCZHrz *cz, byte dir, double dt)
{
  pdFootUWInit( pdFootUWPtr( f ), pdCZHrzUWPtr( cz ) );
  pdFootZInit( pdFootZPtr( f ), pdCZHrzUWPtr( cz ) );
  pdFootSetPos( f, 0, 0, 0 );
  pdFootSetDesPos( f, 0, 0, 0 );
  pdFootSetAtt( f, 0, 0, 0 );
  pdFootSetDesAtt( f, 0, 0, 0 );
  pdFootSetSR( f, NULL );
  pdFootCZPtr( f ) = cz;
  pdFootResetTime( f );
  pdFootSetTimeStep( f, dt );
  pdFootSetTrXK( f, 0 );
  pdFootSetTrXC( f, 0 );
  pdFootSetTrXOld( f, 0 );
  pdFootSetTrYK( f, 0 );
  pdFootSetTrYC( f, 0 );
  pdFootSetTrYOld( f, 0 );
  pdFootSetTrZK( f, 0 );
  pdFootSetTrZC( f, 0 );
  pdFootSetTrZOld( f, 0 );
  pdFootRefPosX( f ) = 0;
  pdFootRefPosY( f ) = 0;
  pdFootRefPosZ( f ) = 0;
  pdFootRefAttX( f ) = 0;
  pdFootRefAttY( f ) = 0;
  pdFootRefAttZ( f ) = 0;
  if( dir == PD_FOOT_LEFT ){
    pdFootUWSign( pdFootUWPtr(f) ) = 1;
    pdFootZSign( pdFootZPtr(f) ) = 1;
  } else if( dir == PD_FOOT_RIGHT ){
    pdFootUWSign( pdFootUWPtr(f) ) = -1;
    pdFootZSign( pdFootZPtr(f) ) = -1;
  } else{
    ZRUNERROR( "invalid foot dir specified - %d", dir );
    pdFootDestroy( f );
    return;
  }
}

void pdFootDestroy(pdFoot *f)
{
  pdFootUWDestroy( pdFootUWPtr( f ) );
  pdFootZDestroy( pdFootZPtr( f ) );
  pdFootSetPos( f, 0, 0, 0 );
  pdFootSetDesPos( f, 0, 0, 0 );
  pdFootSetAtt( f, 0, 0, 0 );
  pdFootSetDesAtt( f, 0, 0, 0 );
  pdFootSetSR( f, NULL );
  pdFootCZPtr( f ) = NULL;
  pdFootResetTime( f );
  pdFootSetTimeStep( f, 0 );
  pdFootSetTrXK( f, 0 );
  pdFootSetTrXC( f, 0 );
  pdFootSetTrXOld( f, 0 );
  pdFootSetTrYK( f, 0 );
  pdFootSetTrYC( f, 0 );
  pdFootSetTrYOld( f, 0 );
  pdFootSetTrZK( f, 0 );
  pdFootSetTrZC( f, 0 );
  pdFootSetTrZOld( f, 0 );
  pdFootRefPosX( f ) = 0;
  pdFootRefPosY( f ) = 0;
  pdFootRefPosZ( f ) = 0;
  pdFootRefAttX( f ) = 0;
  pdFootRefAttY( f ) = 0;
  pdFootRefAttZ( f ) = 0;
}

void pdFootXformSRXYtoUW(pdFoot *f, zVec3DList *sr)
{
  zVec3D *p, *pp;
  zVec3DListCell *cp;
  zVec2D xy, uw;

  if( !sr || zListNum( sr ) == 0 ){
    pdFootZSetSR( pdFootZPtr(f), NULL, 0 );
    return;
  } else {
    if( !( p = zAlloc( zVec3D, zListNum( sr ) ) ) ){
      ZALLOCERROR();
      zFree( p );
      exit( EXIT_FAILURE );
    }
    pp = p;
    zListForEach( sr, cp ){
      zVec2DCreate( xy, zVec3DElem(cp->data,zX), zVec3DElem(cp->data,zY) );
      pdFootXformXYtoUW( f, xy, uw );
      zVec3DCreate( pp++, uw[pdU], uw[pdW], zVec3DElem(cp->data,zZ) );
    }
    pdFootZSetSR( pdFootZPtr(f), p, zListNum(sr) );
    zFree( p );
  }
}

static void _pdFootUpdateSOL(pdFoot *f, zVec3D *p, zVec3D *pd, zVec3D *refp, zAxis i, double dt);
void _pdFootUpdateSOL(pdFoot *f, zVec3D *p, zVec3D *pd, zVec3D *refp, zAxis i, double dt)
{
  refp->e[i] =
      ( f->_sol._k[i]*dt*dt * pd->e[i] + ( f->_sol._c[i]*dt+2 ) * p->e[i] - f->_sol._old[i] ) / ( 1 + f->_sol._c[i]*dt + f->_sol._k[i]*dt*dt );
  f->_sol._old[i] = p->e[i];
}

void pdFootCalcRefPos(pdFoot *f, zVec3D *p, zVec3D *pd, zVec3D *refp)
{
  _pdFootUpdateSOL( f, p, pd, refp, zX, pdFootTimeStep(f) );
  _pdFootUpdateSOL( f, p, pd, refp, zY, pdFootTimeStep(f) );
  _pdFootUpdateSOL( f, p, pd, refp, zZ, pdFootTimeStep(f) );
}

void pdFootCalcRefAtt(pdFoot *f, zVec3D *pd, zVec3D *refa)
{
  double theta, phi;

  /* rotational angle of base link */
  theta = pdCZHrzTheta( pdFootCZPtr( f ) );
  /* rotational angle of foot w.r.t moving frame */
  phi = pdFootUWPhi( pdFootUWPtr( f ) );
  zVec3DCreate( refa, theta + phi, 0, 0 );
  /* zVec3DCreate( refa, ( theta + zPI_2 ) + phi, 0, 0 ); */
}

void _pdFootDesPosUpdate(pdFoot *kf, pdFoot *pf, zVec2D delta, zVec2D vel, zVec3D *zmp, zVec3DList *pfsr)
{
  zVec2D xy;
  zVec2D zmpxy, zmpuw;

  /* world frame -> moving frame */
  zVec2DCreate( zmpxy, zVec3DElem(zmp,zX), zVec3DElem(zmp,zY) );
  pdFootXformXYtoUW( pf, zmpxy, zmpuw );
  pdFootXformSRXYtoUW( pf, pfsr );
  /* update desired position */
  pdFootUWUpdate( pdFootUWPtr( kf ), delta, vel );
  pdFootZUpdate( pdFootZPtr( pf ), pdFootZPtr( kf ), delta, vel, zmpuw );
  /* moving frame -> world frame */
  pdFootXformUWtoXY( kf, pdFootUWRefPos( pdFootUWPtr( kf ) ), xy );
  /* set desired position */
  pdFootSetDesPos( kf, xy[zX], xy[zY], pdFootZRefZ( pdFootZPtr( kf ) ) );
}

void _pdFootRefPosUpdate(pdFoot *kf, zVec3D *kfp, zVec3D *kfa, zVec3DList *kfsr)
{
  /* update current position */
  pdFootSetPosVec( kf, kfp );
  pdFootSetAttVec( kf, kfa );
  /* update referential position */
  if( !kfsr || pdFootDesPosZ(kf) > 0 || pdFootRefPosZ(kf) > 0 ){
    pdFootCalcRefPos( kf, pdFootPos( kf ), pdFootDesPos( kf ), pdFootRefPos( kf ) );
    pdFootCalcRefAtt( kf, pdFootDesPos( kf ), pdFootRefAtt( kf ) );
  } else {
    zVec3DCopy( pdFootPos( kf ), pdFootRefPos( kf ) );
    zVec3DCopy( pdFootAtt( kf ), pdFootRefAtt( kf ) );
  }
}

void pdFootUpdate(pdFoot *lf, pdFoot *rf, zVec2D delta, zVec2D vel, zVec3D *zmp, zVec3D *lfp, zVec3D *rfp, zVec3D *lfa, zVec3D *rfa, zVec3DList *lfsr, zVec3DList *rfsr)
{
  _pdFootDesPosUpdate( lf, rf, delta, vel, zmp, rfsr );
  _pdFootDesPosUpdate( rf, lf, delta, vel, zmp, lfsr );
  _pdFootRefPosUpdate( lf, lfp, lfa, lfsr );
  _pdFootRefPosUpdate( rf, rfp, rfa, rfsr );
}

void pdFootFWrite(FILE *fp, pdFoot *lf, pdFoot *rf)
{
  /* for debug */
  fprintf( fp, "--\n" );
  fprintf( fp, "lf    x:%g,    y:%g,    z:%g\n", zVec3DElem(&lf->_p,zX), zVec3DElem(&lf->_p,zY), zVec3DElem(&lf->_p,zZ) );
  fprintf( fp, "lf   xd:%g,   yd:%g,   zd:%g\n", zVec3DElem(&lf->_pd,zX), zVec3DElem(&lf->_pd,zY), zVec3DElem(&lf->_pd,zZ) );
  fprintf( fp, "lf refx:%g, refy:%g, refz:%g\n", zVec3DElem(&lf->refp,zX), zVec3DElem(&lf->refp,zY), zVec3DElem(&lf->refp,zZ) );
  fprintf( fp, "lf azim:%g, elev:%g, tilt:%g\n", zVec3DElem(&lf->_a,zX), zVec3DElem(&lf->_a,zY), zVec3DElem(&lf->_a,zZ) );
  fprintf( fp, "lf desa:%g, dese:%g, dest:%g\n", zVec3DElem(&lf->_ad,zX), zVec3DElem(&lf->_ad,zY), zVec3DElem(&lf->_ad,zZ) );
  fprintf( fp, "lf refa:%g, refe:%g, reft:%g\n", zVec3DElem(&lf->refa,zX), zVec3DElem(&lf->refa,zY), zVec3DElem(&lf->refa,zZ) );
  fprintf( fp, "lf sign:%g, kappa:%g, dist:%g, phi:%g\n",
           pdFootUWSign(pdFootUWPtr(lf)), pdFootUWKappa(pdFootUWPtr(lf)), pdFootUWDist(pdFootUWPtr(lf)), pdFootUWPhi(pdFootUWPtr(lf)) );
  fprintf( fp, "lf reguz:%g, regwz:%g\n", pdFootUWRegZMPU(pdFootUWPtr(lf)), pdFootUWRegZMPW(pdFootUWPtr(lf)) );
  fprintf( fp, "lf refud:%g, refwd:%g\n", pdFootUWRefPosU(pdFootUWPtr(lf)), pdFootUWRefPosW(pdFootUWPtr(lf)) );
  fprintf( fp, "lf sign:%g, h:%g, rho:%g, dist:%g\n",
           pdFootZSign(pdFootZPtr(lf)), pdFootZMaxHeight(pdFootZPtr(lf)), pdFootZRho(pdFootZPtr(lf)), pdFootZDist(pdFootZPtr(lf)) );
  fprintf( fp, "lf pz:" );zComplexFWrite( fp, pdFootZZMPPhase(pdFootZPtr(lf)) );
  fprintf( fp, ", phase:%g, zd:%g\n", pdFootZFootPhase(pdFootZPtr(lf)), pdFootZRefZ(pdFootZPtr(lf)) );
  fprintf( fp, "--\n" );
  fprintf( fp, "rf    x:%g,    y:%g,    z:%g\n", zVec3DElem(&rf->_p,zX), zVec3DElem(&rf->_p,zY), zVec3DElem(&rf->_p,zZ) );
  fprintf( fp, "rf   xd:%g,   yd:%g,   zd:%g\n", zVec3DElem(&rf->_pd,zX), zVec3DElem(&rf->_pd,zY), zVec3DElem(&rf->_pd,zZ) );
  fprintf( fp, "rf refx:%g, refy:%g, refz:%g\n", zVec3DElem(&rf->refp,zX), zVec3DElem(&rf->refp,zY), zVec3DElem(&rf->refp,zZ) );
  fprintf( fp, "rf azim:%g, elev:%g, tilt:%g\n", zVec3DElem(&rf->_a,zX), zVec3DElem(&rf->_a,zY), zVec3DElem(&rf->_a,zZ) );
  fprintf( fp, "rf desa:%g, dese:%g, dest:%g\n", zVec3DElem(&rf->_ad,zX), zVec3DElem(&rf->_ad,zY), zVec3DElem(&rf->_ad,zZ) );
  fprintf( fp, "rf refa:%g, refe:%g, reft:%g\n", zVec3DElem(&rf->refa,zX), zVec3DElem(&rf->refa,zY), zVec3DElem(&rf->refa,zZ) );
  fprintf( fp, "rf sign:%g, kappa:%g, dist:%g, phi:%g\n",
           pdFootUWSign(pdFootUWPtr(rf)), pdFootUWKappa(pdFootUWPtr(rf)), pdFootUWDist(pdFootUWPtr(rf)), pdFootUWPhi(pdFootUWPtr(rf)) );
  fprintf( fp, "rf reguz:%g, regwz:%g\n", pdFootUWRegZMPU(pdFootUWPtr(rf)), pdFootUWRegZMPW(pdFootUWPtr(rf)) );
  fprintf( fp, "rf refud:%g, refwd:%g\n", pdFootUWRefPosU(pdFootUWPtr(rf)), pdFootUWRefPosW(pdFootUWPtr(rf)) );
  fprintf( fp, "rf sign:%g, h:%g, rho:%g, dist:%g\n",
           pdFootZSign(pdFootZPtr(rf)), pdFootZMaxHeight(pdFootZPtr(rf)), pdFootZRho(pdFootZPtr(rf)), pdFootZDist(pdFootZPtr(rf)) );
  fprintf( fp, "rf pz:" );zComplexFWrite( fp, pdFootZZMPPhase(pdFootZPtr(rf)) );
  fprintf( fp, ", phase:%g, zd:%g\n", pdFootZFootPhase(pdFootZPtr(rf)), pdFootZRefZ(pdFootZPtr(rf)) );
}
