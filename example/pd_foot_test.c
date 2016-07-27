#include <pedi2/pd_cz.h>
#include <pedi2/pd_foot.h>

zVec3D lfvert[4];
zVec3D rfvert[4];
zPH3D lfph, rfph;
zVec3D sr_lf_vert[4];
zVec3D sr_rf_vert[4];
zVec3D sr_vert[8];
zVec3DList sr_lf, sr_rf, sr;

void create_foot_shape(void)
{
  /* create left foot shape */
  zVec3DCreate( &lfvert[0], -0.15,  0.08, 0.0 );
  zVec3DCreate( &lfvert[1],  0.15,  0.08, 0.0 );
  zVec3DCreate( &lfvert[2],  0.15, -0.08, 0.0 );
  zVec3DCreate( &lfvert[3], -0.15, -0.08, 0.0 );
  zPH3DInit( &lfph );
  zPH3DSetVertNum( &lfph, 4 );
  zPH3DSetVertBuf( &lfph, lfvert );
  /* create right foot shape */
  zVec3DCreate( &rfvert[0], -0.15,  0.08, 0.0 );
  zVec3DCreate( &rfvert[1],  0.15,  0.08, 0.0 );
  zVec3DCreate( &rfvert[2],  0.15, -0.08, 0.0 );
  zVec3DCreate( &rfvert[3], -0.15, -0.08, 0.0 );
  zPH3DInit( &rfph );
  zPH3DSetVertNum( &rfph, 4 );
  zPH3DSetVertBuf( &rfph, rfvert );
}

void make_frame(zVec3D *p, zVec3D *a, zFrame3D *f)
{
  zFrame3DZYX( f, p->e[zX], p->e[zY], p->e[zZ],
               a->e[0], a->e[1], a->e[2] );
}

void find_supporting_region(zVec3D *lfp, zVec3D *lfa, zVec3D *rfp, zVec3D *rfa)
{
  zFrame3D frame_lf, frame_rf;
  int i, nl, nr, n;
  zVec3D v;

  nl = nr = n = 0;
  make_frame( lfp, lfa, &frame_lf );
  make_frame( rfp, rfa, &frame_rf );
  /* left foot */
  for( i=0; i<4; i++ ){
    zXfer3D( &frame_lf, zPH3DVert(&lfph,i), &v );
    if( zVec3DElem(&v,zZ) < 1e-03 ){
      zVec3DCopy( &v, &sr_lf_vert[nl++] );
      zVec3DCopy( &v, &sr_vert[n++] );
    }
  }
  /* right foot */
  for( i=0; i<4; i++ ){
    zXfer3D( &frame_rf, zPH3DVert(&rfph,i), &v );
    /* zVec3DWrite( &v ); */
    if( zVec3DElem(&v,zZ) < 1e-03 ){
      zVec3DCopy( &v, &sr_rf_vert[nr++] );
      zVec3DCopy( &v, &sr_vert[n++] );
    }
  }
  /* supporting region */
  zVec3DListDestroy( &sr_lf, false );
  zVec3DListDestroy( &sr_rf, false );
  zVec3DListDestroy( &sr, false );
  if( nl > 0 ) zCH2D( &sr_lf, sr_lf_vert, nl );
  if( nr > 0 ) zCH2D( &sr_rf, sr_rf_vert, nr );
  if( n  > 0 ) zCH2D( &sr, sr_vert, n );
}

void write_sr(zVec3DList *_sr)
{
  zVec3DListCell *cp;

  zListForEach( _sr, cp ){
    printf( "{%.10g %.10g %.10g} ", zVec3DElem(cp->data,zX),zVec3DElem(cp->data,zY),zVec3DElem(cp->data,zZ)  );
  }
  printf( "\n" );
}

void set_ctrl_parameters(pdCZ *cz)
{
  /* vertical motion controller */
  pdCZSetQ1Z( cz, 1.0 );
  pdCZSetQ2Z( cz, 0.5 );
  /* horizontal motion controller */
  pdCZSetRefVelU( cz, 0.2 );
  pdCZSetQ1U( cz, 1.0 );
  pdCZSetQ2U( cz, 0.0 );
  pdCZSetRefVelW( cz, 0.0 );
  pdCZSetQ1W( cz, 1.0 );
  pdCZSetQ2W( cz, 1.5 );
  pdCZSetRho( cz, 1.0 );
  pdCZSetKr( cz, 1.0 );
  pdCZSetDist( cz, 0.2 );
  pdCZSetKappa( cz, 0.0 );
}

#define DT 0.01
#define T 20.0
int main(void)
{
  pdCZ cz;
  zVec3D com, vel, acc, zmp, comd;
  double fz;
  double theta, thetad;
  double t;

  pdFoot lf, rf;
  zVec3D lfp, rfp;
  zVec3D lfa, rfa;

  /* init of pdCZ */
  pdCZInit( &cz, DT );
  set_ctrl_parameters( &cz );
  /* init states */
  zVec3DCreate( &com, 0, -0.001, 0.3 );
  zVec3DCopy( &com, pdCZRefCOM( &cz ) );
  zVec3DClear( &vel );
  zVec3DClear( &acc );
  zVec3DClear( &zmp );
  fz = 0;
  zVec3DCreate( &comd, 0, 0, 0.26 );
  theta = thetad = -zPI_2;

  /* init foot */
  pdFootInit( &lf, pdCZHrzPtr(&cz), PD_FOOT_LEFT, DT );
  pdFootSetMaxHeight( &lf, 0.2 );
  pdFootSetTrK( &lf, 3000, 3000, 3000 );
  pdFootSetTrC( &lf, 50, 50, 50 );
  pdFootSetTrOldVec( &lf, &lfp );
  pdFootInit( &rf, pdCZHrzPtr(&cz), PD_FOOT_RIGHT, DT );
  pdFootSetTrK( &rf, 3000, 3000, 3000 );
  pdFootSetTrC( &rf, 50, 50, 50 );
  pdFootSetTrOldVec( &rf, &rfp );
  pdFootSetMaxHeight( &rf, 0.2 );
  /* init states */
  zVec3DCreate( &lfp, 0.0, 0.1, 0.0 );
  zVec3DCreate( &rfp, 0.0, -0.1, 0.0 );
  zVec3DCreate( &lfa, theta+zPI_2, 0.0, 0.0 );
  zVec3DCreate( &rfa, theta+zPI_2, 0.0, 0.0 );
  create_foot_shape();
  find_supporting_region( &lfp, &lfa, &rfp, &rfa );

  /* loop */
  for( t=0; t<T; t+=DT ){
    /* command update */
    pdCZSetCmdCOMVec( &cz, &comd );
    pdCZSetCmdTheta( &cz, thetad );
    if( t < 0.4*T )
      pdCZSetKappa( &cz, 1.0 );
    else if( t < 0.6*T )
      pdCZSetKappa( &cz, 0.0 );
    else
      pdCZSetKappa( &cz, -1.0 );

    /* update */
    pdCZUpdate( &cz, &com, &vel, &acc, &zmp, fz, theta, &sr );
    pdFootUpdate( &lf, &rf, pdCZDelta(&cz), pdCZVelUW(&cz), &zmp, &lfp, &rfp, &lfa, &rfa, &sr_lf, &sr_rf );
    /* output */
    pdFootDataWrite( &lf, &rf );
    /* auto reference update */
    if( !zIsTiny( pdCZRefVelU(&cz) ) )
      pdCZAutoUpdateRef( &cz, &lfp, &rfp, &comd, &thetad );
    /* state udpate */
    zVec3DCopy( pdCZRefCOM(&cz), &com );
    zVec3DCopy( pdCZRefVel(&cz), &vel );
    zVec3DCopy( pdCZRefAcc(&cz), &acc );
    zVec3DCopy( pdCZRefZMP(&cz), &zmp );
    fz = pdCZRefFZ(&cz);
    theta = thetad;
    zVec3DCopy( pdFootRefPos(&lf), &lfp );
    zVec3DCopy( pdFootRefAtt(&lf), &lfa );
    zVec3DCopy( pdFootRefPos(&rf), &rfp );
    zVec3DCopy( pdFootRefAtt(&rf), &rfa );
    find_supporting_region( &lfp, &lfa, &rfp, &rfa );
  }

  return 0;
}
