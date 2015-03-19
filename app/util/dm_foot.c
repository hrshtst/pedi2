#include "dm_foot.h"

void dmCtrlZMPPhase(pdCtrl *c, double vw, zComplex *pz)
{
  zComplexCreate( pz, pdCtrlZMPRad(c), -(pdCtrlPrmRad(c)->q1*pdCtrlPrmRad(c)->q2+1)*vw/(pdCtrlZeta(c)*sqrt(pdCtrlPrmRad(c)->q1*pdCtrlPrmRad(c)->q2)) );
}

#define DM_FOOT_TOL (1.0e-3)
void dmFootPhase(dmFoot *pf, dmFoot *kf, double xd, double yd, double theta, zComplex *pz)
{
  double dr, d, da, r2, r;
  double s, c;
  zComplex p, plim;

  zSinCos( theta, &s, &c );
  r2 = zComplexSqrAbs(pz);
  r  = sqrt(r2);
  /* for periodic stepping (I don't know why...) */
  dr = -(zVec3DElem(&pf->p,zX)-xd)*c-(zVec3DElem(&pf->p,zY)-yd)*s - pf->dy*0.035;
  da = acos( dr / r );
  if( ( d = r2 - zSqr(dr) ) > 0 ){
    zComplexCreate( &plim, pf->dy*dr, -pf->dy*sqrt(d) );
    zComplexCDiv( pz, &plim, &p );
    kf->phase = zLimit( 0.5*zComplexArg(&p)/da, 0, 1 );
  } else
    kf->phase = 0;
}

#define LMAX 0.27
void dmFootLift(pdCtrl *ctrl, dmFoot *lf, dmFoot *rf, double xd, double yd, double theta, zComplex *pz)
{
  double dh;
  double l;
  double s, c;

  zSinCos( theta, &s, &c );
  /* foot height degree */
  dmFootPhase( rf, lf, xd, yd, theta, pz );
  dmFootPhase( lf, rf, xd, yd, theta, pz );
  if( lf->phase == 0 ){
    l = sqrt( zSqr(zVec3DElem(&lf->p,zX)-xd) + zSqr(zVec3DElem(&lf->p,zY)-yd) + zSqr(ctrl->vrt.z) );
  } else{
    l = sqrt( zSqr(zVec3DElem(&rf->p,zX)-xd) + zSqr(zVec3DElem(&rf->p,zY)-yd) + zSqr(ctrl->vrt.z) );
  }
  /* dh = zComplexAbs(pz) / c->r * zLimit( (c->rho_bar*zE-1)/(zE-1), 0, 1 ) * zLimit( (LMAX-l)/(LMAX-c->vert->z), 0, 1 ); */
  dh = 2.0*zComplexAbs(pz) / pdCtrlPrmRad(ctrl)->dist * zLimit( (pdCtrlPrmRad(ctrl)->rho*zE-1)/(zE-1), 0, 1 );
  zVec3DElem(&lf->pd,zZ) = zCycloidY( 0, lf->h * dh, lf->phase );
  zVec3DElem(&rf->pd,zZ) = zCycloidY( 0, rf->h * dh, rf->phase );
}

bool dmFootIsCurrentFootFloating(dmFoot *f)
{
  /* return (f->phase > 0 && zVec3DElem(&f->p,zZ) > DM_FOOT_TOL); */
  return  zVec3DElem(&f->p,zZ) > DM_FOOT_TOL;
}

bool dmFootIsDesiredFootFloating(dmFoot *f)
{
  /* return (f->phase > 0 && zVec3DElem(&f->pd,zZ) > DM_FOOT_TOL); */
  return zVec3DElem(&f->pd,zZ) > DM_FOOT_TOL;
}

bool dmFootIsDesiredFootForSeedFloating(dmFoot *f)
{
  /* return (f->phase > 0 && zVec3DElem(&f->pds,zZ) > DM_FOOT_TOL); */
  return  zVec3DElem(&f->ps,zZ) > DM_FOOT_TOL;
}

void dmFootStep(pdCtrl *ctrl, dmFoot *pf, dmFoot *kf, double xd, double yd, double theta)
{
  double d;
  double s, c;

  d = 0.5 * pdCtrlPrmRad(ctrl)->dist;
  zSinCos( theta, &s, &c );

  zVec3DElem(&kf->pd,zX) = xd - kf->dy * d * c;
  zVec3DElem(&kf->pd,zY) = yd - kf->dy * d * s;
  zVec3DCreate( &kf->as, theta+zPI_2, 0, 0 );
}

void dmFootMove(pdCtrl *ctrl, dmFoot *lf, dmFoot *rf, double xd, double yd, double theta)
{
  if( dmFootIsCurrentFootFloating( lf ) || ( lf->phase > 0 && zVec3DElem(&lf->pd,zZ) > DM_FOOT_TOL ) ){
    dmFootStep( ctrl, rf, lf, xd, yd, theta );
  } else {
    lf->xk0 = zVec3DElem( &lf->p, zX );
    lf->yk0 = zVec3DElem( &lf->p, zY );
  }
  if( dmFootIsCurrentFootFloating( rf ) || ( rf->phase > 0 && zVec3DElem(&rf->pd,zZ) > DM_FOOT_TOL ) ){
    dmFootStep( ctrl, lf, rf, xd, yd, theta );
  } else {
    rf->xk0 = zVec3DElem( &rf->p, zX );
    rf->yk0 = zVec3DElem( &rf->p, zY );
  }
}

#define DT 0.01
void dmFootUpdatePos(dmFoot *lf, dmFoot *rf)
{
  if( dmFootIsCurrentFootFloating( lf ) || ( lf->phase > 0 && zVec3DElem(&lf->pd,zZ) > DM_FOOT_TOL ) ){
    zVec3DElem(&lf->ps,zX) =
        ( lf->track_kx*DT*DT* zVec3DElem(&lf->pd,zX) + ( lf->track_cx*DT+2 )* zVec3DElem(&lf->p,zX) - lf->track_xold ) / ( 1 + lf->track_cx*DT + lf->track_kx*DT*DT );
    lf->track_xold = zVec3DElem(&lf->p,zX);
    zVec3DElem(&lf->ps,zY) =
        ( lf->track_ky*DT*DT* zVec3DElem(&lf->pd,zY) + ( lf->track_cy*DT+2 )* zVec3DElem(&lf->p,zY) - lf->track_yold ) / ( 1 + lf->track_cy*DT + lf->track_ky*DT*DT );
    lf->track_yold = zVec3DElem(&lf->p,zY);
    zVec3DElem(&lf->ps,zZ) =
        ( lf->track_kz*DT*DT* zVec3DElem(&lf->pd,zZ) + ( lf->track_cz*DT+2 )* zVec3DElem(&lf->p,zZ) - lf->track_zold ) / ( 1 + lf->track_cz*DT + lf->track_kz*DT*DT );
    lf->track_zold = zVec3DElem(&lf->p,zZ);
  }
  if( dmFootIsCurrentFootFloating( rf ) || ( rf->phase > 0 && zVec3DElem(&rf->pd,zZ) > DM_FOOT_TOL ) ){
    zVec3DElem(&rf->ps,zX) =
        ( rf->track_kx*DT*DT* zVec3DElem(&rf->pd,zX) + ( rf->track_cx*DT+2 )* zVec3DElem(&rf->p,zX) - rf->track_xold ) / ( 1 + rf->track_cx*DT + rf->track_kx*DT*DT );
    rf->track_xold = zVec3DElem(&rf->p,zX);
    zVec3DElem(&rf->ps,zY) =
        ( rf->track_ky*DT*DT* zVec3DElem(&rf->pd,zY) + ( rf->track_cy*DT+2 )* zVec3DElem(&rf->p,zY) - rf->track_yold ) / ( 1 + rf->track_cy*DT + rf->track_ky*DT*DT );
    rf->track_yold = zVec3DElem(&rf->p,zY);
    zVec3DElem(&rf->ps,zZ) =
        ( rf->track_kz*DT*DT* zVec3DElem(&rf->pd,zZ) + ( rf->track_cz*DT+2 )* zVec3DElem(&rf->p,zZ) - rf->track_zold ) / ( 1 + rf->track_cz*DT + rf->track_kz*DT*DT );
    rf->track_zold = zVec3DElem(&rf->p,zZ);
  }
}
