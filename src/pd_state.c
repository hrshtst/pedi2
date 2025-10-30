#include <pedi2/pd_state.h>

void pdStateInit(pdState *state)
{
  zVec3DZero( &state->com_pos );
  zVec3DZero( &state->com_vel );
  zVec3DZero( &state->com_acc );
  zVec3DZero( &state->lf_pos );
  zVec3DZero( &state->rf_pos );
  zVec3DZero( &state->lh_pos );
  zVec3DZero( &state->rh_pos );
  /* zMat3DZero( &state->torso_att ); */
  /* zMat3DZero( &state->lf_att ); */
  /* zMat3DZero( &state->rf_att ); */
  /* zMat3DZero( &state->lh_att ); */
  /* zMat3DZero( &state->rh_att ); */
  zVec3DZero( &state->torso_att );
  zVec3DZero( &state->lf_att );
  zVec3DZero( &state->rf_att );
  zVec3DZero( &state->lh_att );
  zVec3DZero( &state->rh_att );
  zVec3DZero( &state->zmp );
  zVec3DZero( &state->deszmp );
  state->fz = 0;
  zVec3DZero( &state->ef );
  zListInit( &state->sr_lf );
  zListInit( &state->sr_rf );
  zListInit( &state->sr );
}

void pdStateDestroy(pdState *state)
{
  pdStateInit( state );
}

#define PD_STATE_FOOT_TOL  (1e-03)
bool pdStateFootIsOn(pdState *state, zVec3D *p, zLoop3D *sr)
{
  return p->c.z < PD_STATE_FOOT_TOL && zListSize( sr ) > 0;
}

bool pdStateFFOn(pdState *state, double vwd)
{
  if( zIsTiny( vwd ) ){
    ZRUNWARN( "The role of foot (FF or BF) cannot be determined" );
    return false;
  } else if( vwd > 0 )
    return pdStateFootIsOn( state, &state->rf_pos, &state->sr_rf );
  else
    return pdStateFootIsOn( state, &state->lf_pos, &state->sr_lf );
}

bool pdStateBFOn(pdState *state, double vwd)
{
  if( zIsTiny( vwd ) ){
    ZRUNWARN( "The role of foot (FF or BF) cannot be determined" );
    return false;
  } else if( vwd > 0 )
    return pdStateFootIsOn( state, &state->lf_pos, &state->sr_lf );
  else
    return pdStateFootIsOn( state, &state->rf_pos, &state->sr_rf );
}

double pdStateFootDist(pdState *state)
{
  return zVec3DDist( &state->lf_pos, &state->rf_pos );
}

static void _pdStateSRDataFWrite(FILE *fp, zLoop3D *sr);
void _pdStateSRDataFWrite(FILE *fp, zLoop3D *sr)
{
  zLoop3DCell *cp;

  if( !sr ){ fprintf( fp, " 0" ); return; }
  fprintf( fp, " %d", zListSize( sr ) );
  zListForEach( sr, cp ){
    fprintf( fp, " %.10g %.10g %.10g", cp->data->c.x, cp->data->c.y, cp->data->c.z );
  }
}

void pdStateSRDataFWrite(FILE *fp, pdState *state)
{
  _pdStateSRDataFWrite( fp, &state->sr );
  _pdStateSRDataFWrite( fp, &state->sr_lf );
  _pdStateSRDataFWrite( fp, &state->sr_rf );
  fprintf( fp, "\n" );
}
