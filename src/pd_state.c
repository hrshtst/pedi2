#include <pedi2/pd_state.h>

void pdStateInit(pdState *state)
{
  zVec3DClear( &state->com_pos );
  zVec3DClear( &state->com_vel );
  zVec3DClear( &state->com_acc );
  zVec3DClear( &state->lf_pos );
  zVec3DClear( &state->rf_pos );
  zVec3DClear( &state->lh_pos );
  zVec3DClear( &state->rh_pos );
  /* zMat3DClear( &state->base_att ); */
  /* zMat3DClear( &state->lf_att ); */
  /* zMat3DClear( &state->rf_att ); */
  /* zMat3DClear( &state->lh_att ); */
  /* zMat3DClear( &state->rh_att ); */
  zVec3DClear( &state->base_att );
  zVec3DClear( &state->lf_att );
  zVec3DClear( &state->rf_att );
  zVec3DClear( &state->lh_att );
  zVec3DClear( &state->rh_att );
  zVec3DClear( &state->zmp );
  zVec3DClear( &state->deszmp );
  state->fz = 0;
  zListInit( &state->sr_lf );
  zListInit( &state->sr_rf );
  zListInit( &state->sr );
}

void pdStateDestroy(pdState *state)
{
  pdStateInit( state );
}

#define PD_STATE_FOOT_TOL  (1e-03)
bool pdStateFootIsOn(pdState *state, zVec3D *p, zVec3DList *sr)
{
  return zVec3DElem( p, zZ ) < PD_STATE_FOOT_TOL && zListNum( sr ) > 0;
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

static void _pdStateSRDataFWrite(FILE *fp, zVec3DList *sr);
void _pdStateSRDataFWrite(FILE *fp, zVec3DList *sr)
{
  zVec3DListCell *cp;

  if( !sr ){ fprintf( fp, " 0" ); return; }
  fprintf( fp, " %d", zListNum( sr ) );
  zListForEach( sr, cp ){
    zVec3DDataFWrite( fp, cp->data );
  }
}

void pdStateSRDataFWrite(FILE *fp, pdState *state)
{
  _pdStateSRDataFWrite( fp, &state->sr );
  _pdStateSRDataFWrite( fp, &state->sr_lf );
  _pdStateSRDataFWrite( fp, &state->sr_rf );
  fprintf( fp, "\n" );
}
