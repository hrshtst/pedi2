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
  state->fz = 0;
  zListInit( &state->sr_lf );
  zListInit( &state->sr_rf );
  zListInit( &state->sr );
}

void pdStateDestroy(pdState *state)
{
  pdStateInit( state );
}

double pdStateFootDist(pdState *state)
{
  return zVec3DDist( &state->lf_pos, &state->rf_pos );
}
