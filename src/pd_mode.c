#include <pedi2/pd_mode.h>

void pdModeInit(pdMode *mode)
{
  mode->standing  = true;
  mode->trymove   = false;
  mode->stepping  = false;
  mode->walking   = false;
  mode->sideways  = false;
  mode->following = false;
  mode->braking   = false;
  mode->rotating  = false;
  mode->warping   = false;
}

void pdModeDestroy(pdMode *mode)
{
  pdModeInit( mode );
}

void pdModeUpdate(pdMode *mode, pdCmd *cmd, pdState *state)
{
  if( !pdCmdTryStop( cmd ) )
    mode->trymove = true;
  else
    mode->trymove = false;

  if( pdStateBothFeetOn( state ) ){
    if( pdCmdTryStop( cmd ) || !mode->stepping ){
      mode->standing = true;
      mode->stepping = false;
    }
    if( !pdCmdTryWalk( cmd ) )
      mode->walking = false;
    if( !pdCmdTryWalkSideways( cmd ) )
      mode->sideways = false;
    mode->following = false;
    mode->braking   = false;
  }

  if( pdStateEitherFootOff( state ) ){
    mode->standing = false;
    mode->stepping = true;
    if( pdCmdTryWalk( cmd ) )
      mode->walking = true;
    if( pdCmdTryWalkSideways( cmd ) ){
      mode->sideways = true;
      mode->following = false;
      mode->braking   = false;
      if( pdStateBFOff( state, cmd->vwd ) )
        mode->following = true;
      else if( pdStateFFOff( state, cmd->vwd ) )
        mode->braking = true;
    }
  }

  if( pdCmdTryWarp( cmd ) )
    mode->warping = true;
  else
    mode->warping = false;
}

#define pdModeBool2Str(b) ( b ? "TRUE" : "FALSE" )
#define pdModeFWriteElem(fp,m,elem) fprintf( fp, #elem":%s, ", pdModeBool2Str( (m)->elem ) )
void pdModeFWrite(FILE *fp, pdMode *mode)
{
  pdModeFWriteElem( fp, mode, standing );
  pdModeFWriteElem( fp, mode, trymove );
  pdModeFWriteElem( fp, mode, stepping );
  pdModeFWriteElem( fp, mode, walking );
  pdModeFWriteElem( fp, mode, sideways );
  pdModeFWriteElem( fp, mode, following );
  pdModeFWriteElem( fp, mode, braking );
  pdModeFWriteElem( fp, mode, rotating );
  pdModeFWriteElem( fp, mode, warping );
  fprintf( fp, "\n" );
}
