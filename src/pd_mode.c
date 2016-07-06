#include <pedi2/pd_mode.h>

void pdModeInit(pdMode *mode)
{
  mode->standing  = true;
  mode->trywalk   = false;
  mode->stepping  = false;
  mode->walking   = false;
  mode->sideways  = false;
  mode->following = false;
  mode->braking   = false;
  mode->rotating  = false;
}

void pdModeDestroy(pdMode *mode)
{
  pdModeInit( mode );
}

#define pdModeBool2Str(b) ( b ? "TRUE" : "FALSE" )
#define pdModeFWriteElem(fp,m,elem) fprintf( fp, #elem":%s, ", pdModeBool2Str( (m)->elem ) )
void pdModeFWrite(FILE *fp, pdMode *mode)
{
  pdModeFWriteElem( fp, mode, standing );
  pdModeFWriteElem( fp, mode, trywalk );
  pdModeFWriteElem( fp, mode, stepping );
  pdModeFWriteElem( fp, mode, walking );
  pdModeFWriteElem( fp, mode, sideways );
  pdModeFWriteElem( fp, mode, following );
  pdModeFWriteElem( fp, mode, braking );
  pdModeFWriteElem( fp, mode, rotating );
  fprintf( fp, "\n" );
}
