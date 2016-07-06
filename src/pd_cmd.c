#include <pedi2/pd_cmd.h>

void pdCmdInit(pdCmd *cmd)
{
  register int i;

  if( (sizeof(pdCmd)/sizeof(double)) != PD_CMD_ENTRY_NUM )
    ZRUNWARN( "size of pdCmd is not matched with num of entries" );
  for( i=0; i<PD_CMD_ENTRY_NUM; i++ )
    cmd->entry[i] = 0;
}

void pdCmdDefaultInit(pdCmd *cmd)
{
  pdCmdInit( cmd );
  cmd->qu1 = 1.0;
  cmd->qu2 = 0.0;
  cmd->qw1 = 1.0;
  cmd->qw2 = 1.5;
  cmd->qz1 = 1.0;
  cmd->qz2 = 0.5;
  cmd->rho = 0.0;
  cmd->kr = 1.0;
  cmd->thetad = -zPI_2;
  cmd->lfkx = cmd->lfky = cmd->lfkz = 3000;
  cmd->lfcx = cmd->lfcy = cmd->lfcz = 50;
  cmd->rfkx = cmd->rfky = cmd->rfkz = 3000;
  cmd->rfcx = cmd->rfcy = cmd->rfcz = 50;
}

void pdCmdDestroy(pdCmd *cmd)
{
  pdCmdInit( cmd );
}

bool pdCmdTryStop(pdCmd *cmd)
{
  return ( zIsTiny( cmd->rho ) &&
           zIsTiny( cmd->vud ) &&
           zIsTiny( cmd->vwd ) );
}

bool pdCmdTryStep(pdCmd *cmd)
{
  return !zIsTiny( cmd->rho );
}

bool pdCmdTryWalk(pdCmd *cmd)
{
  return !zIsTiny( cmd->vud );
}

bool pdCmdTryWalkSideways(pdCmd *cmd)
{
  return !zIsTiny( cmd->vwd );
}

void pdCmdDataFWrite(FILE *fp, pdCmd *cmd)
{
  register int i;

  for( i=0; i<PD_CMD_ENTRY_NUM; i++ )
    fprintf( fp, "%g ", cmd->entry[i] );
  fprintf( fp, "\n" );
}

void _pdCmdDataFReadLine(pdCmd *cmd, const char *line, const char *delim)
{
  char tmp[BUFSIZ];
  char *token;
  register int i;

  strcpy(tmp, line);
  token = strtok(tmp, delim);
  for( i=0; token && i<PD_CMD_ENTRY_NUM; i++){
    cmd->entry[i] = atof( token );
    token = strtok( NULL, delim );
  }
}

bool pdCmdDataFRead(FILE *fp, pdCmd *cmd)
{
  char line[BUFSIZ];

  if( fgets( line, sizeof(line), fp ) ){
    _pdCmdDataFReadLine( cmd, line, " " );
    return true;
  } else
    return false;
}
