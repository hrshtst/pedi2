#include <pedi2/pd_est_zmp.h>

void read_line(zVec v, int num, const char *line, const char *delim)
{
  char tmp[BUFSIZ];
  char *token;
  int i;

  strcpy(tmp, line);
  token = strtok(tmp, delim);
  i = 0;
  while(token){
    if( i == num ) break;
    if( strcmp(token, "\n") )
      zVecSetElem( v, i++, atof(token) );
    token = strtok(NULL, delim);
  }
}

#define DATA_SIZE 12
void alloc_data(FILE *fp, zVecList *data)
{
  char line[BUFSIZ];
  const char delim[] = " ";
  zVec v;

  zListInit( data );
  v = zVecAlloc( DATA_SIZE );
  while( fgets( line, sizeof(line), fp ) != NULL ){
    read_line( v, DATA_SIZE, line, delim );
    zVecListInsertHead( data, v, true );
  }
  zVecFree( v );
}

void split_vec(zVec v, zVec v1, zVec v2)
{
  register int i;
  int n, n1, n2;

  n = zVecSize( v );
  n1 = zVecSize( v1 );
  n2 = zVecSize( v2 );
  if( n != n1 + n2 ){
    ZRUNERROR( "not match size: %d != %d + %d", n, n1, n2 );
    exit( 1 );
  }
  for( i=0; i<n1; i++ )
    zVecElem( v1, i ) = zVecElem( v, i );
  for( i=0; i<n2; i++ )
    zVecElem( v2, i ) = zVecElem( v, n1 + i );
}

#define DT 0.002
int main(int argc, char *argv[])
{
  zVec lfsensor, rfsensor;
  zVecList data;
  zVecListCell *vp;
  pdEstZMP e;
  zFrame3D lf, rf;
  zVec3D v;
  FILE *fp;

  lfsensor = zVecAlloc( DATA_SIZE / 2 );
  rfsensor = zVecAlloc( DATA_SIZE / 2 );
  zVec3DCreate( &v, 0.0, 0.125, 0.0 );
  zFrame3DCreate( &lf, &v, ZMAT3DIDENT );
  zVec3DCreate( &v, 0.0, -0.125, 0.0 );
  zFrame3DCreate( &rf, &v, ZMAT3DIDENT );
  /* load data */
  fp = fopen( "data/hydra_step.dat", "r" );
  alloc_data( fp, &data );
  /* create estimator */
  pdEstZMPConfReadFile( &e, "model/hydra_sensor.conf" );
  zListForEach( &data, vp ){
    split_vec( vp->data, lfsensor, rfsensor );
    pdEstZMPSetData( &e, "lfsensor", lfsensor );
    pdEstZMPSetData( &e, "rfsensor", rfsensor );
    pdEstZMPUpdate( &e, &lf, &rf, 0, DT );
    pdEstZMPDataWrite( &e );
  }
  pdEstZMPDestroy( &e );
  zVecListDestroy( &data, true );
  zVecFree( lfsensor );
  zVecFree( rfsensor );
  fclose( fp );
  return 0;
}
