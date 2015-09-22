#include <pedi2/pd_filter.h>

#define DT 0.01
#define CF 0.5
#define DIM 2
#define STEP 10000

int main(int argc, char *argv[])
{
  register int i;
  pdFilter bwf;
  double val;

  pdFilterCreateBW( &bwf, CF, DIM );
  zRandInit();

  /* signal processing */
  for( i=0; i<=STEP; i++ ){
    /* noised sine wave */
    val = sin( 0.5 * DT * i ) + zRandF(-1.0,1.0) + 1.0;
    printf( "%f ", val );
    /* signal filtering */
    pdFilterInput( &bwf ) = val;
    pdFilterUpdate( &bwf, DT );
    printf( "%f ", pdFilterOutput(&bwf) );
    printf( "\n" );
  }
  pdFilterDestroy( &bwf );
  return 0;
}
