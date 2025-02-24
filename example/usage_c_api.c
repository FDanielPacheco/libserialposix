#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "libserialposix.h"

int
main( void ){
  serial_t *sr = serial_open( "/dev/ttyACM0", 0, NULL ) ;
  if( !sr )
    return EXIT_FAILURE;

  serial_set_baudrate( B19200, sr );
  serial_set_rule( 25, 0, sr );

  serial_set_line_state( SERIAL_DTR, 0, sr );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, 1, sr );
  usleep( 2e6 );
  
  serial_write( sr, "UTEST:WRITE_LF\n" );
  serial_drain( sr );

  char buf[ PATH_MAX ];

  for( ; ; ){
    if( serial_available( sr ) ){
      serial_readLine( buf, sizeof(buf), 0, sr );
      printf("%s", buf );
    }

    // Other process
  }

  serial_close( sr );
  free( sr );
  return 0;
}