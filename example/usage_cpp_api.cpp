#include "libserialposix.hpp"

using namespace libposix;

int
main( void ){
  SerialPort sr;
  if( SerialPort::StatusCode::Success != sr.connect( "/dev/ttyACM0" ) )
    return EXIT_FAILURE;

  (void) sr.setBaudRate( B19200 );
  (void) sr.setRule( 25, 0 );

  (void) sr.setLineState( SerialPort::Line::DTR, false );
  usleep( 1e3 );
  (void) sr.setLineState( SerialPort::Line::DTR, true );
  usleep( 2e6 );
  
  (void) sr.print( true );

  (void) sr.write( "UTEST:WRITE_LF\n" );
  (void) sr.drain( );

  char buf[ PATH_MAX ];

  for( ; ; ){
    if( 0 < sr.available( ).first ){
      (void) sr.readLine( buf, sizeof(buf), 0 );
      printf("%s", buf );
      sleep( 1 );
      (void) sr.write( "UTEST:WRITE_LF\n" );
      (void) sr.drain( );
    }

    // Other process
  }

  (void) sr.disconnect( );
  return EXIT_SUCCESS;
}