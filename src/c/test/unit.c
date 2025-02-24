// For these tests is required an real hardware (eg. Arduino UNO)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/wait.h>

#include "libserialposix.h" 
#include <check.h>

char pathname[ 128 ];

serial_t *
boot( ){
  serial_t * serial = serial_open( pathname, 0, NULL );
  if( !serial ){
    free( serial );
    return NULL;
  }
  
  serial_set_baudrate( B19200, serial );
  serial_set_rule( 10, 0, serial );

  serial_set_line_state( SERIAL_DTR, 0, serial );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, 1, serial );
  usleep( 2e6 );

  return serial;
}

START_TEST(test_serial_open_close_valid){
  serial_t * serial = serial_open( pathname, 0, NULL );
  ck_assert_ptr_nonnull( serial );
  ck_assert_int_eq( serial_close(serial), 0 ); 
  free( serial );
}
END_TEST

START_TEST(test_serial_readLine_valid){
  serial_t * serial = boot( );
  if( NULL == serial )
    return;

  serial_print( 1, serial ); 

  char buf[ 64 ];
  serial_write( serial, "UTEST:WRITE_LF\n" );
  const char expected_response[ ] = "UTEST:OK:WRITE_LF\n";
  
  size_t len = serial_readLine( buf, sizeof(buf), 0, serial);

  ck_assert_str_eq( buf, expected_response );  
  ck_assert_int_eq( (int) len, (int) strlen(expected_response) );
  
  serial_close( serial );
  free( serial );
}
END_TEST

Suite *
serial_suite( void ){
  Suite *s;
  TCase *tc_core;

  s = suite_create("Serial");
  tc_core = tcase_create("Core");

  tcase_add_test(tc_core, test_serial_open_close_valid);
  tcase_add_test(tc_core, test_serial_readLine_valid);

  suite_add_tcase(s, tc_core);
  return s;
}

int 
main( int argc, char ** argv ){
  if( 3 > argc){
    printf("Not enough arguments, perform the tests by doing ./unit.out -p \"<path>\"\n");
    return EXIT_FAILURE;
  }

  for( int i = 0 ; i < argc ; ++i ){
    if( !strcmp( argv[i], "-p" ) )
      strcpy( pathname, argv[i+1] );
  }
  
  Suite *s = serial_suite( );
  SRunner *runner = srunner_create( s );
  
  srunner_run_all( runner, CK_NORMAL );
  int number_failed = srunner_ntests_failed( runner );
  srunner_free( runner );

  return( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
}