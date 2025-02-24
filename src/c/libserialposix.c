/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      libserialposix.c
 * 
 * @version   2.1
 *
 * @date      12-02-2025
 *
 * @brief     Functions providing control, local settings, input, and output for serial ports on Linux, library developed for interacting with embedded systems in mind (non-cannonical).  
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 * @copyright Copyright (c) [2025] [Fábio D. Pacheco]
 * 
 * @note      Manuals:
 *            https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html \n 
 *            https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf \n
 *            https://man7.org/linux/man-pages/man3/errno.3.html
* 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include <stdio.h>    
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>    
#include <string.h>  
#include <fcntl.h>
#include <unistd.h>
#include <errno.h> 
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/limits.h>
#include "libserialposix.h"

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Function Description
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
serial_t *
serial_open( const char * pathname, uint8_t readonly, const serial_config_t * config ){
  if( !pathname ){
    errno = EINVAL;
    fprintf(stderr, "ERROR:  pathname and/or config are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( strlen(pathname) >= PATH_MAX ){
    errno = ENAMETOOLONG;
    fprintf(stderr, "ERROR:  pathname is greater (%ld) than PATH_MAX at line %d in file %s\n", strlen(pathname), __LINE__, __FILE__);
    return NULL;
  }
  
  if( NULL != config )
    readonly = config->readonly;

  serial_t * serial = (serial_t *) malloc( sizeof( serial_t ));
  if( !serial ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }

  if( readonly )
    serial->fd = open( pathname, O_RDONLY | O_NOCTTY );
  else 
    serial->fd = open( pathname, O_RDWR | O_NOCTTY );

  if( -1 == serial->fd ){
    fprintf(stderr, "ERROR:  failed to open %s: %s, at line %d in file %s\n", pathname, strerror(errno), __LINE__, __FILE__);
    free( serial );
    return NULL;
  }

  if( readonly )
    serial->fp = fdopen( serial->fd, "r" );
  else   
    serial->fp = fdopen( serial->fd, "r+" );

  if( NULL == serial->fp ){
    fprintf(stderr, "ERROR:  failed to fdopen %s: %s, at line %d in file %s\n", pathname, strerror(errno), __LINE__, __FILE__);
    if( -1 == close( serial->fd ) )
      fprintf(stderr, "ERROR:  failed to close %s: %s, at line %d in file %s\n", pathname, strerror(errno), __LINE__, __FILE__);
    free( serial );
    return NULL;
  } 

  strncpy( serial->pathname, pathname, sizeof(serial->pathname) - 1 );
  serial->config.readonly = (readonly != 0);

  if( !config ){
    serial_config_t config_default;
    if( -1 == serial_default_config( &config_default ) ){
      fprintf(stderr, "ERROR:  failed to set the default parameters, at line %d in file %s\n", __LINE__, __FILE__);
      serial_close( serial );
      free( serial );
      return NULL;
    }

    if( -1 == serial_config_update( &config_default, serial ) ){
      fprintf(stderr, "ERROR:  failed to configure the serial port, at line %d in file %s\n", __LINE__, __FILE__);
      serial_close( serial );
      free( serial );
      return NULL;
    }
    return serial;
  }

  if( -1 == serial_config_update( config, serial ) ){
    fprintf(stderr, "ERROR:  failed to configure the serial port, at line %d in file %s\n", __LINE__, __FILE__);
    serial_close( serial );
    free( serial );
    return NULL;
  }
  return serial;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_close( serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( EOF == fclose( serial->fp ) ){
    fprintf(stderr, "ERROR:  failed to fclose, %s, at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

int8_t 
serial_default_config( serial_config_t * config ){
  if( !config ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  config->baudrate = B9600;
  config->dataBits = DATA_BITS_8;
  config->flow     = FLOWCONTROL_NONE;
  config->minBytes = 0;
  config->parity   = PARITY_NONE;
  config->stopBits = STOP_BITS_1;
  config->timeout  = 100;
  config->readonly = 0;

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_baudrate( const baudRate_t baudrate, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  result = cfsetispeed( &tty, baudrate );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  result = cfsetospeed( &tty, baudrate );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  result = tcsetattr( serial->fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_parity( const parity_t parity, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  switch( parity ){
    default:
      errno = EINVAL;
      return -1;
      
    case PARITY_NONE:
      tty.c_cflag &= (tcflag_t) ~(PARENB);                                    // Disable parity (Clear bit)
      tty.c_iflag &= (tcflag_t) ~(INPCK);                                     // Disable parity checking
      break;
    
    case PARITY_ODD:
      tty.c_cflag |= (tcflag_t) (PARENB) | (PARODD);                          // Enable parity (Set bit) and Enable odd parity
      tty.c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
      break;


    case PARITY_EVEN:
      tty.c_cflag |= (tcflag_t) (PARENB);                                     // Enable parity (Set bit)
      tty.c_cflag &= (tcflag_t) ~(PARODD);                                    // Enable even parity
      tty.c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
      break;
  }

  result = tcsetattr( serial->fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_stopbits( const stopBits_t stopBits, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  switch( stopBits ){
    default:
      errno = EINVAL;
      return -1;

    case STOP_BITS_1:
      tty.c_cflag &= (tcflag_t) ~(CSTOPB);                                    // Set 1 stop bit 
      break;

    case STOP_BITS_2:      
      tty.c_cflag |= (tcflag_t) (CSTOPB);                                     // Set 2 stop bits
      break;
  }

  result = tcsetattr( serial->fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_databits( const dataBits_t dataBits, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  tty.c_cflag &= (tcflag_t) ~CSIZE;                                          // Clear all the size bits, then use one of the statements below
  switch( dataBits ){
    default:
      errno = EINVAL;
      return -1;

    case DATA_BITS_5:
    case DATA_BITS_6:
    case DATA_BITS_7:
    case DATA_BITS_8:
      tty.c_cflag |= (tcflag_t) dataBits;                                   // Bits per byte
      break;
  }

  result = tcsetattr( serial->fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_flowcontrol( const flowControl_t flowControl, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  switch( flowControl ){
    default:
      errno = EINVAL;
      return -1;

    case FLOWCONTROL_NONE:
      tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
      tty.c_cflag &= (tcflag_t) ~(CRTSCTS);
      tty.c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
      tty.c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
      break;

    case FLOWCONTROL_HARDWARE:
      tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
      tty.c_cflag |= (tcflag_t) (CRTSCTS);
      tty.c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
      tty.c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
      break;
    
    case FLOWCONTROL_SOFTWARE:
      tty.c_iflag |= (tcflag_t) (IXON | IXOFF | IXANY);
      tty.c_cflag &= (tcflag_t) ~(CRTSCTS);
      tty.c_cc[VSTART] = 1;                                                   // Enable start character (XON) - enable software flow control
      tty.c_cc[VSTOP] = 1;                                                    // Enable stop character (XOFF) - enable software flow control
      break;
  }

  result = tcsetattr( serial->fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_set_rule( const uint8_t timeout, const uint8_t min, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  tty.c_lflag &= (tcflag_t) ~(ICANON);                                        // Disable canonical mode
  tty.c_lflag &= (tcflag_t) ~(ECHO);                                          // Disable echo
  tty.c_lflag &= (tcflag_t) ~(ECHOE);                                         // Disable erasure
  tty.c_lflag &= (tcflag_t) ~(ECHONL);                                        // Disable new-line echo
  tty.c_lflag &= (tcflag_t) ~(ISIG);                                          // Disable interpretation of INTR, QUIT and SUSP
  tty.c_oflag &= (tcflag_t) ~(OPOST);                                         // Set to raw output
  tty.c_oflag &= (tcflag_t) ~(ONLCR);                                         // Disable the conversion of new line to CR/LF
  tty.c_iflag &= (tcflag_t) ~(IGNBRK);                                        // Disable ignore break condition
  tty.c_iflag &= (tcflag_t) ~(BRKINT);                                        // Disable send a SIGINT when a break condition is detected
  tty.c_iflag &= (tcflag_t) ~(INLCR);                                         // Disable map NL to CR
  tty.c_iflag &= (tcflag_t) ~(IGNCR);                                         // Disable ignore CR
  tty.c_iflag &= (tcflag_t) ~(ICRNL);                                         // Disable map CR to NL
  tty.c_cc[VEOF]     = 4;                                                     // Set EOF character to EOT (Ctrl+D, ASCII 4) - or 0 if not used
  tty.c_cc[VTIME]    = timeout;                                               // Set timeout for read() in tenths of a second
  tty.c_cc[VMIN]     = min;                                                   // Set minimum number of bytes for read() to return
  tty.c_cc[VINTR]    = 0;                                                     // Disable interrupt character (Ctrl+C)
  tty.c_cc[VQUIT]    = 0;                                                     // Disable quit character (Ctrl+\)
  tty.c_cc[VERASE]   = 0;                                                     // Disable erase character (backspace) - not relevant in raw mode
  tty.c_cc[VKILL]    = 0;                                                     // Disable kill character (Ctrl+U) - not relevant in raw mode
  tty.c_cc[VSWTC]    = 0;                                                     // Disable switch character - not usually needed
  tty.c_cc[VSUSP]    = 0;                                                     // Disable suspend character (Ctrl+Z)
  tty.c_cc[VEOL]     = 0;                                                     // Disable end-of-line character - not relevant in raw mode
  tty.c_cc[VREPRINT] = 0;                                                     // Disable reprint character - not relevant in raw mode
  tty.c_cc[VDISCARD] = 0;                                                     // Disable discard character - not relevant in raw mode
  tty.c_cc[VWERASE]  = 0;                                                     // Disable word erase character - not relevant in raw mode
  tty.c_cc[VLNEXT]   = 0;                                                     // Disable literal next character - not relevant in raw mode
  tty.c_cc[VEOL2]    = 0;                                                     // Disable alternate end-of-line character - not relevant in raw mode

  result = tcsetattr( serial->fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_config_update( const serial_config_t * config, serial_t * serial ){
  if( !config || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  
  if( -1 == serial_set_parity( config->parity, serial ) ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", errno, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  if( -1 == serial_set_stopbits( config->stopBits, serial ) ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", errno, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  if( -1 == serial_set_databits( config->dataBits, serial ) ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", errno, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
      
  if( -1 == serial_set_flowcontrol( config->flow, serial ) ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", errno, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  if( 0 != serial_set_rule( config->timeout, config->minBytes, serial ) ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", errno, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  serial->config = *config;  
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_readLine( char * buf, const size_t size, const size_t offset, const serial_t * serial ){  
  if( !buf || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial and/or buf are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( size <= offset ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: serial_fgets, %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return 0;
  }

  clearerr( serial->fp );

  if( NULL == fgets( buf + offset, (int) (size - offset), serial->fp ) ){
    if( feof( serial->fp ) ){
      clearerr( serial->fp );
      errno = ETIME;
      return 0;
    }
    
    else if( ferror( serial->fp ) ) 
      fprintf(stderr, "ERROR: fgets failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    else 
      fprintf(stderr, "ERROR: fgets returned NULL for unknown reason at line %d in file %s\n", __LINE__, __FILE__);

    clearerr( serial->fp );
    return 0;
  }

  return strlen( buf + offset );  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_read( char * buf, const size_t size, const size_t offset, const size_t length, const serial_t * serial ){
  if( !buf || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial and/or buf are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( size <= (offset + length) ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: serial_read, %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return 0;
  }

  clearerr( serial->fp );

  size_t len = fread( buf + offset, 1, length, serial->fp );

  if( length > len ){
    if( feof( serial->fp ) ){
      clearerr( serial->fp );
      errno = ETIME;
      return 0;
    }
    
    else if( ferror( serial->fp ) ) 
      fprintf(stderr, "ERROR: fgets failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    else 
      fprintf(stderr, "ERROR: fgets returned NULL for unknown reason at line %d in file %s\n", __LINE__, __FILE__);

    clearerr( serial->fp );
    return 0;
  }

  return len;  
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_write( const serial_t * serial, const char * format, ... ){
  if( !serial || !format ) {
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial and/or format are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  va_list args;
  va_start( args, format );

  char buf[ PIPE_BUF ];
  int len = vsnprintf( buf, sizeof(buf), format, args );

  if( 0 > len ) {
    fprintf(stderr, "ERROR: Formatting string failed at line %d in file %s\n", __LINE__, __FILE__);
    va_end( args );
    return 0;
  }

  if( (int) sizeof( buf ) <= len ) {
    fprintf(stderr, "ERROR: Formatted string too long for buffer at line %d in file %s\n", __LINE__, __FILE__);
    va_end( args );
    return 0;
  }

  size_t size = fwrite( buf, 1, (size_t) len, serial->fp );

  if( size < (size_t) len ){
    if( ferror( serial->fp ) )
      fprintf(stderr, "ERROR: write to serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    else
      fprintf(stderr, "WARNING: Wrote less bytes than expected to the serial port [debug:(len)%d,(size)%ld] at line %d in file %s\n", len, size, __LINE__, __FILE__);
    
    va_end( args );
    return 0;
  }

  va_end( args ); 
  return size;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
uint8_t 
serial_valid( const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( 0 > serial->fd ){
    fprintf(stderr, "WARNING: serial port (file descriptor) isn't valid\n");
    return 0;
  }

  if( -1 == fcntl( serial->fd, F_GETFD ) ){
    fprintf(stderr, "WARNING: serial port (file descriptor) isn't opened\n");
    return 0;
  }

  return 1;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_drain( const serial_t * serial ){
  if( !serial ) {
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  
  if( -1 == tcdrain( serial->fd ) ){
    fprintf(stderr, "ERROR: tcdrain the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_flush( const serial_t * serial, uint8_t option ){
  if( !serial ) {
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  
  if( -1 == tcflush( serial->fd, option ) ){
    fprintf(stderr, "ERROR: tcdflush the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
size_t 
serial_available( const serial_t * serial ){
  if( !serial ) {
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return 0;
  }
  
  int32_t len;
  if( -1 == ioctl( serial->fd, FIONREAD, &len ) ) {
    fprintf(stderr, "ERROR: ioctl(FIONREAD) the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return 0;
  }  

  if( 0 > len ){
    fprintf(stderr, "ERROR: ioctl(FIONREAD) the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return 0;
  }

  return (size_t) len;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_set_line_state( const serialLines_t line, uint8_t state, const serial_t * serial ){
  if( !serial ) {
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  uint8_t currentState;
  if( -1 == serial_get_line_state( line, &currentState, serial ) ){
    fprintf(stderr, "ERROR: serial_get_line_state at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( currentState != state ){
    int status;
    if( -1 == ioctl( serial->fd, TIOCMGET, &status ) ){
      fprintf(stderr, "ERROR: ioctl(TIOCMGET) the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
      return -1;
    }
    
    if( state ) status |= (int) line;
    else        status &= (int) ~line;

    if( -1 == ioctl( serial->fd, TIOCMSET, &status ) ){
      fprintf(stderr, "ERROR: ioctl(TIOCMSET) the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
      return -1;
    }

    if( -1 == serial_get_line_state( line, &currentState, serial ) ){
      fprintf(stderr, "ERROR: serial_get_line_state at line %d in file %s\n", __LINE__, __FILE__);
      return -1;
    }

    if( currentState != state ){
      fprintf(stderr, "ERROR: serial_get_line_state at line %d in file %s\n", __LINE__, __FILE__);
      return -1;
    }
  }
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_get_line_state( const serialLines_t line, uint8_t *state , const serial_t * serial ){
  if( !serial ) {
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  int status;
  if( -1 == ioctl( serial->fd, TIOCMGET, &status ) ){
    fprintf(stderr, "ERROR: ioctl(TIOCMGET) the serial port failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  *state = ( (uint8_t) status & line) != 0;
  return 0;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_print( uint8_t out, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }
  
  const size_t size = PATH_MAX + 1;
  char * output = (char *) malloc( size );
  if( !output ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  int len = snprintf( output, size, 
                          "Serial Port %s Configuration\n"
                          "Baud Rate: %s [bps]\n"
                          "Parity: %s\n"
                          "Data Bits: %s [b]\n"
                          "Stop Bits: %s [b]\n"
                          "Flow Control: %s\n"
                          "Timeout, Minimum Number Bytes: %s [ds, B]",
                          serial->pathname, serial_get_baudrate( NULL, serial ),
                          serial_get_parity( NULL, serial ),
                          serial_get_databits( NULL, serial ), 
                          serial_get_stopbits( NULL, serial ),
                          serial_get_flowcontrol( NULL, serial ),
                          serial_get_rule( NULL, NULL, serial )
                          );

  char * r = (char *) realloc( output, (size_t) len + 1 );
  if( !r ){
    free( output );
    fprintf(stderr, "ERROR: realloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }

  if( out ) printf( "%s\n", r );
  return r;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_get_baudrate( baudRate_t * baudrate, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  char * baudrate_string = ( char * ) malloc( BFS_GET );
  if( !baudrate_string ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }

  const baudRate_t br = cfgetospeed( &tty );
  switch( br ){
    default:        snprintf( baudrate_string, BFS_GET, "unkown"); break;
    case B0:        snprintf( baudrate_string, BFS_GET, "0"); break;  
    case B50:       snprintf( baudrate_string, BFS_GET, "50"); break;  
    case B75:       snprintf( baudrate_string, BFS_GET, "75"); break;  
    case B110:      snprintf( baudrate_string, BFS_GET, "110"); break;  
    case B134:      snprintf( baudrate_string, BFS_GET, "134"); break;  
    case B150:      snprintf( baudrate_string, BFS_GET, "150"); break;  
    case B200:      snprintf( baudrate_string, BFS_GET, "200"); break;  
    case B300:      snprintf( baudrate_string, BFS_GET, "300"); break;  
    case B600:      snprintf( baudrate_string, BFS_GET, "600"); break;  
    case B1200:     snprintf( baudrate_string, BFS_GET, "1200"); break;  
    case B1800:     snprintf( baudrate_string, BFS_GET, "1800"); break;  
    case B2400:     snprintf( baudrate_string, BFS_GET, "2400"); break;  
    case B4800:     snprintf( baudrate_string, BFS_GET, "4800"); break;  
    case B9600:     snprintf( baudrate_string, BFS_GET, "9600"); break;  
    case B19200:    snprintf( baudrate_string, BFS_GET, "19200"); break;  
    case B38400:    snprintf( baudrate_string, BFS_GET, "38400"); break;  
    case B57600:    snprintf( baudrate_string, BFS_GET, "57600"); break;  
    case B115200:   snprintf( baudrate_string, BFS_GET, "115200"); break;  
    case B230400:   snprintf( baudrate_string, BFS_GET, "230400"); break;  
    case B460800:   snprintf( baudrate_string, BFS_GET, "460800"); break;  
    case B500000:   snprintf( baudrate_string, BFS_GET, "500000"); break;  
    case B576000:   snprintf( baudrate_string, BFS_GET, "576000"); break;  
    case B921600:   snprintf( baudrate_string, BFS_GET, "921600"); break;  
    case B1000000:  snprintf( baudrate_string, BFS_GET, "1000000"); break;  
    case B1152000:  snprintf( baudrate_string, BFS_GET, "1152000"); break;  
    case B1500000:  snprintf( baudrate_string, BFS_GET, "1500000"); break;  
    case B2000000:  snprintf( baudrate_string, BFS_GET, "2000000"); break;  
  }

  if( NULL != baudrate )
    *baudrate = br; 
  return baudrate_string;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_get_parity( parity_t * parity, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  char * parity_string = ( char * ) malloc( BFS_GET );
  if( !parity_string ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }

  if( !( tty.c_iflag & (tcflag_t) INPCK ) ){
    if( NULL != parity )
      *parity = PARITY_NONE;
    snprintf( parity_string, BFS_GET, "none" );
  } 
  else if( !( tty.c_cflag & (tcflag_t) PARODD ) ){
    if( NULL != parity )
      *parity = PARITY_EVEN;
    snprintf( parity_string, BFS_GET, "even" );
  } 
  else{
    if( NULL != parity )
      *parity = PARITY_ODD;    
    snprintf( parity_string, BFS_GET, "odd" );
  }  

  return parity_string;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_get_stopbits( stopBits_t * stopBits, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  char * stopbits_string = ( char * ) malloc( BFS_GET );
  if( !stopbits_string ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }

  if( !( tty.c_iflag & (tcflag_t) CSTOPB ) ){
    if( NULL != stopBits )
      *stopBits = STOP_BITS_1;
    snprintf( stopbits_string, BFS_GET, "1" );
    return stopbits_string;
  }

  if( NULL != stopBits )
    *stopBits = STOP_BITS_2;
  snprintf( stopbits_string, BFS_GET, "2" );
  return stopbits_string;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_get_databits( dataBits_t * dataBits, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  char * databits_string = ( char * ) malloc( BFS_GET );
  if( !databits_string ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  tcflag_t _databits = tty.c_cflag & (tcflag_t) CSIZE;
  switch( _databits ){
    default:          snprintf( databits_string, BFS_GET, "0");  break;
    case DATA_BITS_5: snprintf( databits_string, BFS_GET, "5");  break;
    case DATA_BITS_6: snprintf( databits_string, BFS_GET, "6");  break;
    case DATA_BITS_7: snprintf( databits_string, BFS_GET, "7");  break;
    case DATA_BITS_8: snprintf( databits_string, BFS_GET, "8");  break;
  }
  if( NULL != dataBits ) 
    *dataBits = _databits;
  return databits_string;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_get_flowcontrol( flowControl_t * flowControl, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  char * flowcontrol_string = ( char * ) malloc( BFS_GET );
  if( !flowcontrol_string ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  }

  if( !(tty.c_cflag & (tcflag_t) CRTSCTS) ){
    if( !(tty.c_iflag & (tcflag_t) (IXON | IXOFF | IXANY) ) ){
      if( NULL != flowControl )
        *flowControl = FLOWCONTROL_NONE;
      snprintf( flowcontrol_string, BFS_GET, "none" );
    }
    else{
      if( NULL != flowControl )
        *flowControl = FLOWCONTROL_SOFTWARE;
      snprintf( flowcontrol_string, BFS_GET, "software" );
    }
  }
  else{
    if( NULL != flowControl )
      *flowControl = FLOWCONTROL_HARDWARE;
    snprintf( flowcontrol_string, BFS_GET, "hardware" );
  }

  return flowcontrol_string;
}

/**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
char * 
serial_get_rule( uint8_t * timeout, uint8_t * min, const serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  if( !serial_valid( serial ) ){
    errno = EBADF;
    fprintf(stderr, "ERROR: serial port ins't opened at line %d in file %s\n", __LINE__, __FILE__);
    return NULL;
  }

  struct termios tty;
  int result = tcgetattr( serial->fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return NULL;
  }
  
  char * string = ( char * ) malloc( BFS_GET );
  if( !string ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: malloc (%s) at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return NULL;
  } 
  if( NULL != timeout )
    *timeout = tty.c_cc[VTIME];
  if( NULL != min )
    *min = tty.c_cc[VTIME];

  snprintf( string, BFS_GET, "%hhd,%hhd", tty.c_cc[VTIME], tty.c_cc[VMIN] );
  return string;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/