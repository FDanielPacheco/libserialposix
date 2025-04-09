/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      libserialposix.cpp
 * 
 * @version   1.0
 *
 * @date      03-03-2025
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

#include <stdarg.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/limits.h>
#include <cstdlib>    
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cerrno>
#include <memory>
#include <vector>
#include <stdexcept>
#include <optional>
#include <iostream>
#include <sstream>
#include <fstream>

#include "libserialposix.hpp"

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Local Macros
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#define errorPrint( txt, ... ) fprintf( stderr, "Error: " txt ",at line %d in file %s\nErrno: %d, %s\n", ##__VA_ARGS__, __LINE__, __FILE__, errno, strerror(errno) )

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Function Description
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::
SerialPort( void ) : 
  fd( -1 ), 
  fp( nullptr, &fclose ), 
  pathname( "" ), 
  config( )
  { };

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::connect( const std::string &_pathname, std::optional <SerialPort::Configuration> _config, std::optional <uint8_t> _rdonly ){
  if( _pathname.empty( ) ){
    errorPrint( "pathname empty" );
    return StatusCode::InvalidParameter;
  }

  if( _pathname.size( ) >= PATH_MAX ){
    errorPrint( "pathname to long" );
    return StatusCode::NameToLong;
  }

  Configuration __config = _config.value_or( Configuration( ) );  
  __config.readonly = _rdonly.value_or( __config.readonly );

  if( __config.readonly )
    this->fd = open( _pathname.c_str( ) , O_RDONLY | O_NOCTTY );
  else 
    this->fd = open( _pathname.c_str( ) , O_RDWR | O_NOCTTY );

  if( -1 == this->fd ){
    errorPrint( "open(%s)", _pathname.c_str( ) );
    return StatusCode::DeviceNotFound;
  }

  if( __config.readonly )
    this->fp = std::unique_ptr<FILE, decltype(&fclose)> (fdopen( this->fd, "r" ), &fclose) ;
  else 
    this->fp = std::unique_ptr<FILE, decltype(&fclose)> (fdopen( this->fd, "r+" ), &fclose) ;

  if( !this->fp ){
    errorPrint( "fdopen" );
    if( -1 == close( this->fd ) )
      errorPrint( "close" );
    return StatusCode::Error;
  }

  this->pathname = _pathname;

  if( StatusCode::Success != this->update( __config ) ){
    errorPrint( "update" );
    return StatusCode::Error;
  }

  return StatusCode::Success;
};

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
bool
libposix::SerialPort::getTermios( void ){
  int result = tcgetattr( this->fd, &(this->tty) );
  if( 0 != result ){
    errorPrint( "tcgetatrr" );
    return false;
  }
  return true;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
bool
libposix::SerialPort::applyTermios( void ){
  int result = tcsetattr( this->fd, TCSANOW, &(this->tty) );
  if( 0 != result ){
    errorPrint( "tcsetattr" );
    return false;
  }
  return true;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::update( const Configuration &_config ){
  if( !this->isValid( ) )
    return StatusCode::Error;

  if( StatusCode::Error == this->setBaudRate( _config.baudrate ) ){
    errorPrint( "setBaudRate" );
    return StatusCode::Error;
  }

  if( StatusCode::Error == this->setParity( _config.parity ) ){
    errorPrint( "setParity" );
    return StatusCode::Error;
  }

  if( StatusCode::Error == this->setDataBits( _config.bData ) ){
    errorPrint( "setDataBits" );
    return StatusCode::Error;
  }

  if( StatusCode::Error == this->setStopBits( _config.bStop ) ){
    errorPrint( "setStopBits" );
    return StatusCode::Error;
  }

  if( StatusCode::Error == this->setFlowControl( _config.flow ) ){
    errorPrint( "setFlowControl" );
    return StatusCode::Error;
  }

  if( StatusCode::Error == this->setRule( _config.timeout, _config.minBytes ) ){
    errorPrint( "setRule" );
    return StatusCode::Error;
  }

  this->config = _config;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
bool
libposix::SerialPort::isValid( void ){
  if( 0 > this->fd )
    return false;

  if( -1 == fcntl( this->fd, F_GETFD ) )
    return false;

  return true;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
void
libposix::SerialPort::setReadOnly( const bool _state ){
  this->config.readonly = _state;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::setBaudRate( const BaudRate _baudrate ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return StatusCode::Error;

  int result = cfsetispeed( &(this->tty), _baudrate );
  if( 0 != result ){
    errorPrint( "cfsetispeed" );
    return StatusCode::Error;
  }

  result = cfsetospeed( &(this->tty), _baudrate );
  if( 0 != result ){
    errorPrint( "cfsetospeed" );
    return StatusCode::Error;
  }

  if( !this->applyTermios( ) )
    return StatusCode::Error;
  
  this->config.baudrate = _baudrate;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::setParity( const Parity _parity ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return StatusCode::Error;
  
  switch( _parity ){
    default:
      return StatusCode::InvalidParameter;
      
    case Parity::None:
      tty.c_cflag &= (tcflag_t) ~(PARENB);                                    // Disable parity (Clear bit)
      tty.c_iflag &= (tcflag_t) ~(INPCK);                                     // Disable parity checking
      break;
    
    case Parity::Odd:
      tty.c_cflag |= (tcflag_t) (PARENB) | (PARODD);                          // Enable parity (Set bit) and Enable odd parity
      tty.c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
      break;


    case Parity::Even:
      tty.c_cflag |= (tcflag_t) (PARENB);                                     // Enable parity (Set bit)
      tty.c_cflag &= (tcflag_t) ~(PARODD);                                    // Enable even parity
      tty.c_iflag |= (tcflag_t) (INPCK);                                      // Enable parity checking
      break;
  }

  if( !this->applyTermios( ) )
    return StatusCode::Error;

  this->config.parity = _parity;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::setFlowControl( const FlowControl _flow ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return StatusCode::Error;

  switch( _flow ){
    default:
      return StatusCode::InvalidParameter;

    case FlowControl::None:
      tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
      tty.c_cflag &= (tcflag_t) ~(CRTSCTS);
      tty.c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
      tty.c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
      break;

    case FlowControl::Hardware:
      tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
      tty.c_cflag |= (tcflag_t) (CRTSCTS);
      tty.c_cc[VSTART] = 0;                                                   // Disable start character (XON) - disable software flow control
      tty.c_cc[VSTOP] = 0;                                                    // Disable stop character (XOFF) - disable software flow control
      break;
    
    case FlowControl::Software:
      tty.c_iflag |= (tcflag_t) (IXON | IXOFF | IXANY);
      tty.c_cflag &= (tcflag_t) ~(CRTSCTS);
      tty.c_cc[VSTART] = 1;                                                   // Enable start character (XON) - enable software flow control
      tty.c_cc[VSTOP] = 1;                                                    // Enable stop character (XOFF) - enable software flow control
      break;
  }

  if( !this->applyTermios( ) )
    return StatusCode::Error;

  this->config.flow = _flow;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::setDataBits( const DataBits _databits ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return StatusCode::Error;

  switch( _databits ){
    default:
      return StatusCode::InvalidParameter;

    case DataBits::Five:
    case DataBits::Six:
    case DataBits::Seven:
    case DataBits::Eight:
      tty.c_cflag |= (tcflag_t) _databits;                                  // Bits per word
      break;
  }

  if( !this->applyTermios( ) )
    return StatusCode::Error;

  this->config.bData = _databits;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::setStopBits( const StopBits _stopbits ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return StatusCode::Error;

  switch( _stopbits ){
    default:
      return StatusCode::InvalidParameter;

    case StopBits::One:
      tty.c_cflag &= (tcflag_t) ~(CSTOPB);                                    // Set 1 stop bit 
      break;

    case StopBits::Two:      
      tty.c_cflag |= (tcflag_t) (CSTOPB);                                     // Set 2 stop bits
      break;
  }

  if( !this->applyTermios( ) )
    return StatusCode::Error;

  this->config.bStop = _stopbits;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::setRule( const uint8_t _timeout, const uint8_t _minbytes ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return StatusCode::Error;

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
  tty.c_cc[VTIME]    = _timeout;                                              // Set timeout for read() in tenths of a second
  tty.c_cc[VMIN]     = _minbytes;                                             // Set minimum number of bytes for read() to return
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

  if( !this->applyTermios( ) )
    return StatusCode::Error;

  this->config.timeout = _timeout;
  this->config.minBytes = _minbytes;
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, bool>
libposix::SerialPort::isReadOnly( void ){
  if( this->config.readonly )
    return std::pair <std::string, bool> ( "yes", true );
  return std::pair <std::string, bool> ( "no", false );  
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, libposix::SerialPort::BaudRate>
libposix::SerialPort::getBaudRate( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, BaudRate> ( "unknown", B0 );

  const BaudRate _baudrate = cfgetospeed( &tty );
  switch( _baudrate ){
    default:        
    case B0:       return std::pair <std::string, BaudRate> ( "0", _baudrate ); 
    case B50:      return std::pair <std::string, BaudRate> ( "50", _baudrate );  
    case B75:      return std::pair <std::string, BaudRate> ( "75", _baudrate );  
    case B110:     return std::pair <std::string, BaudRate> ( "110", _baudrate );   
    case B134:     return std::pair <std::string, BaudRate> ( "134", _baudrate );   
    case B150:     return std::pair <std::string, BaudRate> ( "150", _baudrate );   
    case B200:     return std::pair <std::string, BaudRate> ( "200", _baudrate );   
    case B300:     return std::pair <std::string, BaudRate> ( "300", _baudrate );   
    case B600:     return std::pair <std::string, BaudRate> ( "600", _baudrate );   
    case B1200:    return std::pair <std::string, BaudRate> ( "1200", _baudrate ); 
    case B1800:    return std::pair <std::string, BaudRate> ( "1800", _baudrate ); 
    case B2400:    return std::pair <std::string, BaudRate> ( "2400", _baudrate ); 
    case B4800:    return std::pair <std::string, BaudRate> ( "4800", _baudrate ); 
    case B9600:    return std::pair <std::string, BaudRate> ( "9600", _baudrate ); 
    case B19200:   return std::pair <std::string, BaudRate> ( "19200", _baudrate );  
    case B38400:   return std::pair <std::string, BaudRate> ( "38400", _baudrate );  
    case B57600:   return std::pair <std::string, BaudRate> ( "57600", _baudrate );  
    case B115200:  return std::pair <std::string, BaudRate> ( "115200", _baudrate );   
    case B230400:  return std::pair <std::string, BaudRate> ( "230400", _baudrate );   
    case B460800:  return std::pair <std::string, BaudRate> ( "460800", _baudrate );   
    case B500000:  return std::pair <std::string, BaudRate> ( "500000", _baudrate );   
    case B576000:  return std::pair <std::string, BaudRate> ( "576000", _baudrate );   
    case B921600:  return std::pair <std::string, BaudRate> ( "921600", _baudrate );   
    case B1000000: return std::pair <std::string, BaudRate> ( "1000000", _baudrate );   
    case B1152000: return std::pair <std::string, BaudRate> ( "1152000", _baudrate );   
    case B1500000: return std::pair <std::string, BaudRate> ( "1500000", _baudrate );   
    case B2000000: return std::pair <std::string, BaudRate> ( "2000000", _baudrate );   
  }
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, libposix::SerialPort::FlowControl>
libposix::SerialPort::getFlowControl( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, FlowControl> ( "unknown", FlowControl::Error );

  if( !(tty.c_cflag & (tcflag_t) CRTSCTS) ){
    if( !(tty.c_iflag & (tcflag_t) (IXON | IXOFF | IXANY) ) )
      return std::pair <std::string, FlowControl> ( "none", FlowControl::None );
    else
      return std::pair <std::string, FlowControl> ( "software", FlowControl::Software );
  }
  return std::pair <std::string, FlowControl> ( "hardware", FlowControl::Hardware );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, libposix::SerialPort::Parity>
libposix::SerialPort::getParity( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, Parity> ( "unknown", Parity::Error );

  if( !( tty.c_iflag & (tcflag_t) INPCK ) )
    return std::pair <std::string, Parity> ( "none", Parity::None );

  else if( !( tty.c_cflag & (tcflag_t) PARODD ) )
    return std::pair <std::string, Parity> ( "even", Parity::Even );

  return std::pair <std::string, Parity> ( "odd", Parity::Odd );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, libposix::SerialPort::DataBits>
libposix::SerialPort::getDataBits( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, DataBits> ( "unknown", DataBits::Error );

  tcflag_t _databits = tty.c_cflag & (tcflag_t) CSIZE;
  switch( _databits ){
    default:  return std::pair <std::string, DataBits> ("unknown", DataBits::Error );
    case CS5: return std::pair <std::string, DataBits> ("5", DataBits::Five);
    case CS6: return std::pair <std::string, DataBits> ("6", DataBits::Six);
    case CS7: return std::pair <std::string, DataBits> ("7", DataBits::Seven);
    case CS8: return std::pair <std::string, DataBits> ("8", DataBits::Eight );
  }
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, libposix::SerialPort::StopBits>
libposix::SerialPort::getStopBits( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, StopBits> ( "unknown", StopBits::Error );

  if( !( tty.c_iflag & (tcflag_t) CSTOPB ) )
    return std::pair <std::string, StopBits> ( "1", StopBits::One );

  return std::pair <std::string, StopBits> ( "2", StopBits::Two );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, uint8_t>
libposix::SerialPort::getTimeout( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, uint8_t> ( "unknown", 0 );

  return std::pair <std::string, uint8_t> ( std::to_string( tty.c_cc[VTIME] ) , (uint8_t) tty.c_cc[VTIME] );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <std::string, uint8_t>
libposix::SerialPort::getMinBytes( void ){
  if( !this->isValid( ) || !this->getTermios( ) )
    return std::pair <std::string, uint8_t> ( "unknown", 0 );

  return std::pair <std::string, uint8_t> ( std::to_string( tty.c_cc[VMIN] ) , (uint8_t) tty.c_cc[VMIN] );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::string
libposix::SerialPort::print( bool toConsole ){
  std::ostringstream output;
  output << "Serial Port " << this->pathname << " Configuration\n"
         << "Baud Rate: " << this->getBaudRate( ).first << " [bps]\n"
         << "Parity: " << this->getParity( ).first << "\n"
         << "Data Bits: " << this->getDataBits( ).first << " [b]\n"
         << "Stop Bits: " << this->getStopBits( ).first << " [b]\n"
         << "Flow Control: " << this->getFlowControl( ).first << "\n"
         << "Timeout: " << this->getTimeout( ).first << " [ds]\n"
         << "Minimum Number Bytes: " << this->getMinBytes( ).first << " [B]\n";
  
  if( toConsole )
    std::cout << output.str( );

  return output.str();
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::~SerialPort( void ){
  this->disconnect( );
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::disconnect( void ){
  if( !this->isValid( ) )
    return StatusCode::Error;
  
  if( EOF == fclose( this->fp.get( ) ) ){
    errorPrint( "fclose" );
    return StatusCode::Error;
  }

  this->fd = -1;
  this->fp = std::unique_ptr<FILE, decltype(&fclose)> (nullptr, &fclose);
  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::flush( Buffer buf ){
  if( !this->isValid( ) )
    return StatusCode::Error;

  int option;
  switch( buf ){
    default: return StatusCode::Error;
    case Buffer::Input: option = TCIFLUSH; break;
    case Buffer::Output: option = TCOFLUSH; break;
    case Buffer::Both: option = TCIOFLUSH; break;
  }

  if( -1 == tcflush( this->fd, option ) ){
    errorPrint( "tcflush" );
    return StatusCode::Error;
  }

  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode
libposix::SerialPort::drain( void ){
  if( !this->isValid( ) )
    return StatusCode::Error;

  if( -1 == tcdrain( this->fd ) ){
    errorPrint( "tcdrain" );
    return StatusCode::Error;
  }

  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode 
libposix::SerialPort::fsError( void ){
  if( feof( this->fp.get( ) ) ){
    clearerr( this->fp.get( ) );
    return StatusCode::Timeout;
  }
  
  if( ferror( this->fp.get( ) ) )
    errorPrint( "ferror" );
  else 
    errorPrint( "else_ferror" );

  clearerr( this->fp.get( ) );
  return StatusCode::Error;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <size_t, libposix::SerialPort::StatusCode> 
libposix::SerialPort::readLine( char * buf, const size_t size, const size_t offset ){
  if( !this->isValid( ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);

  if( size <= offset ){
    errorPrint( "readLine overflow" );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }

  clearerr( this->fp.get( ) );

  if( !fgets( buf + offset, (int)(size - offset), this->fp.get( ) ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, this->fsError( ));

  return std::pair <size_t, libposix::SerialPort::StatusCode> (strlen( buf + offset), StatusCode::Success);
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <size_t, libposix::SerialPort::StatusCode> 
libposix::SerialPort::read( char * buf, const size_t size, const size_t offset, const size_t length ){
  if( !this->isValid( ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);

  if( size <= (offset + length) ){
    errorPrint( "read overflow" );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }

  clearerr( this->fp.get( ) );

  size_t len = fread( buf + offset, 1, length, this->fp.get( ) );

  if( length > len )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, this->fsError( ));

  return std::pair <size_t, libposix::SerialPort::StatusCode> (len, StatusCode::Success);  
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <size_t, libposix::SerialPort::StatusCode> 
libposix::SerialPort::write( const uint8_t * data, const size_t len ){
  if( !this->isValid( ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);

  if( !data )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::InvalidParameter);

  size_t size = fwrite( data, 1, len, this->fp.get( ) );

  if( size <  len )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, this->fsError( ));

  return std::pair <size_t, libposix::SerialPort::StatusCode> (size, StatusCode::Success);  
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <size_t, libposix::SerialPort::StatusCode> 
libposix::SerialPort::writef( const char * format, ... ){
  if( !this->isValid( ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);

  if( !format )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::InvalidParameter);

  va_list args;
  va_start( args, format );

  char buf[ PIPE_BUF ];
  int len = vsnprintf( buf, sizeof(buf), format, args );

  if( 0 > len ) {
    errorPrint( "vsnprintf" );
    va_end( args );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }

  if( (int) sizeof( buf ) <= len ) {
    errorPrint( "write overflow" );
    va_end( args );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }

  size_t size = fwrite( buf, 1, (size_t) len, this->fp.get( ) );

  va_end( args );
  if( size < (size_t) len )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, this->fsError( ));

  return std::pair <size_t, libposix::SerialPort::StatusCode> (size, StatusCode::Success);  
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <size_t, libposix::SerialPort::StatusCode> 
libposix::SerialPort::available( void ){
  if( !this->isValid( ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);

  int32_t len;
  if( -1 == ioctl( this->fd, FIONREAD, &len ) ){
    errorPrint( "available ioctl" );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }  

  if( 0 > len ){
    errorPrint( "available ioctl" );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }

  return std::pair <size_t, libposix::SerialPort::StatusCode> ((size_t) len, StatusCode::Success);
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
libposix::SerialPort::StatusCode  
libposix::SerialPort::setLineState( const Line _line, const bool state ){
  if( !this->isValid( ) )
    return StatusCode::Error;

  int status;
  if( -1 == ioctl( this->fd, TIOCMGET, &status ) ){
    errorPrint( "setLineState ioctl" );
    return StatusCode::Error;
  }
  
  if( state ) 
    status |= static_cast<std::uint8_t>(_line);
  else        
    status &= ~static_cast<std::uint8_t>(_line);

  if( -1 == ioctl( this->fd, TIOCMSET, &status ) ){
    errorPrint( "setLineState ioctl" );
    return StatusCode::Error;
  }

  return StatusCode::Success;
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
std::pair <bool, libposix::SerialPort::StatusCode> 
libposix::SerialPort::getLineState( const Line _line ){
  if( !this->isValid( ) )
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);

  int status;
  if( -1 == ioctl( this->fd, TIOCMGET, &status ) ){
    errorPrint( "getLineState ioctl" );
    return std::pair <size_t, libposix::SerialPort::StatusCode> (0, StatusCode::Error);
  }
  return std::pair <size_t, libposix::SerialPort::StatusCode> ( ( (uint8_t) status & static_cast<std::uint8_t>(_line) ) != 0, StatusCode::Success);
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/