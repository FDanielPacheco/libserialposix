 /***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      libserialposix.hpp
 * 
 * @version   1.0
 *
 * @date      19-02-2025
 *
 * @brief     Prototype of functions and object, providing control, local settings, input, and output for serial ports on Linux, library developed for interacting with embedded systems in mind (non-cannonical).  
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 * @copyright Copyright (c) [2025] [Fábio D. Pacheco]
 * 
 * @note      Manuals:
 *            https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html , 
 *            https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#ifndef LIBSERIALPOSIX_HPP
#define LIBSERIALPOSIX_HPP

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

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Objects
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

namespace 
libposix{
  class
  SerialPort {
    public:
      enum class
      FlowControl : uint8_t {
        None,
        Hardware,
        Software,
        Error,
      };
    
      enum class 
      Parity : uint8_t {
        None,
        Odd,
        Even,
        Error,
      };
    
      enum class
      DataBits : uint8_t {
        Five = CS5,
        Six = CS6,
        Seven = CS7,
        Eight = CS8,
        Error,
      };
    
      enum class
      StopBits : uint8_t {
        One = 1,
        Two = 2,
        Error,
      };
    
      enum class
      Line {
        DSR = TIOCM_LE,
        DTR = TIOCM_DTR,
        RTS = TIOCM_RTS,
        CTS = TIOCM_CTS,
        DCD = TIOCM_CAR,
      };
      
      typedef speed_t BaudRate;
      
      enum class
      StatusCode {
        Error,
        Success,
        InvalidParameter,
        DeviceNotFound,
        NameToLong,
        Timeout,
        BufferOverflow,
        IOError
      };
      
      enum class
      Buffer {
        Input,
        Output,
        Both,
      };

      struct 
      Configuration {
        bool        readonly   = false;                                        //!< If the file pointer will be given in read only mode, default false = r+.
        BaudRate    baudrate   = B9600;                                        //!< The baud rate of the communication in bits per second, default B9600.
        FlowControl flow       = FlowControl::None;                            //!< The hardware flow control, default None.
        Parity      parity     = Parity::None;                                 //!< The detection of error parity, default None.
        DataBits    bData      = DataBits::Eight;                              //!< The number of bits per serial word, default 8.
        StopBits    bStop      = StopBits::One;                                //!< The number of stop bits per serial word, default 1.
        uint8_t     timeout    = 1;                                            //!< The time any read function will wait in deciseconds for the information to arrive, default 1.
        uint8_t     minBytes   = 0;                                            //!< The minimum number of bytes to necessary receive before returning the read function, default 0.
      };

      explicit SerialPort( void );
      ~SerialPort( void );  

      /**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
      * @brief Initializes the serial port interface.
      *  
      * @param[in]  _pathname The absolute path for the device file, example `/dev/ttyUSB0`
      * @param[in]  _config The serial port configuration data structure (`SerialPort::Configuration`), this configuration must have been updated with the user's desired information, can be nullable (std::nullopt) or not specified if the user desire the default configuration present in `Configuration`. 
      * @param[in]  _rdonly The permission of the file descriptor associated with the serial port, 0 is read/write and 1 is read only, can be nullable (std::nullopt) or not specified. 
      * 
      * @return Upon success, performing the process of opening the serial port, it returns StatusCode::Success.\n
      *         Otherwise, returns the indication of the error.
      * 
      *  - `InvalidParameter`: Invalid argument \n
      *  - `NameToLong`: Pathname to long \n
      *  - `DeviceNotFound`: Open operation failed \n
      *  - `Error`: Other error
      * 
      * @b Example \n
      *  Opening the serial port with 115200 bps and 1 second until timeout, other configurations are the default: \n
      *  Method-1:
      * @code{.c}
      *    using namespace libposix;
      *    SerialPort::Configuration cfg;
      *    cfg.baudrate = B115200;
      *    cfg.timeout = 10;
      *    SerialPort sr;
      *    if( SerialPort::StatusCode::Success != sr.connect( "/dev/ttyUSB0", cfg ) )
      *      // handle error
      *    // ready to use ...
      * @endcode
      *   Method-2:
      * @code{.c}
      *    using namespace libposix;
      *    SerialPort sr;
      *    if( SerialPort::StatusCode::Success != sr.connect( "/dev/ttyUSB0" ) )
      *      // handle error
      *    sr.setBaudRate( B115200 );
      *    sr.setRule( 10, 0 );
      *    // ready to use ...
      * @endcode
      *   
      **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
      StatusCode connect( const std::string &_pathname, std::optional <SerialPort::Configuration> _config = std::nullopt, std::optional <uint8_t> _rdonly = std::nullopt );
 
      /**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
      * @brief Updates the serial port configuration based on the configuration structure passed as argument.
      *  
      * @param[in] _config The serial port configuration data structure (`Configuration`), this configuration must have been updated with the user's desired information. 
      * 
      * @return Upon success, the serial port current configuration is updated, and it returns StatusCode::Sucess. \n 
      *         Otherwise, returns StatusCode::Error.
      * 
      **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
      StatusCode update( const Configuration &_config );

      /**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
      * @brief Closes the connection with the serial port interface.
      *  
      * @param none
      * 
      * @return Upon success, the serial port current configuration is updated, and it returns StatusCode::Sucess. \n 
      *         Otherwise, returns StatusCode::Error.
      *
      **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
      StatusCode disconnect( void );

      void       setReadOnly    ( const bool _state );
      StatusCode setBaudRate    ( const BaudRate _baudrate );
      StatusCode setFlowControl ( const FlowControl _flow );
      StatusCode setParity      ( const Parity _parity );
      StatusCode setDataBits    ( const DataBits _databits );
      StatusCode setStopBits    ( const StopBits _stopbits );
      StatusCode setRule        ( const uint8_t _timeout, const uint8_t _minbytes );

      std::pair <std::string, bool>        isReadOnly     ( void );
      std::pair <std::string, BaudRate>    getBaudRate    ( void );
      std::pair <std::string, FlowControl> getFlowControl ( void );
      std::pair <std::string, Parity>      getParity      ( void );
      std::pair <std::string, DataBits>    getDataBits    ( void );
      std::pair <std::string, StopBits>    getStopBits    ( void );
      std::pair <std::string, uint8_t>     getTimeout     ( void );
      std::pair <std::string, uint8_t>     getMinBytes    ( void );

      std::string print( bool toConsole );

      StatusCode flush( Buffer buf );
      StatusCode drain( void );
      
      bool       isValid( void );

      std::pair <size_t, StatusCode> readLine ( char * buf, const size_t size, const size_t offset );
      std::pair <size_t, StatusCode> read     ( char * buf, const size_t size, const size_t offset, const size_t length );
      std::pair <size_t, StatusCode> write    ( const uint8_t * data, const size_t len );      
      std::pair <size_t, StatusCode> writef   ( const char * format, ... );      

      std::pair <size_t, StatusCode> available( void );

      StatusCode  setLineState( const Line _line, const bool state );
      std::pair <bool, StatusCode> getLineState( const Line _line );

    private:
      struct termios tty;
      bool getTermios( void );
      bool applyTermios( void );
      StatusCode fsError( void );

    protected:
      int                                          fd;                         //!< The file descriptor for the serial port opened.
      std::unique_ptr <FILE, decltype( &fclose )>  fp;                         //!< The file pointer for the serial port opened.
      std::string                                  pathname;                   //!< The path to the serial port device in Linux file system, example "/dev/ttyUSB0"
      Configuration                                config;                     //!< The confifguration of the serial port object.
  };
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/