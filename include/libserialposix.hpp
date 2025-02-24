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

#include <stdlib.h>    
#include <stdarg.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/limits.h>

#include <string>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cerrno>
#include <memory>
#include <vector>
#include <stdexcept>

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Objects
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

namespace 
serial_port {
  class
  SerialPort {
    private:
      enum class
      FlowControl : uint8_t {
        None,
        Hardware,
        Software,
      };
    
      enum class 
      Parity : uint8_t {
        None,
        Odd,
        Even,
      };
    
      enum class
      DataBits : uint8_t {
        Five = 5,
        Six = 6,
        Seven = 7,
        Eight = 8,
      };
    
      enum class
      StopBits : uint8_t {
        One = 1,
        Two = 2,
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

      struct 
      Configuration {
        bool        readonly   = false;                                        //!< If the file pointer will be given in read only mode, example NO = r+.
        BaudRate    baudrate   = B9600;                                        //!< The baud rate of the communication in bits per second, example B9600.
        FlowControl flow       = FlowControl::None;                            //!< The hardware flow control, example Hardware.
        Parity      parity     = Parity::None;                                 //!< The detection of error parity, example Odd.
        DataBits    bData      = DataBits::Eight;                              //!< The number of bits per serial word, example 8.
        StopBits    bStop      = StopBits::One;                                //!< The number of stop bits per serial word, example 1.
        uint8_t     timeout    = 1;                                            //!< The time any read function will wait in deciseconds for the information to arrive, example 200.
        uint8_t     minBytes   = 0;                                            //!< The minimum number of bytes to necessary receive before returning the read function.
      };
      
      enum class
      StatusCode {
        Success,
        Error,
      };
      
      StatusCode open( const std::string &pathname );
      StatusCode close( void );

      enum class
      Buffer {
        Input,
        Output,
      };

    protected:
      Configuration                                config;                     //!< The confifguration of the serial port object.
      std::unique_ptr <FILE, decltype( &fclose )>  fp;                         //!< The file descriptor for the serial port opened.
      std::unique_ptr <int, decltype( &close )>    fd;                         //!< The file pointer for the serial port opened.
      std::string                                  pathname;                   //!< The path to the serial port device in Linux file system, example "/dev/ttyUSB0"

    public:
      SerialPort( const std::string &_pathname );
      ~SerialPort( void );  

      StatusCode   setReadOnly    ( bool _state );
      StatusCode   setBaudRate    ( BaudRate _baudrate );
      StatusCode   setFlowControl ( FlowControl _flow );
      StatusCode   setParity      ( Parity _parity );
      StatusCode   setDataBits    ( DataBits _databits );
      StatusCode   setStopBits    ( StopBits _stopbits );
      StatusCode   setTimeout     ( uint8_t _timeout );
      StatusCode   setMinBytes    ( uint8_t _minbytes );  

      std::pair <bool, StatusCode>        getReadOnly    ( void );
      std::pair <BaudRate, StatusCode>    getBaudRate    ( void );
      std::pair <FlowControl, StatusCode> getFlowControl ( void );
      std::pair <Parity, StatusCode>      getParity      ( void );
      std::pair <DataBits, StatusCode>    getDataBits    ( void );
      std::pair <StopBits, StatusCode>    getStopBits    ( void );
      std::pair <uint8_t, StatusCode>     getTimeout     ( void );
      std::pair <uint8_t, StatusCode>     getMinBytes    ( void );

      StatusCode   flush( Buffer buf );
      StatusCode   drain( void );
      
      bool         isValid( void );

      std::pair <size_t, StatusCode> readLine ( std::vector <char> &buf, const size_t offset );
      std::pair <size_t, StatusCode> read     ( std::vector <char> &buf, const size_t offset, const size_t length );
      std::pair <size_t, StatusCode> write    ( const std::string  &format, ... );      

      std::pair <size_t, StatusCode> available( void );

      StatusCode  setLineState( Line _line, bool state );
      std::pair <bool, StatusCode> getLineState( Line _line );
  };
}

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Definition file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#endif

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * End of file
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/