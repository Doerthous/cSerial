/**
  ******************************************************************************
  * \brief      
  * \file       serial.h
  * \author     doerthous
  * \date       2019-09-12
  * \details    
  ******************************************************************************
  */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(__WIN32__)
  #define _SRL_WIN32_
#elif defined(__CYGWIN32__) || defined(__CYGWIN__)
  #define _SRL_CYGWIN32_
#else
  #error "Not support platform!"
#endif

#ifdef _SRL_WIN32_
  #include <windows.h>
#endif

typedef struct serial
{
#ifdef _SRL_WIN32_
    HANDLE fd;
#elif defined(_SRL_CYGWIN32_)
    int fd;
#endif
    
    int rx_timeout;
} * serial_t;

enum serial_param
{
    SRL_BAUD_RATE = 1, // 
    SRL_DATA_BIT, // 5, 6, 7, 8
    SRL_PARITY_CHECK, // 'N','n','E','e','O','o'
    SRL_STOP_BIT, // 1, 2
    SRL_FLOW_CONTROL, // 'N','n','H','h','S','s'
    SRL_RTS, // 0, 1(or others)
    SRL_DRT, // 0, 1(or others)
    SRL_RX_TIMEOUT, // 
};

serial_t serial_open(const char *name, ...);
void serial_close(serial_t serial);
uint32_t serial_write(serial_t serial, const uint8_t *data, uint32_t size);
uint32_t serial_read(serial_t serial, uint8_t *buff, uint32_t size);


#endif /* SERIAL_H_ */

/****************************** Copy right 2019 *******************************/
