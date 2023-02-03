/*******************************************************************************

  Copyright © 2019

  Permission is hereby granted, free of charge, to any person obtaining a copy 
  of this software and associated documentation files (the “Software”), to deal 
  in the Software without restriction, including without limitation the rights 
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
  copies of the Software, and to permit persons to whom the Software is 
  furnished to do so, subject to the following conditions:

  1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
  SOFTWARE.

  Author: doerthous <doerthous@gmail.com>

*******************************************************************************/


#include "serial.h"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(__WIN32__)
# define _SRL_WIN32_
# include <windows.h>
#elif defined(__CYGWIN32__) || defined(__CYGWIN__)
# define _SRL_CYGWIN32_
# define HANDLE int
#elif defined(__linux)
# define _SRL_LINUX_
# define HANDLE int
#else
# error "Not support platform!"
#endif

typedef struct serial
{
    HANDLE fd;
    int rx_timeout;
} * serial_t;

#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>


// --------------------------- SRL WIN32 ---------------------------------------
#if defined(_SRL_WIN32_)


static int serial_other_config(serial_t sr, DCB *options)
{

    if (!SetupComm(sr->fd, 2048, 2048)) {
        fprintf(stderr, "serial in/output buffer setting failed.\n");
    }

    PurgeComm(sr->fd, PURGE_TXCLEAR|PURGE_RXCLEAR);

    return 1;
}

static int serial_set_baud_rate(serial_t sr, DCB *options, int baud)
{
    switch(baud) {
        case 4800:
            options->BaudRate = CBR_4800;
            break;
        case 9600:
            options->BaudRate = CBR_9600;
            break;
        case 19200:
            options->BaudRate = CBR_9600;
            break;
        case 38400:
            options->BaudRate = CBR_38400;
            break;
        case 115200:
            options->BaudRate = CBR_115200;
            break;
        default:
            fprintf(stderr,"Unkown baud rate!\n");
            return 0;
    }

    return 1;
}

static int serial_set_data_bit(serial_t sr, DCB *options, int data_bit)
{
    switch(data_bit) {
        case 5:
        case 6:
        case 7:
        case 8:
            options->ByteSize = data_bit;
            break;
        default:
            fprintf(stderr,"Unkown data bit!\n");
            return 0;
    }

    return 1;
}

static int serial_set_parity_check(serial_t sr, DCB *options, char parity)
{
    switch(parity) {
        case 'n':
        case 'N':
            options->fParity = 1;
            options->Parity = NOPARITY;
            break;
        case 'o':
        case 'O':
            options->fParity = 1;
            options->Parity = ODDPARITY;
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options->fParity = 1;
            options->Parity = EVENPARITY;
            break;
        default:
            fprintf(stderr,"Unkown parity check!\n");
            return 0;
    }

    return 1;
}

static int serial_set_stop_bit(serial_t sr, DCB *options, int stop_bit)
{
    switch(stop_bit) {
        case 1:
            options->StopBits = ONESTOPBIT;
            break;
        case 2:
            options->StopBits = TWOSTOPBITS;
            break;
        default:
            fprintf(stderr,"Unkown stop bit!\n");
            return 0;
    }

    return 1;
}

static int serial_set_flow_control(serial_t sr, DCB *options, char fctrl)
{
    switch(fctrl) {
        case 'N':
        case 'n':
            //
            //break;
        case 'H':
        case 'h':
            //
            //break;
        case 'S':
        case 's':
            //
            //break;
        default:
            fprintf(stderr,"%s, Not support.\n", __func__);
            return 0;
    }

    return 1;
}

static int serial_set_rts_dtr(serial_t sr, int rts_dtr, int val)
{
    fprintf(stderr, "%s, Not support.\n", __func__);
    return 0;
}

static int _serial_set(serial_t sr, va_list args)
{
    int p;
    DCB options;


    GetCommState(sr->fd, &options);

    while ((p = va_arg(args, int)) != 0) {
        switch (p) {
            case SRL_BAUD_RATE:
                serial_set_baud_rate(sr, &options, va_arg(args, int));
                break;
            case SRL_PARITY_CHECK:
                serial_set_parity_check(sr, &options, va_arg(args, int));
                break;
            case SRL_STOP_BIT:
                serial_set_stop_bit(sr, &options, va_arg(args, int));
                break;
            case SRL_DATA_BIT:
                serial_set_data_bit(sr, &options, va_arg(args, int));
                break;
            case SRL_FLOW_CONTROL:
                serial_set_flow_control(sr, &options, va_arg(args, int));
                break;
            case SRL_RTS:
                #define RTS 0
                serial_set_rts_dtr(sr, RTS, va_arg(args, int));
                break;
            case SRL_DRT:
                #define DTR 1
                serial_set_rts_dtr(sr, DTR, va_arg(args, int));
                break;
            case SRL_RX_TIMEOUT:
                sr->rx_timeout = va_arg(args, int);
                break;
        }
    }

    serial_other_config(sr, &options);

    SetCommState(sr->fd, &options);

    return 1;
}

serial_t serial_open(const char *name, ...)
{
    serial_t sr = NULL;
    va_list args;
    int ret;


    assert(name);

    if ((sr = malloc(sizeof(struct serial))) != NULL) {
        sr->fd = CreateFile(name, GENERIC_READ|GENERIC_WRITE, //允许读和写
            0, //独占方式
            NULL,
            OPEN_EXISTING, //打开而不是创建
            0, //同步方式
            NULL);
        sr->rx_timeout = 0;

        if(sr->fd == INVALID_HANDLE_VALUE) {
            perror("Open UART failed!");
            free(sr);
            sr = NULL;
        }
        else {
            va_start(args, name);
            ret = _serial_set(sr, args);
            va_end(args);

            if (!ret) {
                free(sr);
                sr = NULL;
            }
        }
    }

    return sr;
}

void serial_close(serial_t serial)
{
    if (serial) {
        if (!CloseHandle(serial->fd)) {
            fprintf(stderr, "serial close failed.\n");
        }
        free(serial);
    }
}

size_t serial_write(serial_t sr, const uint8_t *data, size_t size)
{
    size_t twc = 0;
    DWORD wc;

    while(size > 0) {
        if (!WriteFile(sr->fd, (void *)data, (DWORD)size, &wc, NULL)) {
            fprintf(stderr, "serial write failed.\n");
            break;
        }

        size -= (int)wc;
        data += (int)wc;
        twc += (int)wc;
    }

    return twc;
}

size_t serial_read(serial_t sr, uint8_t *buff, size_t size)
{
    DWORD rc = 0;
    size_t trc = 0;


    COMMTIMEOUTS timeouts;
    GetCommTimeouts(sr->fd, &timeouts);
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = sr->rx_timeout;
    SetCommTimeouts(sr->fd,&timeouts);

    while(size > 0) {
        if (!ReadFile(sr->fd, buff, size, &rc, NULL)) {
            fprintf(stderr, "serial read failed.\n");
            break;
        }
        if (rc == 0) {
            break;
        }

        size -= (int)rc;
        buff += (int)rc;
        trc += (int)rc;
    }

    return trc;
}




// --------------------------- SRL CYGWIN32 ------------------------------------
#elif defined(_SRL_CYGWIN32_) || defined(_SRL_LINUX_)


#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>


static int serial_other_config(int fd, struct termios *options)
{
    /*设置控制模式*/
    options->c_cflag |= CLOCAL;//保证程序不占用串口
    options->c_cflag |= CREAD;//保证程序可以从串口中读取数据


    /*设置输出模式为原始输出*/
    options->c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    /*设置本地模式为原始模式*/
    options->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */
    options->c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);


    // If data is available, read(2) returns immediately, with the lesser of the
    // number of bytes available, or the number of bytes requested.  If no data
    // is available, read(2) returns 0.
    options->c_cc[VTIME] = 0;
    options->c_cc[VMIN] = 0;

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd, TCIFLUSH);

    return 1;
}

static int serial_set_baud_rate(int fd, struct termios *options, int baud)
{
    switch(baud) {
        case 4800:
            cfsetispeed(options, B4800);
            cfsetospeed(options, B4800);
            break;
        case 9600:
            cfsetispeed(options, B9600);
            cfsetospeed(options, B9600);
            break;
        case 19200:
            cfsetispeed(options, B19200);
            cfsetospeed(options, B19200);
            break;
        case 38400:
            cfsetispeed(options, B38400);
            cfsetospeed(options, B38400);
            break;
        case 115200:
            cfsetispeed(options, B115200);
            cfsetospeed(options, B115200);
            break;
        default:
            fprintf(stderr,"Unkown baud rate!\n");
            return 0;
    }

    return 1;
}

static int serial_set_data_bit(int fd, struct termios *options, int data_bit)
{
    switch(data_bit) {
        case 5:
            options->c_cflag &= ~CSIZE;//屏蔽其它标志位
            options->c_cflag |= CS5;
            break;
        case 6:
            options->c_cflag &= ~CSIZE;//屏蔽其它标志位
            options->c_cflag |= CS6;
            break;
        case 7:
            options->c_cflag &= ~CSIZE;//屏蔽其它标志位
            options->c_cflag |= CS7;
            break;
        case 8:
            options->c_cflag &= ~CSIZE;//屏蔽其它标志位
            options->c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown data bit!\n");
            return 0;
    }

    return 1;
}

static int serial_set_parity_check(int fd, struct termios *options, char parity)
{
    switch(parity) {
        case 'n':
        case 'N':
            options->c_cflag &= ~PARENB;
            break;
        case 'o':
        case 'O':
            options->c_cflag |= PARENB;
            options->c_cflag |= PARODD;
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options->c_cflag |= PARENB;
            options->c_cflag &= ~PARODD;
            break;
        default:
            fprintf(stderr,"Unkown parity check!\n");
            return 0;
    }

    return 1;
}

static int serial_set_stop_bit(int fd, struct termios *options, int stop_bit)
{
    switch(stop_bit) {
        case 1:
            options->c_cflag &= ~CSTOPB;
            break;
        case 2:
            options->c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unkown stop bit!\n");
            return 0;
    }

    return 1;
}

static int serial_set_flow_control(int fd, struct termios *options, char fctrl)
{
    switch(fctrl) {
        case 'N':
        case 'n':
            options->c_cflag &= ~CRTSCTS;
            break;
        case 'H':
        case 'h':
            options->c_cflag |= CRTSCTS;
            break;
        case 'S':
        case 's':
            options->c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown flow control!\n");
            return 0;
    }

    return 1;
}

static int serial_set_rts_dtr(int fd, int rts_dtr, int val)
{
    if (!val) {
        ioctl(fd, TIOCMBIC, &rts_dtr);
    }
    else {
        ioctl(fd, TIOCMBIS, &rts_dtr);
    }
}

static int _serial_set(serial_t sr, va_list args)
{
    int p;
    struct termios options;


    if(tcgetattr(sr->fd, &options) < 0) {
        perror("tcgetattr error");
        return 0;
    }

    while ((p = va_arg(args, int)) != 0) {
        switch (p) {
            case SRL_BAUD_RATE:
                serial_set_baud_rate(sr->fd, &options, va_arg(args, int));
                break;
            case SRL_PARITY_CHECK:
                serial_set_parity_check(sr->fd, &options, va_arg(args, int));
                break;
            case SRL_STOP_BIT:
                serial_set_stop_bit(sr->fd, &options, va_arg(args, int));
                break;
            case SRL_DATA_BIT:
                serial_set_data_bit(sr->fd, &options, va_arg(args, int));
                break;
            case SRL_FLOW_CONTROL:
                serial_set_flow_control(sr->fd, &options, va_arg(args, int));
                break;
            case SRL_RTS:
                serial_set_rts_dtr(sr->fd, TIOCM_RTS, va_arg(args, int));
                break;
            case SRL_DRT:
                serial_set_rts_dtr(sr->fd, TIOCM_DTR, va_arg(args, int));
                break;
            case SRL_RX_TIMEOUT:
                sr->rx_timeout = va_arg(args, int);
                break;
        }
    }

    serial_other_config(sr->fd, &options);

    if(tcsetattr(sr->fd, TCSANOW, &options) < 0) {
        perror("tcsetattr failed");
        return 0;
    }

    return 1;
}


serial_t serial_open(const char *name, ...)
{
    serial_t sr = NULL;
    va_list args;
    int ret;


    assert(name);

    if ((sr = malloc(sizeof(struct serial))) != NULL) {
        sr->fd = open(name, O_RDWR|O_NOCTTY|O_NONBLOCK);
        sr->rx_timeout = 0;

        if(sr->fd == -1) {
            perror("Open UART failed!");
            free(sr);
            sr = NULL;
        }
        else {
            va_start(args, name);
            ret = _serial_set(sr, args);
            va_end(args);

            if (!ret) {
                free(sr);
                sr = NULL;
            }
        }
    }

    return sr;
}

void serial_close(serial_t serial)
{
    if (serial) {
        close(serial->fd);
        free(serial);
    }
}

size_t serial_write(serial_t serial, const uint8_t *data, size_t size)
{
    ssize_t wc = 0;
    size_t twc = 0;


    while(size > 0) {
        if((wc = write(serial->fd, data, size)) == -1) {
            if(errno == EINTR) {
                wc = 0;
            }
            else {
                fprintf(stderr, "serial write errno: %d\n", errno);
                break;
            }
        }

        size -= wc;
        data += wc;
        twc += wc;
    }

    return twc;
}

size_t serial_read(serial_t serial, uint8_t *buff, size_t size)
{
    ssize_t rc = 0;
    size_t trc = 0;

    fd_set fds; 
    int ret = -1;
    while (size > 0) {
        FD_ZERO(&fds);
        FD_SET(serial->fd, &fds);
        struct timeval timeout = {
            serial->rx_timeout/1000, serial->rx_timeout%1000*1000
        };

        ret = select(serial->fd+1, &fds, NULL, NULL, &timeout);
        if (ret == -1) {
            return 0;
        }
        if (ret == 0) {
            return trc;
        }
        if(FD_ISSET(serial->fd, &fds)) {
            rc = read(serial->fd, buff, size);
            size -= rc;
            buff += rc;
            trc += rc;
        }
    }

    return trc;
}

#endif


void serial_flush(serial_t sr, int option)
{
    assert(option == SRL_FLUSH_I 
        || option == SRL_FLUSH_O 
        || option == (SRL_FLUSH_I|SRL_FLUSH_O));
    
#if defined(_SRL_WIN32_)
    if (option == (SRL_FLUSH_I|SRL_FLUSH_O)) {
        PurgeComm(sr->fd, PURGE_TXCLEAR|PURGE_RXCLEAR);
        return;
    }
    if (option == SRL_FLUSH_I) {
        PurgeComm(sr->fd, PURGE_RXCLEAR);
        return;
    }
    if (option == SRL_FLUSH_O) {
        PurgeComm(sr->fd, PURGE_TXCLEAR);
        return;
    }
#elif defined(_SRL_CYGWIN32_) || defined(_SRL_LINUX_)
    if (option == (SRL_FLUSH_I|SRL_FLUSH_O)) {
        tcflush(sr->fd, TCIOFLUSH);
        return;
    }
    if (option == SRL_FLUSH_I) {
        tcflush(sr->fd, TCIFLUSH);
        return;
    }
    if (option == SRL_FLUSH_O) {
        tcflush(sr->fd, TCOFLUSH);
        return;
    }
#endif
}
