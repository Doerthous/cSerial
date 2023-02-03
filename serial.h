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


#ifndef SERIAL_H_
#define SERIAL_H_

#if defined(_MSC_VER)
# if defined(SERIAL_API_EXPORT)
#  define SERIAL_API __declspec(dllexport)
# elif defined(SERIAL_API_IMPORT)
#  define SERIAL_API __declspec(dllimport)
# endif
#endif
#ifndef SERIAL_API
# define SERIAL_API
#endif

#ifdef  __cplusplus
# define SERIAL_BEGIN_DECLS  extern "C" {
# define SERIAL_END_DECLS    }
#else
# define SERIAL_BEGIN_DECLS
# define SERIAL_END_DECLS
#endif

#include <stdint.h>
#include <stddef.h>

SERIAL_BEGIN_DECLS

struct serial;
typedef struct serial * serial_t;

enum serial_param
{
    SRL_BAUD_RATE = 1, //
    SRL_DATA_BIT,      // 5, 6, 7, 8
    SRL_PARITY_CHECK,  // 'N','n','E','e','O','o'
    SRL_STOP_BIT,      // 1, 2
    SRL_FLOW_CONTROL,  // 'N','n','H','h','S','s'
    SRL_RTS,           // 0, 1(or others)
    SRL_DRT,           // 0, 1(or others)
    SRL_RX_TIMEOUT,    //
    SRL_NULL = 0,
};

SERIAL_API 
serial_t serial_open(const char *name, ...);

SERIAL_API 
void serial_close(serial_t serial);

SERIAL_API 
size_t serial_write(serial_t serial, const uint8_t *data, size_t size);

SERIAL_API 
size_t serial_read(serial_t serial, uint8_t *buff, size_t size);

enum
{
    SRL_FLUSH_I = (1UL << 0),
    SRL_FLUSH_O = (1UL << 1),
};
SERIAL_API 
void serial_flush(serial_t serial, int dir);

SERIAL_END_DECLS

#endif /* SERIAL_H_ */
