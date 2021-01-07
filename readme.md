## Brief

A Simple Serial Port Interface.

### Usage

```C
#include <stdio.h>
#include <string.h>
#include "serial.h"

int main(int argc, char const *argv[])
{
    serial_t sr;
    int rc, wc;
    char buff[1024];

    sr = serial_open(argv[1], 
        SRL_BAUD_RATE, 9600, 
        SRL_RTS, 0, 
        SRL_DRT, 1,
        SRL_RX_TIMEOUT, 3000,
        SRL_NULL);

    if (!sr) {
        fprintf(stderr, "serial open failed.\n");
        return -1;
    }

    wc = serial_write(sr, argv[2], strlen(argv[2]));

    printf("write(%d): %s\n", wc, argv[2]);

    if ((rc = serial_read(sr, buff, 1024)) > 0) {
        buff[rc] = '\0';
        printf("read(%d): %s\n", rc, buff);
    }

    serial_close(sr);
}
```