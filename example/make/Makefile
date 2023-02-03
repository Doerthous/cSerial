SRC=../../serial.c main.c
INC=../../

ifeq ("$(uname -s)","Linux")
	LIB=so
	LINK_LIB=cserial
else
	LIB=lib
	LINK_LIB=libcserial.lib
endif

all: lib main

main: 
	gcc -I$(INC) $(SRC) -o $@

lib:
	gcc ../../serial.c -fPIC -shared -o libcserial.$(LIB)

libmain: lib
	gcc -I$(INC) main.c -L. -Wl,-rpath=. -llibcserial -o $@

clean:
	-rm -rf *.exe *.so *.lib