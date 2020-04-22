SRC=serial.c main.c

LIB_OUTPUT=build/lib
BIN_OUTPUT=build/bin

ifeq ("$(uname -s)","Linux")
	LIB=so
	LINK_LIB=cserial
else
	LIB=lib
	LINK_LIB=libcserial.lib
endif

prefix=/usr

all: lib main

main: 
	mkdir -p $(BIN_OUTPUT)
	gcc serial.c main.c -o $(BIN_OUTPUT)/main.exe

lib:
	mkdir -p $(LIB_OUTPUT)
	gcc serial.c -fPIC -shared -o $(LIB_OUTPUT)/libcserial.$(LIB)

libmain: lib
	mkdir -p $(BIN_OUTPUT)
	gcc main.c -L$(LIB_OUTPUT) -Wl,-rpath=$(LIB_OUTPUT) -llibcserial -o $(BIN_OUTPUT)/libmain.exe

# make install prefix=/install_dir
install: lib
	install -D  serial.h $(prefix)/include/de/io/serial.h
	install -D $(LIB_OUTPUT)/libcserial.$(LIB) $(prefix)/lib/libcserial.$(LIB)

clean:
	-rm -rf build