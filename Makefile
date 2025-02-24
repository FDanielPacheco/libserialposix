CC   = gcc
CCPP = g++

CFLAGS = -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude
LDFLAGS = -pthread -lcheck_pic -lrt -lm -lsubunit -lcheck -lpthread

LIB_NAME = libserialposix
LIB_A = build/c/$(LIB_NAME).a

build/c: src/c/libserialposix.c include/libserialposix.h
		mkdir -p build/c/
		$(CC) $(CFLAGS) -c src/c/libserialposix.c -o build/c/libserialposix.o
		ar rcs $(LIB_A) build/c/libserialposix.o

build/c/example: example/usage_c_api.c include/libserialposix.h
		mkdir -p build/c/example/
		$(CC) $(CFLAGS) example/usage_c_api.c -o build/c/example/usage_c_api.out -Lbuild $(LIB_A) $(LDFLAGS)

build/c/test: include/libserialposix.h
		mkdir -p build/c/test/
		$(CC) $(CFLAGS) src/c/test/unit.c -o build/c/test/unit.out -Lbuild $(LIB_A) $(LDFLAGS)
		
install/c: clean build/c
		sudo cp include/libserialposix.h /usr/local/include
		sudo cp build/c/libserialposix.a /usr/local/lib/libserialposix.a

documentation:
		cd docs && doxygen && cd ..

clean:
		rm -rf docs/html docs/latex
		rm -rf build 

.PHONY: all clean  
all: clean build/c build/c/example