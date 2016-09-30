# Makefile for the MASDR Application
CC=g++
CFLAGS=-c -Wall

all: masdr doc
	
masdr: masdr.cpp
	$(CC) $(CFLAGS) masdr.cpp -o masdr

doc: masdr.h config.dox
	doxygen config.dox

clean:
	rm -rf masdr *.o docs