
# Makefile for the Query_CAN example.
#
# Alta Engineering 2017
#

CC=gcc

all: Query_CAN

Query_CAN: Query_CAN.o FiFo.o
	@$(CC) Query_CAN.o FiFo.o -lpthread -o Query_CAN

Query_CAN.o: Query_CAN.c FiFo.h CAN_Devices.h
	@$(CC) -c -Wall Query_CAN.c

FiFo.o: FiFo.c FiFo.h
	@$(CC) -c -Wall FiFo.c

clean:
	-@rm Query_CAN Query_CAN.o FiFo.o
	


