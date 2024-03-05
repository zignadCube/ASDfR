# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

.PHONY = all clean

CC = g++

CFLAGS = 

SRC := timing_2.1.cpp
OUT := timing_2.1

all:
	$(CC) $(SRC) -o $(OUT) $(CFLAGS)

clean:
	@echo "Cleaning up"
	@rm -rvf *.o ${OUT}