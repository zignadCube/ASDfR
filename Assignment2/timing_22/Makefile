# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

.PHONY = all clean

CC = g++

CFLAGS = -I/usr/evl/include -L/usr/evl/lib -levl

SRC := timing_22.cpp
OUT := timing_22

all:
	$(CC) $(SRC) -o $(OUT) $(CFLAGS)

clean:
	@echo "Cleaning up"
	@rm -rvf *.o ${OUT}

clean-txt:
	@echo "Cleaning up measurements"
	@rm -rvf *.txt