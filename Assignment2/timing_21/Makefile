# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

.PHONY = all clean

CC = g++

CFLAGS = 

SRC := timing_21.cpp
OUT := timing_21

all:
	$(CC) $(SRC) -o $(OUT) $(CFLAGS)

clean:
	@echo "Cleaning up"
	@rm -rvf *.o ${OUT}

clean-txt:
	@echo "Cleaning up measurements"
	@rm -rvf *.txt