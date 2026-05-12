# Host-side unit tests for the pure-C bits of the firmware.
# Run with:  make -C tools test

CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -O2 -g -DNVCONFIG_HOST_TEST
INCLUDES = -I../Core/Inc

SRCS = ../Core/Src/dshot_gcr.c ../Core/Src/rpm_stats.c ../Core/Src/calibration.c ../Core/Src/nvconfig.c decode_test.c
BIN  = decode_test

$(BIN): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) -o $@

.PHONY: test
test: $(BIN)
	./$(BIN)

.PHONY: clean
clean:
	rm -f $(BIN)
