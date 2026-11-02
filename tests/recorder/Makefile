CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: run_rec_tests

run_rec_tests: test_run_rec.c ../../app_program/run_rec.c ../../app_program/run_rec.h
	$(CC) $(CFLAGS) -DVC_RECORDER=1 -I../../app_program -o $@ test_run_rec.c ../../app_program/run_rec.c

test: run_rec_tests
	./run_rec_tests

clean:
	$(RM) run_rec_tests run_rec_tests.exe
