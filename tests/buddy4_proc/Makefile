CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
SRC = ../../app_program/b4_proc.c ../../app_program/b4_cfg.c

all: b4_proc_tests

b4_proc_tests: test_b4_proc.c $(SRC)
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_b4_proc.c $(SRC) -lm

test: b4_proc_tests
	./b4_proc_tests

clean:
	$(RM) b4_proc_tests b4_proc_tests.exe
