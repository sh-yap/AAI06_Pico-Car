CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
SRC = ../../app_program/b4_proc.c ../../app_program/b4_pack.c \
      ../../app_program/b4_cfg.c

all: b4_proc_tests b4_pack_tests

b4_proc_tests: test_b4_proc.c $(SRC)
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_b4_proc.c $(SRC) -lm

b4_pack_tests: test_b4_pack.c ../../app_program/b4_pack.c \
               ../../app_program/b4_pack.h ../../app_program/b4_types.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_b4_pack.c \
	    ../../app_program/b4_pack.c

test: all
	./b4_pack_tests
	./b4_proc_tests

clean:
	$(RM) b4_proc_tests b4_proc_tests.exe b4_pack_tests b4_pack_tests.exe
