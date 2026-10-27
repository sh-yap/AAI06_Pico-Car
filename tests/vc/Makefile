CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: vc_tests

vc_tests: test_vc.c ../../app_program/vc.c ../../app_program/vc.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_vc.c ../../app_program/vc.c

test: vc_tests
	./vc_tests

clean:
	$(RM) vc_tests vc_tests.exe
