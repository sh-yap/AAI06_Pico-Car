CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O2

all: motion_control_tests

motion_control_tests: test_motion_control.c ../../app_program/motion_control.c ../../app_program/motion_control.h ../../app_program/motion_config.c ../../app_program/motion_config.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_motion_control.c ../../app_program/motion_control.c ../../app_program/motion_config.c -lm

test: motion_control_tests
	./motion_control_tests

clean:
	$(RM) motion_control_tests motion_control_tests.exe
