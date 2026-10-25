CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: obstacle_calc_tests

obstacle_calc_tests: test_obstacle_calc.c ../../app_program/obstacle_calc.c ../../app_program/obstacle_calc.h ../../app_program/obstacle_monitor.c ../../app_program/obstacle_monitor.h ../../app_program/obstacle_config.h ../../app_program/obstacle_types.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_obstacle_calc.c ../../app_program/obstacle_calc.c ../../app_program/obstacle_monitor.c

test: obstacle_calc_tests
	./obstacle_calc_tests

clean:
	$(RM) obstacle_calc_tests obstacle_calc_tests.exe
