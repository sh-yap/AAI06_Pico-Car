# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: obstacle_calc_tests

obstacle_calc_tests: test_obstacle_calc.c $(APP_BUDDY5)/obstacle_calc.c $(APP_BUDDY5)/obstacle_calc.h $(APP_BUDDY5)/obstacle_monitor.c $(APP_BUDDY5)/obstacle_monitor.h $(APP_BUDDY5)/obstacle_scan.c $(APP_BUDDY5)/obstacle_scan.h $(APP_BUDDY5)/obstacle_plan.c $(APP_BUDDY5)/obstacle_plan.h $(APP_BUDDY5)/obstacle_pack.c $(APP_BUDDY5)/obstacle_pack.h $(APP_BUDDY5)/obstacle_config.h $(APP_BUDDY5)/obstacle_types.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_obstacle_calc.c $(APP_BUDDY5)/obstacle_calc.c $(APP_BUDDY5)/obstacle_monitor.c $(APP_BUDDY5)/obstacle_scan.c $(APP_BUDDY5)/obstacle_plan.c $(APP_BUDDY5)/obstacle_pack.c -lm

test: obstacle_calc_tests
	./obstacle_calc_tests

clean:
	$(RM) obstacle_calc_tests obstacle_calc_tests.exe
