# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: vc_tests vc_pack_tests

vc_tests: test_vc.c $(APP_BUDDY1)/vc.c $(APP_BUDDY1)/vc.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_vc.c $(APP_BUDDY1)/vc.c

vc_pack_tests: test_vc_pack.c $(APP_BUDDY1)/vc_pack.c $(APP_BUDDY1)/vc_pack.h $(APP_BUDDY1)/vc.c $(APP_BUDDY1)/vc.h $(APP_BUDDY5)/obstacle_plan.c $(APP_BUDDY5)/obstacle_plan.h $(APP_BUDDY5)/obstacle_types.h $(APP_BUDDY5)/obstacle_config.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_vc_pack.c $(APP_BUDDY1)/vc_pack.c $(APP_BUDDY1)/vc.c $(APP_BUDDY5)/obstacle_plan.c

test: all
	./vc_tests
	./vc_pack_tests

clean:
	$(RM) vc_tests vc_tests.exe vc_pack_tests vc_pack_tests.exe
