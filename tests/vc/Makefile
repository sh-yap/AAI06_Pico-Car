CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: vc_tests vc_pack_tests

vc_tests: test_vc.c ../../app_program/vc.c ../../app_program/vc.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_vc.c ../../app_program/vc.c

vc_pack_tests: test_vc_pack.c ../../app_program/vc_pack.c ../../app_program/vc_pack.h ../../app_program/vc.c ../../app_program/vc.h ../../app_program/obstacle_plan.c ../../app_program/obstacle_plan.h ../../app_program/obstacle_types.h ../../app_program/obstacle_config.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_vc_pack.c ../../app_program/vc_pack.c ../../app_program/vc.c ../../app_program/obstacle_plan.c

test: all
	./vc_tests
	./vc_pack_tests

clean:
	$(RM) vc_tests vc_tests.exe vc_pack_tests vc_pack_tests.exe
