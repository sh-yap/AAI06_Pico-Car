# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: run_rec_tests run_rec_b4_tests

run_rec_tests: test_run_rec.c $(APP_BUDDY1)/run_rec.c $(APP_BUDDY1)/run_rec.h
	$(CC) $(CFLAGS) -DVC_RECORDER=1 $(APP_INC) -o $@ test_run_rec.c $(APP_BUDDY1)/run_rec.c

run_rec_b4_tests: test_run_rec.c $(APP_BUDDY1)/run_rec.c $(APP_BUDDY1)/run_rec.h
	$(CC) $(CFLAGS) -DVC_RECORDER=1 -DB4_RECORDER=1 $(APP_INC) -o $@ test_run_rec.c $(APP_BUDDY1)/run_rec.c

test: run_rec_tests run_rec_b4_tests
	./run_rec_tests
	./run_rec_b4_tests

clean:
	$(RM) run_rec_tests run_rec_tests.exe run_rec_b4_tests run_rec_b4_tests.exe
