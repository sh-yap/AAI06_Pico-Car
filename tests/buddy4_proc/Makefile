# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2
SRC = $(APP_BUDDY4)/b4_proc.c $(APP_BUDDY4)/b4_pack.c \
      $(APP_BUDDY4)/b4_cfg.c

all: b4_proc_tests b4_pack_tests

b4_proc_tests: test_b4_proc.c $(SRC)
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_b4_proc.c $(SRC) -lm

b4_pack_tests: test_b4_pack.c $(APP_BUDDY4)/b4_pack.c \
               $(APP_BUDDY4)/b4_pack.h $(APP_BUDDY4)/b4_types.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_b4_pack.c \
	    $(APP_BUDDY4)/b4_pack.c

test: all
	./b4_pack_tests
	./b4_proc_tests

clean:
	$(RM) b4_proc_tests b4_proc_tests.exe b4_pack_tests b4_pack_tests.exe
