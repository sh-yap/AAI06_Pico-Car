# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

# make's built-in default is "cc"; use gcc unless CC is set explicitly.
ifeq ($(origin CC),default)
CC = gcc
endif
# -s (strip): Windows Smart App Control blocked the unstripped test binary.
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2 -s

SRCS = test_codec.c $(APP_BUDDY1)/comms_codec.c
DEPS = $(SRCS) $(APP_BUDDY1)/comms_codec.h $(APP_BUDDY1)/comms_link.h

all: codec_tests codec_tests_json

# Binary format (the firmware default).
codec_tests: $(DEPS)
	$(CC) $(CFLAGS) $(APP_INC) -o $@ $(SRCS)

# JSON format (COMMS_FORMAT=json fallback).
codec_tests_json: $(DEPS)
	$(CC) $(CFLAGS) -DCOMMS_JSON=1 $(APP_INC) -o $@ $(SRCS)

test: codec_tests codec_tests_json
	./codec_tests
	./codec_tests_json

clean:
	$(RM) codec_tests codec_tests.exe codec_tests_json codec_tests_json.exe
