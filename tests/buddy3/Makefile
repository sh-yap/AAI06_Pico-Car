# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: buddy3_status_tests barcode_tests nav_map_tests line_follow_tests

buddy3_status_tests: test_buddy3_status.c $(APP_BUDDY3)/buddy3_status.c $(APP_BUDDY3)/buddy3_status.h $(APP_BUDDY3)/line_follow.h $(APP_BUDDY3)/barcode.h $(APP_BUDDY3)/navigation.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_buddy3_status.c $(APP_BUDDY3)/buddy3_status.c

barcode_tests: test_barcode.c $(APP_BUDDY3)/barcode.c $(APP_BUDDY3)/barcode.h $(APP_BUDDY3)/nav_map.c $(APP_BUDDY3)/nav_map.h $(APP_BUDDY3)/navigation.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_barcode.c $(APP_BUDDY3)/barcode.c $(APP_BUDDY3)/nav_map.c

nav_map_tests: test_nav_map.c $(APP_BUDDY3)/nav_map.c $(APP_BUDDY3)/nav_map.h $(APP_BUDDY3)/navigation.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_nav_map.c $(APP_BUDDY3)/nav_map.c

line_follow_tests: test_line_follow.c $(APP_BUDDY3)/line_follow.c $(APP_BUDDY2)/motion_config.c $(APP_BUDDY2)/motion_config.h $(APP_BUDDY3)/line_follow.h $(APP_BUDDY3)/ir_sensor.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_line_follow.c $(APP_BUDDY3)/line_follow.c $(APP_BUDDY2)/motion_config.c

test: buddy3_status_tests barcode_tests nav_map_tests line_follow_tests
	./buddy3_status_tests
	./barcode_tests
	./nav_map_tests
	./line_follow_tests

clean:
	$(RM) buddy3_status_tests buddy3_status_tests.exe barcode_tests barcode_tests.exe nav_map_tests nav_map_tests.exe line_follow_tests line_follow_tests.exe
