CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

all: buddy3_status_tests barcode_tests line_follow_tests

buddy3_status_tests: test_buddy3_status.c ../../app_program/buddy3_status.c ../../app_program/buddy3_status.h ../../app_program/line_follow.h ../../app_program/barcode.h ../../app_program/navigation.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_buddy3_status.c ../../app_program/buddy3_status.c

barcode_tests: test_barcode.c ../../app_program/barcode.c ../../app_program/barcode.h ../../app_program/navigation.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_barcode.c ../../app_program/barcode.c

line_follow_tests: test_line_follow.c ../../app_program/line_follow.c ../../app_program/line_follow.h ../../app_program/ir_sensor.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_line_follow.c ../../app_program/line_follow.c

test: buddy3_status_tests barcode_tests line_follow_tests
	./buddy3_status_tests
	./barcode_tests
	./line_follow_tests

clean:
	$(RM) buddy3_status_tests buddy3_status_tests.exe barcode_tests barcode_tests.exe line_follow_tests line_follow_tests.exe
