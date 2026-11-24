# Application module folders: APP_BUDDYn, APP_COMMON and APP_INC.
APP_ROOT := ../../app_program
include $(APP_ROOT)/modules.mk

CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O2

all: motion_control_tests motion_task_tests motion_pack_tests motion_encoder_snapshot_tests motion_usb_isolation_tests motion_encoder_irq_tests motion_motor_tests

motion_motor_tests: test_motion_motor.c $(APP_BUDDY2)/motion_motor.c $(APP_BUDDY2)/motion_motor.h $(APP_BUDDY2)/motion_config.c $(APP_BUDDY2)/motion_config.h $(wildcard stubs/tk/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs $(APP_INC) -o $@ test_motion_motor.c $(APP_BUDDY2)/motion_config.c

motion_encoder_irq_tests: test_motion_encoder_irq.c $(APP_BUDDY2)/motion_encoder.c $(APP_BUDDY2)/motion_control.c $(APP_BUDDY2)/motion_config.c $(wildcard $(APP_BUDDY2)/motion_*.h stubs/tk/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs $(APP_INC) -o $@ test_motion_encoder_irq.c $(APP_BUDDY2)/motion_control.c $(APP_BUDDY2)/motion_config.c

motion_usb_isolation_tests: test_motion_usb_isolation.c $(APP_BUDDY2)/motion_closed_loop_button_test_main.c $(APP_BUDDY2)/motion_clock.h $(wildcard stubs/tk/*.h stubs/tm/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs $(APP_INC) -o $@ test_motion_usb_isolation.c

motion_task_tests: test_motion_task.c $(APP_BUDDY2)/motion_task.c $(APP_BUDDY2)/motion_pack.c $(APP_BUDDY2)/motion_types.h $(APP_BUDDY2)/motion_encoder.h $(APP_BUDDY2)/motion_clock.h $(APP_BUDDY2)/motion_control.c $(APP_BUDDY2)/motion_config.c $(wildcard stubs/tk/*.h stubs/tm/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs $(APP_INC) -o $@ test_motion_task.c $(APP_BUDDY2)/motion_control.c $(APP_BUDDY2)/motion_pack.c $(APP_BUDDY2)/motion_config.c -lm

motion_pack_tests: test_motion_pack.c $(APP_BUDDY2)/motion_pack.c $(APP_BUDDY2)/motion_types.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_motion_pack.c $(APP_BUDDY2)/motion_pack.c

motion_encoder_snapshot_tests: test_motion_encoder_snapshot.c $(APP_BUDDY2)/motion_encoder.c $(APP_BUDDY2)/motion_encoder.h $(APP_BUDDY2)/motion_clock.h $(APP_BUDDY2)/motion_config.c $(wildcard stubs/tk/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs $(APP_INC) -o $@ test_motion_encoder_snapshot.c $(APP_BUDDY2)/motion_config.c

motion_control_tests: test_motion_control.c $(APP_BUDDY2)/motion_control.c $(APP_BUDDY2)/motion_control.h $(APP_BUDDY2)/motion_config.c $(APP_BUDDY2)/motion_config.h $(APP_BUDDY2)/motion_stage4_profile.c $(APP_BUDDY2)/motion_stage4_profile.h
	$(CC) $(CFLAGS) $(APP_INC) -o $@ test_motion_control.c $(APP_BUDDY2)/motion_control.c $(APP_BUDDY2)/motion_config.c $(APP_BUDDY2)/motion_stage4_profile.c -lm

test: all
	./motion_control_tests
	./motion_task_tests
	./motion_pack_tests
	./motion_encoder_snapshot_tests
	./motion_usb_isolation_tests
	./motion_encoder_irq_tests
	./motion_motor_tests

clean:
	$(RM) motion_encoder_irq_tests motion_encoder_irq_tests.exe
	$(RM) motion_control_tests motion_control_tests.exe
	$(RM) motion_task_tests motion_task_tests.exe
	$(RM) motion_pack_tests motion_pack_tests.exe
	$(RM) motion_encoder_snapshot_tests motion_encoder_snapshot_tests.exe
	$(RM) motion_usb_isolation_tests motion_usb_isolation_tests.exe
	$(RM) motion_motor_tests motion_motor_tests.exe
