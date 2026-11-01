CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O2

all: motion_control_tests motion_task_tests motion_encoder_snapshot_tests motion_usb_isolation_tests motion_encoder_irq_tests motion_motor_tests

motion_motor_tests: test_motion_motor.c ../../app_program/motion_motor.c ../../app_program/motion_motor.h ../../app_program/motion_config.c ../../app_program/motion_config.h $(wildcard stubs/tk/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs -I../../app_program -o $@ test_motion_motor.c ../../app_program/motion_config.c

motion_encoder_irq_tests: test_motion_encoder_irq.c ../../app_program/motion_encoder.c ../../app_program/motion_control.c ../../app_program/motion_config.c $(wildcard ../../app_program/motion_*.h stubs/tk/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs -I../../app_program -o $@ test_motion_encoder_irq.c ../../app_program/motion_control.c ../../app_program/motion_config.c

motion_usb_isolation_tests: test_motion_usb_isolation.c ../../app_program/motion_closed_loop_button_test_main.c ../../app_program/motion_clock.h $(wildcard stubs/tk/*.h stubs/tm/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs -I../../app_program -o $@ test_motion_usb_isolation.c

motion_task_tests: test_motion_task.c ../../app_program/motion_task.c ../../app_program/motion_types.h ../../app_program/motion_encoder.h ../../app_program/motion_clock.h ../../app_program/motion_control.c ../../app_program/motion_config.c $(wildcard stubs/tk/*.h stubs/tm/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs -I../../app_program -o $@ test_motion_task.c ../../app_program/motion_control.c ../../app_program/motion_config.c -lm

motion_encoder_snapshot_tests: test_motion_encoder_snapshot.c ../../app_program/motion_encoder.c ../../app_program/motion_encoder.h ../../app_program/motion_clock.h ../../app_program/motion_config.c $(wildcard stubs/tk/*.h stubs/bsp/*.h stubs/sys/*.h stubs/sys/sysdepend/cpu/rp2040/*.h)
	$(CC) $(CFLAGS) -Istubs -I../../app_program -o $@ test_motion_encoder_snapshot.c ../../app_program/motion_config.c

motion_control_tests: test_motion_control.c ../../app_program/motion_control.c ../../app_program/motion_control.h ../../app_program/motion_config.c ../../app_program/motion_config.h ../../app_program/motion_stage4_profile.c ../../app_program/motion_stage4_profile.h
	$(CC) $(CFLAGS) -I../../app_program -o $@ test_motion_control.c ../../app_program/motion_control.c ../../app_program/motion_config.c ../../app_program/motion_stage4_profile.c -lm

test: all
	./motion_control_tests
	./motion_task_tests
	./motion_encoder_snapshot_tests
	./motion_usb_isolation_tests
	./motion_encoder_irq_tests
	./motion_motor_tests

clean:
	$(RM) motion_encoder_irq_tests motion_encoder_irq_tests.exe
	$(RM) motion_control_tests motion_control_tests.exe
	$(RM) motion_task_tests motion_task_tests.exe
	$(RM) motion_encoder_snapshot_tests motion_encoder_snapshot_tests.exe
	$(RM) motion_usb_isolation_tests motion_usb_isolation_tests.exe
	$(RM) motion_motor_tests motion_motor_tests.exe
