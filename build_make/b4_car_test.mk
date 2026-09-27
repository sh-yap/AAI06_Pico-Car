################################################################################
# Buddy 4 ROBO-PICO diagnostic firmware makefile
################################################################################

EXE_FILE := b4_robopico_car_test
TARGET := _PICO_RP2040_
.DEFAULT_GOAL := all

SMP ?= 0
CONSOLE ?= usb_cdc
WIFI ?= none
B4_TEST_CALIBRATION ?= 0

# The target board is the ROBO-PICO car, so the shared kernel and driver
# sources select the robot board branches. That is what reserves PWM for
# Buddy 2 and leaves GP8 to GP11 to the motor driver. It also gives this
# image the shared I2C1 bus selection instead of a private copy of it.
BOARD := robo_pico_robot

ifneq ($(SMP),0)
$(error Buddy 4 car test requires SMP=0)
endif
ifneq ($(CONSOLE),usb_cdc)
$(error Buddy 4 car test requires CONSOLE=usb_cdc)
endif
ifneq ($(WIFI),none)
$(error Buddy 4 car test requires WIFI=none)
endif
ifneq ($(filter $(B4_TEST_CALIBRATION),0 1),$(B4_TEST_CALIBRATION))
$(error Unknown B4_TEST_CALIBRATION '$(B4_TEST_CALIBRATION)'; use 0 or 1)
endif

INCPATH = -I"../include" -I"../config" -I"../kernel/knlinc" \
          -I"../app_program" -I"../tests/buddy4"

# The profile includes the test-local calibration mode. The shared platform
# makefile accepts this profile identity without knowing about Buddy 4 tests.
PROFILE_ID := b4-car-test-cal$(B4_TEST_CALIBRATION)-$(CONSOLE)-$(WIFI)-smp$(SMP)
PROFILE_FILE := .build_profile

-include sources.mk

include mtkernel_3/lib/libtm/subdir.mk
include mtkernel_3/lib/libtk/subdir.mk
include mtkernel_3/kernel/usermain/subdir.mk
include mtkernel_3/kernel/tstdlib/subdir.mk
include mtkernel_3/kernel/tkernel/subdir.mk
include mtkernel_3/kernel/sysinit/subdir.mk
include mtkernel_3/kernel/inittask/subdir.mk
include mtkernel_3/device/subdir.mk
include pico_rp2040.mk

EXE_FILE := b4_robopico_car_test

# The shared platform file supplies the normal kernel and device objects. This
# diagnostic image deliberately adds only its Buddy 4 production modules and
# dedicated test modules. It never links app_main.c or demo_tasks.c.
B4_APP_SRCS := ../app_program/b4_cfg.c \
               ../app_program/b4_imu.c \
               ../app_program/b4_proc.c
B4_TEST_SRCS := ../tests/buddy4/b4_car_test.c \
                ../tests/buddy4/b4_car_test_log.c \
                ../tests/buddy4/b4_car_test_main.c
B4_APP_OBJS := $(patsubst ../app_program/%.c,mtkernel_3/app_program/%.o,$(B4_APP_SRCS))
B4_TEST_OBJS := $(patsubst ../tests/buddy4/%.c,mtkernel_3/tests/buddy4/%.o,$(B4_TEST_SRCS))
B4_OBJS := $(B4_APP_OBJS) $(B4_TEST_OBJS)
B4_DEPS := $(B4_OBJS:.o=.d)
OBJS += $(B4_OBJS)
C_DEPS += $(B4_DEPS)

CFLAGS += -DB4_TEST_CALIBRATION=$(B4_TEST_CALIBRATION) \
          -DB4_TEST_PROFILE=b4_robopico_car_test_cal$(B4_TEST_CALIBRATION)

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building Buddy 4 production file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/tests/buddy4/%.o: ../tests/buddy4/%.c
	@echo 'Building Buddy 4 test file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

-include $(C_DEPS)

all: $(EXE_FILE).elf

$(EXE_FILE).elf: $(OBJS) $(E2U)
	@echo 'Linker: $@'
	$(LINK) $(LFLAGS) -T $(LNKFILE) -Wl,-Map,"$(EXE_FILE).map" -o "$(EXE_FILE).elf" $(OBJS)
	@echo 'Finished building target: $@'
	$(SIZE) $(EXE_FILE).elf
	$(E2U) $(EXE_FILE).elf $(EXE_FILE).uf2

clean:
	-$(RM) $(OBJS) $(C_DEPS) $(ASM_DEPS) $(S_UPPER_DEPS) \
	        $(EXE_FILE).elf $(EXE_FILE).map $(EXE_FILE).uf2

.PHONY: all clean

#*** end of file ***
