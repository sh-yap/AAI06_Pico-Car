################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# Application sources live in one folder per buddy (app_program/modules.mk).
# Every module folder is on the include path, so sources keep plain
# #include "x.h" across folders.
APP_ROOT := ../app_program
include $(APP_ROOT)/modules.mk
INCPATH += $(APP_INC)

# The original tree discovered every C file in app_program.  That is unsafe
# once test applications are added because each profile would contain several
# usermain() definitions.  Keep shared modules automatic, and select exactly
# one entry source.
APP_ENTRY_SRCS := $(APP_COMMON)/app_main.c \
                  $(APP_BUDDY2)/motion_sim_test_main.c \
                  $(APP_BUDDY2)/motion_motor_test_main.c \
                  $(APP_BUDDY2)/motion_encoder_test_main.c \
                  $(APP_BUDDY2)/motion_clock_test_main.c \
                  $(APP_BUDDY2)/motion_closed_loop_test_main.c \
                  $(APP_BUDDY2)/motion_closed_loop_button_test_main.c \
                  $(APP_BUDDY2)/motion_stage4_test_main.c \
                  $(APP_BUDDY2)/motion_stage5_test_main.c \
                  $(APP_BUDDY2)/motion_stage6_test_main.c \
                  $(APP_BUDDY2)/motion_stage7_test_main.c \
                  $(APP_BUDDY2)/motion_route_test_main.c \
                  $(APP_BUDDY5)/ultrasonic_test_main.c \
                  $(APP_BUDDY5)/servo_test_main.c \
                  $(APP_BUDDY5)/obstacle_task_test_main.c \
                  $(APP_BUDDY1)/vc_car_main.c

ifeq ($(APP),demo)
APP_ENTRY_SRC := $(APP_COMMON)/app_main.c
else ifeq ($(APP),motion_sim_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_sim_test_main.c
else ifeq ($(APP),motion_motor_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_motor_test_main.c
else ifeq ($(APP),motion_encoder_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_encoder_test_main.c
else ifeq ($(APP),motion_clock_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_clock_test_main.c
else ifeq ($(APP),motion_closed_loop_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_closed_loop_test_main.c
else ifeq ($(APP),motion_stage4_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_stage4_test_main.c
else ifeq ($(APP),motion_stage5_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_stage5_test_main.c
else ifeq ($(APP),motion_stage6_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_stage6_test_main.c
else ifeq ($(APP),motion_stage7_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_stage7_test_main.c
else ifeq ($(APP),motion_route_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_route_test_main.c
else ifeq ($(APP),motion_closed_loop_button_test)
APP_ENTRY_SRC := $(APP_BUDDY2)/motion_closed_loop_button_test_main.c
else ifeq ($(APP),ultrasonic_test)
APP_ENTRY_SRC := $(APP_BUDDY5)/ultrasonic_test_main.c
else ifeq ($(APP),vc_car)
APP_ENTRY_SRC := $(APP_BUDDY1)/vc_car_main.c
else ifeq ($(APP),servo_test)
APP_ENTRY_SRC := $(APP_BUDDY5)/servo_test_main.c
else ifeq ($(APP),obstacle_task_test)
APP_ENTRY_SRC := $(APP_BUDDY5)/obstacle_task_test_main.c
endif

TEMP_ALL_SRCS := $(foreach dir,$(APP_SRC_DIRS),$(wildcard $(dir)/*.c))
TEMP_COMMON_SRCS := $(filter-out $(APP_ENTRY_SRCS),$(TEMP_ALL_SRCS))
TEMP_SRCS = $(TEMP_COMMON_SRCS) $(APP_ENTRY_SRC)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# % spans the module sub-folder, so one rule builds every module.
mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -Wall -Wextra -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
