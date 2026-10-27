################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# The original tree discovered every C file in app_program.  That is unsafe
# once test applications are added because each profile would contain several
# usermain() definitions.  Keep shared modules automatic, and select exactly
# one entry source.
ifeq ($(APP),demo)
APP_ENTRY_SRC := ../app_program/app_main.c
else ifeq ($(APP),motion_sim_test)
APP_ENTRY_SRC := ../app_program/motion_sim_test_main.c
else ifeq ($(APP),motion_motor_test)
APP_ENTRY_SRC := ../app_program/motion_motor_test_main.c
else ifeq ($(APP),motion_encoder_test)
APP_ENTRY_SRC := ../app_program/motion_encoder_test_main.c
else ifeq ($(APP),motion_closed_loop_test)
APP_ENTRY_SRC := ../app_program/motion_closed_loop_test_main.c
else ifeq ($(APP),ultrasonic_test)
APP_ENTRY_SRC := ../app_program/ultrasonic_test_main.c
else ifeq ($(APP),servo_test)
APP_ENTRY_SRC := ../app_program/servo_test_main.c
else ifeq ($(APP),obstacle_task_test)
APP_ENTRY_SRC := ../app_program/obstacle_task_test_main.c
endif

TEMP_ALL_SRCS := $(wildcard ../app_program/*.c)
TEMP_COMMON_SRCS := $(filter-out ../app_program/app_main.c ../app_program/motion_sim_test_main.c ../app_program/motion_motor_test_main.c ../app_program/motion_encoder_test_main.c ../app_program/motion_closed_loop_test_main.c ../app_program/ultrasonic_test_main.c ../app_program/servo_test_main.c ../app_program/obstacle_task_test_main.c,$(TEMP_ALL_SRCS))
TEMP_SRCS = $(TEMP_COMMON_SRCS) $(APP_ENTRY_SRC)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -Wall -Wextra -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
