################################################################################
# Application module folders, one per buddy plus shared code.
#
# The single list of app_program sub-folders, used by the firmware build
# (build_make/mtkernel_3/app_program/subdir.mk, build_make/b4_car_test.mk) and
# by the host tests (tests/*/Makefile).  Add a folder here and every build
# finds its sources and headers.
#
# Set APP_ROOT to the path of app_program as seen from the including makefile
# before including this file.  Sources keep plain #include "x.h": every module
# folder is on the include path (APP_INC).
################################################################################

APP_ROOT ?= ../app_program

# One folder per module, named for use in source lists.
APP_COMMON := $(APP_ROOT)/common
APP_BUDDY1 := $(APP_ROOT)/buddy1_vehicle_comms
APP_BUDDY2 := $(APP_ROOT)/buddy2_motion
APP_BUDDY3 := $(APP_ROOT)/buddy3_line_barcode
APP_BUDDY4 := $(APP_ROOT)/buddy4_imu_terrain
APP_BUDDY5 := $(APP_ROOT)/buddy5_obstacle

APP_SRC_DIRS := $(APP_COMMON) $(APP_BUDDY1) $(APP_BUDDY2) $(APP_BUDDY3) \
                $(APP_BUDDY4) $(APP_BUDDY5)
APP_INC      := $(foreach dir,$(APP_SRC_DIRS),-I"$(dir)")
