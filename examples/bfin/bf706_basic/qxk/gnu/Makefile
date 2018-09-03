##############################################################################
# Product: Makefile for DPP on EMF32-SLSTK3401A, QXK kernel, GNU-BFIN
# Last Updated for Version: 6.1.0
# Date of the Last Update:  2018-02-07
#
#                    Q u a n t u m     L e a P s
#                    ---------------------------
#                    innovating embedded systems
#
# Copyright (C) 2005-2018 Quantum Leaps, LLC. All rights reserved.
#
# This program is open source software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Alternatively, this program may be distributed and modified under the
# terms of Quantum Leaps commercial licenses, which expressly supersede
# the GNU General Public License and are specifically designed for
# licensees interested in retaining the proprietary status of their code.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Contact information:
# https://www.state-machine.com
# mailto:info@state-machine.com
##############################################################################
# examples of invoking this Makefile:
# building configurations: Debug (default), Release, and Spy
# make
# make CONF=rel
# make CONF=spy
#
# cleaning configurations: Debug (default), Release, and Spy
# make clean
# make CONF=rel clean
# make CONF=spy clean
#
# NOTE:
# To use this Makefile on Windows, you will need the GNU make utility, which
# is included in the Qtools collection for Windows, see:
#    http://sourceforge.net/projects/qpc/files/Qtools/
#

#-----------------------------------------------------------------------------
# project name
#
PROJECT     := bf706-qxk

#-----------------------------------------------------------------------------
# project directories
#

# location of the QP/C++ framework (if not provided in an environemnt var.)
ifeq ($(QPCPP),)
QPCPP := ../../../../..
endif

# QP port used in this project
QP_PORT_DIR := $(QPCPP)/ports/bfin/qxk/gnu

# list of all source directories used by this project
VPATH = \
	.. \
	../.. \
	$(QPCPP)/src/qf \
	$(QPCPP)/src/qxk \
	$(QPCPP)/src/qs \
	$(QP_PORT_DIR) \
	$(QPCPP)/3rd_party/bfin \

# list of all include directories needed by this project
INCLUDES  = \
	-I../.. \
	-I$(QPCPP)/include \
	-I$(QPCPP)/src \
	-I$(QP_PORT_DIR) \
	-I$(QPCPP)/3rd_party/bfin-CMSIS \
	-I$(QPCPP)/3rd_party/bfin

#-----------------------------------------------------------------------------
# files
#

# assembler source files
ASM_SRCS := \
	app_start.S
# C source files
C_SRCS := \
	handlers.c

# C++ source files
CPP_SRCS :=	\
	fw_evt.cpp \
	bsp.cpp \
	main.cpp \
	System.cpp

OUTPUT    := $(PROJECT)
LD_SCRIPT := $(PROJECT).ld

QP_SRCS := \
	qep_hsm.cpp \
	qep_msm.cpp \
	qf_act.cpp \
	qf_actq.cpp \
	qf_defer.cpp \
	qf_dyn.cpp \
	qf_mem.cpp \
	qf_ps.cpp \
	qf_qact.cpp \
	qf_qeq.cpp \
	qf_qmact.cpp \
	qf_time.cpp \
	qxk.cpp \
	qxk_mutex.cpp \
	qxk_sema.cpp \
	qxk_xthr.cpp \
	qxk_port.cpp

QP_ASMS :=

QS_SRCS := \
	qs.cpp \
	qs_rx.cpp \
	qs_fp.cpp

LIB_DIRS  :=
LIBS      :=

# defines
DEFINES   := -D__BF70x__

BFIN_CPU   := -mcpu=bf706

#-----------------------------------------------------------------------------
# GNU-BFIN toolset (NOTE: You need to adjust to your machine)
#
ifeq ($(GNU_BFIN),)
GNU_BFIN := C:/cygwin64/bfin-gnu-new/buildscript/bfin-elf-win
endif

# make sure that the GNU-BFIN toolset exists...
ifeq ("$(wildcard $(GNU_BFIN))","")
$(error GNU_BFIN toolset not found. Please adjust the Makefile)
endif

CC    := $(GNU_BFIN)/bin/bfin-elf-gcc
CPP   := $(GNU_BFIN)/bin/bfin-elf-g++
AS    := $(GNU_BFIN)/bin/bfin-elf-as
LINK  := $(GNU_BFIN)/bin/bfin-elf-gcc
BIN   := $(GNU_BFIN)/bin/bfin-elf-objcopy

##############################################################################
# Typically, you should not need to change anything below this line

# basic utilities (included in Qtools for Windows), see:
#    http://sourceforge.net/projects/qpc/files/Qtools

MKDIR := mkdir
RM    := rm

#-----------------------------------------------------------------------------
# build options for various configurations
#

# combine all the soruces...
CPP_SRCS += $(QP_SRCS)
ASM_SRCS += $(QP_ASMS)

ifeq (rel, $(CONF)) # Release configuration ..................................

BIN_DIR := rel

ASFLAGS = $(BFIN_CPU) $(ASM_CPU) -x assembler-with-cpp -c \
	$(INCLUDES) $(DEFINES)

CFLAGS = -c $(BFIN_CPU) -Wall \
	-ffunction-sections -fdata-sections \
	-O1 $(INCLUDES) $(DEFINES) -DNDEBUG

CPPFLAGS = -c $(BFIN_CPU) -Wall \
	-ffunction-sections -fdata-sections -fno-threadsafe-statics -fno-rtti -fno-exceptions \
	-O1 $(INCLUDES) $(DEFINES) -DNDEBUG

else ifeq (spy, $(CONF))  # Spy configuration ................................

BIN_DIR := spy

CPP_SRCS += $(QS_SRCS)

ASFLAGS = -g $(BFIN_CPU) $(ASM_CPU) -x assembler-with-cpp -c \
	$(INCLUDES) $(DEFINES)

CFLAGS = -c -g $(BFIN_CPU) -Wall \
	-ffunction-sections -fdata-sections \
	-O $(INCLUDES) $(DEFINES) -DQ_SPY

CPPFLAGS = -c -g $(BFIN_CPU) -Wall \
	-ffunction-sections -fdata-sections -fno-rtti -fno-exceptions \
	-O $(INCLUDES) $(DEFINES) -DQ_SPY

else # default Debug configuration ..........................................

BIN_DIR := dbg

ASFLAGS = -g $(BFIN_CPU) $(ASM_CPU) -x assembler-with-cpp -c \
	$(INCLUDES) $(DEFINES)

CFLAGS = -c -g $(BFIN_CPU) -Wall \
	-ffunction-sections -fdata-sections \
	-nostdlib \
	-O $(INCLUDES) $(DEFINES)

CPPFLAGS = -c -g $(BFIN_CPU) -Wall \
	-ffunction-sections -fdata-sections -fno-rtti -fno-exceptions \
	-fno-threadsafe-statics -nostdlib \
	-O $(INCLUDES) $(DEFINES)

endif # ......................................................................


LINKFLAGS = -T$(LD_SCRIPT) $(BFIN_CPU) \
	-Wl,-Map,$(BIN_DIR)/$(OUTPUT).map,--cref --specs=nosys.specs $(LIB_DIRS)

ASM_OBJS     := $(patsubst %.S,%.o,  $(notdir $(ASM_SRCS)))
C_OBJS       := $(patsubst %.c,%.o,  $(notdir $(C_SRCS)))
CPP_OBJS     := $(patsubst %.cpp,%.o,$(notdir $(CPP_SRCS)))

TARGET_BIN   := $(BIN_DIR)/$(OUTPUT).bin
TARGET_ELF   := $(BIN_DIR)/$(OUTPUT).elf
ASM_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(ASM_OBJS))
C_OBJS_EXT   := $(addprefix $(BIN_DIR)/, $(C_OBJS))
C_DEPS_EXT   := $(patsubst %.o, %.d, $(C_OBJS_EXT))
CPP_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(CPP_OBJS))
CPP_DEPS_EXT := $(patsubst %.o, %.d, $(CPP_OBJS_EXT))

# create $(BIN_DIR) if it does not exist
ifeq ("$(wildcard $(BIN_DIR))","")
$(shell $(MKDIR) $(BIN_DIR))
endif

#-----------------------------------------------------------------------------
# rules
#

all: $(TARGET_BIN)
#all: $(TARGET_ELF)

$(TARGET_BIN): $(TARGET_ELF)
	$(BIN) -O binary $< $@

$(TARGET_ELF) : $(ASM_OBJS_EXT) $(C_OBJS_EXT) $(CPP_OBJS_EXT)
	$(CPP) $(CPPFLAGS) -c $(QPCPP)/include/qstamp.cpp -o $(BIN_DIR)/qstamp.o
	$(LINK) $(LINKFLAGS) -o $@ $^ $(BIN_DIR)/qstamp.o $(LIBS)

$(BIN_DIR)/%.d : %.S
	$(CC) -MM -MT $(@:.d=.o) $(ASFLAGS) $< > $@

$(BIN_DIR)/%.d : %.c
	$(CC) -MM -MT $(@:.d=.o) $(CFLAGS) $< > $@

$(BIN_DIR)/%.d : %.cpp
	$(CPP) -MM -MT $(@:.d=.o) $(CPPFLAGS) $< > $@

$(BIN_DIR)/%.o : %.S
	$(CC) $(ASFLAGS) $< -o $@

$(BIN_DIR)/%.o : %.c
	$(CC) $(CFLAGS) $< -o $@

$(BIN_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) $< -o $@

# include dependency files only if our goal depends on their existence
ifneq ($(MAKECMDGOALS),clean)
  ifneq ($(MAKECMDGOALS),show)
-include $(C_DEPS_EXT) $(CPP_DEPS_EXT)
  endif
endif


.PHONY : clean
clean:
	-$(RM) $(BIN_DIR)/*.o \
	$(BIN_DIR)/*.d \
	$(BIN_DIR)/*.bin \
	$(BIN_DIR)/*.elf \
	$(BIN_DIR)/*.map
	
show:
	@echo PROJECT = $(PROJECT)
	@echo CONF = $(CONF)
	@echo DEFINES = $(DEFINES)
	@echo ASM_SRCS = $(ASM_SRCS)
	@echo C_SRCS = $(C_SRCS)
	@echo CPP_SRCS = $(CPP_SRCS)
	@echo ASM_OBJS_EXT = $(ASM_OBJS_EXT)
	@echo C_OBJS_EXT = $(C_OBJS_EXT)
	@echo C_DEPS_EXT = $(C_DEPS_EXT)
	@echo CPP_DEPS_EXT = $(CPP_DEPS_EXT)
	@echo TARGET_ELF = $(TARGET_ELF)
