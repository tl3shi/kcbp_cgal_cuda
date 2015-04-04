#! @file
#
#  @brief
#  makefile include
#

#QTPOOL			= $(BASEDIRABSOLUTE)/../common/qt
QTPOOL			= /usr/lib/qt2
QTPOOLBIN       = $(QTPOOL)/bin

MOC				:= $(QTPOOLBIN)/moc
UIC				:= $(QTPOOLBIN)/uic

INCDIRS_CXX		+= $(QTPOOL)/include
#LIBDIRS			+= $(QTPOOL)/lib/$(OS)
LIBDIRS			+= $(QTPOOL)/lib
ifeq "$(OS)" "cygwin11"
# The following flags could be set in qconfig.h. We do this here
# because we whant to use the same header for all operating systems
FLAGS_CXX		+= -DQT_NO_IMAGEIO_MNG -DQT_DLL
LIBS_POST1		+=  qt223 Imm32
else
LIBS_POST1		+=  qt
endif


