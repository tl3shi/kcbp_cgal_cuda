#!/usr/bin/make -R

PROJECT_BASE ?= $(shell expr $(CURDIR) : '\(.*/CollDet[^/]*\).*' )

#--- packages / targets -------------------------------------------------------

REQUIRED_PACKAGES	:= boost

DEFAULT_TARGETS		:= so linklib

EXCLUDED_SOURCES	:= 

#--- include standard makefile includes ---------------------------------------

include $(PROJECT_BASE)/mak/common.mak
include $(PROJECT_BASE)/mak/subdir.mak

#--- local settings -----------------------------------------------------------

ifeq "$(OS)" "cygwin11"
WARNINGS_OFF_CXX	+= -Qwd181,279,171
# 181 : argument is incompatible with corresponding format string conversion
# 279 : controlling expression is constant
# 171: invalid type conversion
else
ifeq "$(OS)" "irix65"
WARNINGS_OFF_CXX	+= -woff 1209,1552
# 1209: The controlling expression is constant
# 1552: The parameter "..." is set but never used
else
ifeq "$(OS)" "linux2"
WARNINGS_OFF_CXX	+= -Wno-float-equal
# no-float-equal: comparing floating point with == or != is unsafe
# Siehe info gcc 'Invoking GCC' 'Warning Options'
endif
endif
endif

