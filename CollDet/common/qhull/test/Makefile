#!gmake -R

PROJECT_BASE ?= $(shell expr $(CURDIR) : '\(.*/CollDet[^/]*\).*' )

#--- packages / targets -------------------------------------------------------

REQUIRED_PACKAGES	:= qhull

DEFAULT_TARGETS		:= unix rbox user_eg

NO_AUTO_LIB			:= 1

#--- include standard makefile includes ---------------------------------------

include $(PROJECT_BASE)/mak/common.mak
include $(PROJECT_BASE)/mak/subdir.mak

