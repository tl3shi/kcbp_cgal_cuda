#!/usr/bin/make -R

PROJECT_BASE ?= $(shell expr $(CURDIR) : '\(.*/CollDet[^/]*\).*' )

#--- packages / targets -------------------------------------------------------

REQUIRED_PACKAGES	:= OpenSG boost qhull

DEFAULT_TARGETS		:= so linklib
#DEFAULT_TARGETS		:= lib

#--- include standard makefile includes ---------------------------------------

include $(PROJECT_BASE)/mak/common.mak
include $(PROJECT_BASE)/mak/subdir.mak
include $(PROJECT_BASE)/mak/doc.mak

