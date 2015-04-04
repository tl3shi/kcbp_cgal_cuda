#!/usr/bin/make -R

PROJECT_BASE ?= $(shell expr $(CURDIR) : '\(.*/CollDet[^/]*\).*' )


#--- packages / targets -------------------------------------------------------

REQUIRED_PACKAGES    := coll qhull OpenSG-Glut

DEFAULT_TARGETS	     := bench boxtree convexHull doptree exceptions grid \
						interactive intersect movem queue rollingball \
						topology utils multithreading

NO_AUTO_LIB         := 1

#--- include standard makefile includes ---------------------------------------

include $(PROJECT_BASE)/mak/common.mak
include $(PROJECT_BASE)/mak/subdir.mak

