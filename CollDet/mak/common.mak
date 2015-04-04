#! @file
#
#  @brief
#  Standard include for all Makefiles
#
#  This file must be included in all makefiles.
#
#  @warning
#  Dieser File muss *vor* subdir.mak oder toplevel.mak included werden!
#


###############################################################################
# Set basic directories
###############################################################################

BASEDIRABSOLUTE			?= $(PROJECT_BASE)

ifeq "$(CURDIR)" "$(BASEDIRABSOLUTE)"
CURRENTDIRRELFROMBASE	:= .
BASEDIRRELATIVE			:= .
else
CURRENTDIRRELFROMBASE	:= $(subst $(BASEDIRABSOLUTE)/,,$(CURDIR))
BASEDIRRELATIVE			:= $(shell echo $(CURRENTDIRRELFROMBASE) \
								| sed -e 's:[^/][^/]*:\.\.:g' )
endif

BASEDIR					:= $(BASEDIRRELATIVE)
CURRENTDIR				:= $(notdir $(CURDIR))

MAKDIR					:= $(BASEDIR)/mak
MAKDIR_ABS				:= $(BASEDIRABSOLUTE)/mak

# make recursive makes a little faster
export PROJECT_BASE BASEDIRABSOLUTE

###############################################################################
# Set shell
# Always use bash, since its available on all platforms
###############################################################################

SHELL	:= /bin/bash --noprofile --norc --noediting

###############################################################################
# Determine OS type, processor and domain
###############################################################################

ifeq "$(MAKELEVEL)" "0"
# do this only once

OS						:= $(strip $(shell $(MAKDIR)/getos.sh))
PROCESSOR				:= $(shell uname -m)
export OS PROCESSOR

endif

###############################################################################
# Add some command-line options via MAKEFLAGS
###############################################################################

MAKEFLAGS				+= -r -R

# -r or --no-builtin-rules:
#	Eliminate use of the built-in implicit rules.
#
# -R or --no-builtin-variables:
#	Eliminate use of the built-in rule-specific variables.

.SUFFIXES:
# removes all built-in rules that are implemented as suffix rules

###############################################################################
# Check version of GNU make
###############################################################################

ifeq "$(NO_MAKE_VERSION)" ""
ifeq "$(MAKELEVEL)" "0"

MAKE_VERSION_REQ		:= 3.79

ifneq "$(shell expr $(MAKE_VERSION) \>= $(MAKE_VERSION_REQ))" "1"
    $(error Please use GNU make version $(MAKE_VERSION_REQ) or later - \
			$(MAKE) reported version $(MAKE_VERSION)!)
endif

endif
endif

###############################################################################
# Determine compiler
###############################################################################

# check the COMPILER environment variable
ifneq ($(COMPILER),)
COMPILER				:= $(firstword $(COMPILER))
endif

# g++ is same as gcc
ifeq "$(COMPILER)" "g++"
COMPILER				:= gcc
endif

# set default compiler if not yet set
ifeq "$(COMPILER)" ""

ifeq "$(OS)" "irix65"					# IRIX 6.5
COMPILER				:= CC
else
ifeq "$(OS)" "hpux11"					# HP-UX 11.x
COMPILER				:= aCC
else
ifeq "$(OS)" "cygwin11"				# Cygwin
COMPILER				:= icl
else								# other systems
COMPILER				:= gcc
endif
endif
endif

endif

# make recursive makes a little faster
export COMPILER

###############################################################################
# Determine platform (OS and compiler)
###############################################################################

# Note: OSCOMP and PLATFORM are used as synonyms

OSCOMP					:= $(OS)_$(COMPILER)
PLATFORM				:= $(OSCOMP)

###############################################################################
# Set directories and filenames
###############################################################################

# mark that we are included (this is cross-checked in subdir.mak, toplevel.mak)
HAVE_COMMON_MAK			= 1

PLATFORMINCLUDE			:= $(MAKDIR)/$(PLATFORM).mak

TOOLSDIR				:= $(BASEDIR)/internal/Tools
TOOLSDIR_OS				:= $(TOOLSDIR)/$(OS)

DEBUG_MARK				= dbg
OPTIM_MARK				= opt
LASTDEBUG_FILE			= .lastdebug_$(PLATFORM)

OBJDIRBASE				= obj
OBJDIR					= $(OBJDIRBASE)/$(PLATFORM)_$(DEBUG)
OBJDIRDEBUG				= $(OBJDIRBASE)/$(PLATFORM)_$(DEBUG_MARK)
OBJDIROPTIM				= $(OBJDIRBASE)/$(PLATFORM)_$(OPTIM_MARK)

SCRATCHDIR				= $(BASEDIRABSOLUTE)/scratch
SCRATCHOBJDIRBASE		= $(SCRATCHDIR)/$(PLATFORM)_$(DEBUG)
SCRATCHOBJDIRDEBUGBASE	= $(SCRATCHDIR)/$(PLATFORM)_$(DEBUG_MARK)
SCRATCHOBJDIROPTIMBASE	= $(SCRATCHDIR)/$(PLATFORM)_$(OPTIM_MARK)
SCRATCHOBJDIR			= $(SCRATCHOBJDIRBASE)/$(CURRENTDIRRELFROMBASE)
SCRATCHOBJDIRDEBUG		= $(SCRATCHOBJDIRDEBUGBASE)/$(CURRENTDIRRELFROMBASE)
SCRATCHOBJDIROPTIM		= $(SCRATCHOBJDIROPTIMBASE)/$(CURRENTDIRRELFROMBASE)

###############################################################################
# Set defaults for suffixes
###############################################################################

# These are UNIX defaults. They may be modified in platform specific includes.
# Will be redefined by cygwin*.mak.

EXESUFFIX			=
DEPSUFFIX			= .d
OBJSUFFIX			= .o
LIBPREFIX			= lib
LIBSUFFIX			= a
SOPREFIX			= lib
SOSUFFIX			= .so

CC_OUT_OPT			= -o # the space is important! [GZ]
LD_OUT_OPT			= -o # the space is important! [GZ]
AR_OUT_OPT			=

BISONCXXFLAGS		:=
FLEXCXXFLAGS		:=

MOC						= echo
UIC						= echo
# These are dummies for the Qt binaries in case qtxx.mak is *not* included

###############################################################################
# Set installation directories
###############################################################################

# Directory structure:
# 
# bin					- global scripts
# bin/<os>				- global binaries
# lib/<os>				- global shared objects
# <subsys>/bin/<os>		- subsystem binaries
# <subsys>/lib/<os>		- subsystem shared objects
# <subsys>/plugins/<os>	- subsystem shared objects for GUI
# <subsys>/include		- subsystem include files (developer release only)
# <subsys>/Data    		- subsystem data    files
# <subsys>/Documentation- subsystem doc     files
# <subsys>/Examples		- subsystem example files
# conf/<subsys>			- subsystem config  files

# Note: <os>=$(INST_OS) is different from $(OS)!
#
#		<os> : run-time variable (set by install/bin/platform script which is
#								  also used in application scripts)
#		$(OS): compile-time variable to distinguish different builds

INSTDIR					= $(BASEDIR)/install
INST_OS					:= $(OS)

BINDIR					= $(INSTDIR)/bin/$(INST_OS)
LIBDIR					= $(INSTDIR)/lib/$(INST_OS)
INCDIR					= $(INSTDIR)/include
DATDIR					= $(INSTDIR)/Data
DOCDIR					= $(INSTDIR)/Documentation
EXADIR					= $(INSTDIR)/Examples
CNFDIR					= $(INSTDIR)/conf

INSTDIRS				= $(BINDIR)		\
						  $(LIBDIR)		\
						  $(INCDIR)		\
						  $(DATDIR)		\
						  $(DOCDIR)		\
						  $(EXADIR)		\
						  $(CNFDIR)

II_FILESDIR				:= 

###############################################################################
# Get last compilation state (debug/optim) and set DEBUG flag
###############################################################################

# if we are called from parent dir we use the given default for DEBUG

ifeq "$(DEBUG)" ""

-include $(LASTDEBUG_FILE)

ifeq "$(DEBUG)" ""
DEBUG					= $(DEBUG_MARK)
endif

endif

export DEBUG

###############################################################################
# Default Targets (must be first target!)
###############################################################################

.PHONY: default_target

default_target:	default

###############################################################################
# General optimizations
###############################################################################

Makefile:
$(MAKDIR)/%.mak:
$(MAKDIR_ABS)/%.mak:
$(PLATFORMINCLUDE):

###############################################################################
# String macros to get source/header/object filenames etc.
###############################################################################

# get all source files
getAllSourceFiles		= $(wildcard *.cpp)	\
						  $(wildcard *.cxx)	\
						  $(wildcard *.cc)	\
						  $(wildcard *.c)	\
						  $(wildcard *.s)

getAllHeaderFiles		= $(wildcard *.h)

# get special source files
getBisonSources			= $(wildcard *.y)
getFlexSources			= $(wildcard *.l)
getMocSources			= $(shell grep -l Q_OBJECT *.h 2>/dev/null)
getUicSources			= $(wildcard *.ui)

# get all files
getAllFiles				= $(filter-out Makefile, \
						  $(shell ls -1F | grep -v '/$$' 2>/dev/null))

# add a platform dependent suffix, e.g. ".linux2_gcc" or ".irix65_CC"
# (usually applied to executables)
addPlatform				= $(addsuffix .$(PLATFORM),$(1))

# add the object directory as a prefix, e.g. "obj/linux2_gcc_dbg/..."
# (usually appied to object files)
addObjectDir			= $(strip $(addprefix $(OBJDIR)/,$(1)) )

# convert source filenames to object filenames
cppSourceToObject		= $(subst .cpp,$(OBJSUFFIX),$(1))
cxxSourceToObject		= $(subst .cxx,$(OBJSUFFIX),$(1))
ccSourceToObject		= $(subst .cc,$(OBJSUFFIX),$(1))
cSourceToObject			= $(subst .c,$(OBJSUFFIX),$(1))

cnvSourceToObject		= $(call cSourceToObject,	\
						  $(call ccSourceToObject,	\
						  $(call cxxSourceToObject,	\
						  $(call cppSourceToObject, $(1)))))
# first replace .c suffix, otherwise .cpp would get replaced as .opp! [GZ]

# convert object filenames to executable filenames
cnvObjectToExecutable	= $(subst $(OBJSUFFIX),%$(EXESUFFIX),$(1))

# convert object filenames to dependency filenames
cnvObjectToDependency	= $(subst $(OBJSUFFIX),$(DEPSUFFIX),$(1))
# don't use patsubst, because OBJSUFFIX contains a '.'! [GZ]

# to check if a directory is empty
getDirContents			= $(shell ls -1A $(1) 2>/dev/null)

###############################################################################
# Create library name / shared object name
###############################################################################

createLibName			= $(addprefix $(LIBPREFIX), \
						  $(addsuffix .$(LIBSUFFIX), $(1)))

createSOName			= $(addprefix $(SOPREFIX), \
						  $(addsuffix $(SOSUFFIX), $(1)))

###############################################################################
# Common variables
###############################################################################

# Command to ignore error return codes in the shell
ERRIGNORE				:= true

SEP1 := -------------------------------------------------------------------------------
SEP2 := +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
SEP3 := ===============================================================================
SEP4 := *******************************************************************************
SEP5 := !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

###############################################################################
# Include OS and compiler specific stuff
###############################################################################

ifneq ($(wildcard $(PLATFORMINCLUDE)),)
include $(PLATFORMINCLUDE)
else
$(error Missing include file $(PLATFORMINCLUDE))
endif

# We have to include this early, so that variables get their correct
# value before they are used in implicit rules [like %.test: %$(EXESUFFIX) ]
# [GZ]

###############################################################################
# Include personal customization file
###############################################################################

-include "dirs_$(OS).mak"

###############################################################################
# Internal targets (needed to extract automatic targets in subdir.mak)
###############################################################################

INTERNALTARGETS		+= default_target \
					   tests   test_progs   test_runs   test_files \
					   tests_l test_progs_l test_runs_l test_files_l \
					   help help_general help_init help_clean help_std \
					   help_aux help_top \
					   showvars showpathvars showallvars showcompvars

###############################################################################
# Testing commands
###############################################################################

TESTDIR		:= test
TESTBATCH	:= Commands
TESTTMPDIR	:= $(OBJDIR)
.PHONY: tests test_progs test_runs test_files \
		tests_l test_progs_l test_runs_l test_files_l

tests test_progs test_runs test_files: %: %_l

ifeq "$(CURRENTDIR)" "$(TESTDIR)"
# we are already in the test subdirectory

.SUFFIXES: .out .test

ifeq "$(wildcard $(TESTBATCH))" ""
    $(error Test directory $(CURRENTDIRRELFROMBASE) does not contain a \
			batch file '$(TESTBATCH)'!)
else

# extract targets from batch file
TEST_PRGS		:= $(shell cat $(TESTBATCH) | \
						   sed -e '/^\#/d' -e '/^[ 	]*$$/d' | cut -f1 -d' ' )
TEST_EXECS		:= $(filter-out %.sh,$(TEST_PRGS))
TEST_OUTS		:= $(addsuffix .out, $(subst .sh,,$(TEST_PRGS)))

ifeq "$(strip $(TEST_PRGS))" ""
    $(error Could not extract test program targets from file $(TESTBATCH)!)
endif

test_progs_l: $(call addObjectDir, $(TEST_EXECS))

tests_l test_runs_l: test_progs_l \
					 $(addsuffix .test, $(subst .sh,,$(TEST_PRGS)))

ifeq "$(OS)" "macosx"
VDIFFPROG := opendiff
else
VDIFFPROG := xxdiff
endif

define run_testprog
	@if [[ "$@" = *.test ]];												  \
	then																	  \
		echo "$(SEP4)"; echo "*** Running test program $*"; echo "$(SEP4)";   \
	else																	  \
		echo "$(SEP4)"; echo "*** Creating regression output $*.out"; echo "$(SEP4)";   \
	fi
	@cat $(TESTBATCH) | sed -e '/^\#/d' -e '/^[ 	]*$$/d' |				  \
	{																		  \
	while read p l;															  \
	do																		  \
		if [[ "$$p" = "$*" || "$$p" = "$*.sh" ]];							  \
		then																  \
			prog=$$p;														  \
			break;															  \
		fi;																	  \
	done;																	  \
	if [[ "$$prog" != "" ]];												  \
	then																	  \
		if [[ -x "$$prog" ]];												  \
		then																  \
			l="$$prog $$l";													  \
			echo ": $$l";													  \
			ulimit -c 0;													  \
			if [[ "$@" = *.test ]];											  \
			then															  \
				outdir=$(TESTTMPDIR);								  		  \
			else															  \
				outdir=.;											  		  \
			fi;																  \
			eval ./$$l > $$outdir/$*.out;							  		  \
			if [[ $$? = 0 ]];												  \
			then															  \
				if [[ -s $$outdir/$*.out ]];								  \
				then														  \
					if [[ "$@" = *.test ]];									  \
					then													  \
						if $(DIFF) $$outdir/$*.out $*.out >/dev/null;		  \
						then												  \
							echo "  ok";									  \
						else												  \
							echo;											  \
							echo "Test with program $$prog failed!";		  \
							echo "   Command: $$l";							  \
							echo "   Actual output in $$outdir/$*.out";		  \
							echo "   differs from regression output in $*.out";\
							echo "   $VDIFFPROG $$outdir/$*.out $*.out";	  \
							echo;											  \
						fi;													  \
					else													  \
						if grep -qi failed $*.out;							  \
						then												  \
							echo "Error";                                     \
							echo "Error: $*.out contains the word 'failed'!"; \
							echo "Error";                                     \
						fi;													  \
					fi;														  \
				else														  \
					echo "Error: $$prog didn't produce any output!";		  \
				fi;															  \
			else															  \
				echo "Error: Test program $$prog exited with error status!";  \
			fi;																  \
		else																  \
			echo "Error: No executable $$prog found";						  \
		fi;																	  \
	else																	  \
		echo "Error: no program found for $@ in $(TESTBATCH)!";				  \
	fi;																		  \
	}
endef

# Note: die {} sind (in der bash) notwendig,
#       da eine Pipe einen Subprozess startet; Variablen, die innerhalb
#       des 'while' gesetzt werden, waeren ohne die {} nach dem 'while'
#       verloren.

#	@export OSG_LOG_FILE=""; \
# doesn't work in OSG yet.

%.test : %$(EXESUFFIX)
	$(run_testprog)

%.test : %.sh
	$(run_testprog)

test_files_l: test_progs_l $(TEST_OUTS)

.PHONY:	forcemake

%.out: forcemake
	$(run_testprog)

# Note: Das Target test_files haengt absichtlich nicht von den .out-Files ab.
#       (Bloss, weil das Programm neu compiliert wurde, darf der .out-File
#        *nicht* neu gemacht werden!)

endif

else
# we are not yet in a test subdirectory

TESTDIRS	:= $(wildcard $(TESTDIR))

tests_l test_progs_l test_runs_l test_files_l:
ifeq "$(TESTDIRS)" ""
	echo
	echo $@: No directory '$(TESTDIR)' found!
	echo
else
ifeq "$(wildcard $(TESTDIR)/Makefile)" ""
	echo
	echo $@: Test directory $(TESTDIR) does not contain a Makefile!
	echo
else
	$(MAKE) -C $(TESTDIR) $@
endif
endif

endif

###############################################################################
# help
###############################################################################

.PHONY:	help help_general help_init help_clean help_std help_aux help_top

help: help_general help_init help_clean help_std help_aux help_top

help_general:
	@echo
	@echo "* Target naming convention:"
	@echo
	@echo "   <target>_l           - make <target> locally     (only current directory)"
	@echo "   <target>_r           - make <target> recursively (only sub-directories)"
	@echo

help_init:
	@echo
	@echo "* Initialization targets:"
	@echo
	@echo "   [dbg|opt]init        - create object directories and dependencies"
	@echo "   [dbg|opt]initscratch - same as init, in scratch directory"
	@echo "   [dbg|opt]depend      - create dependencies (included in init)"
	@echo
	@echo "   Remarks:"
	@echo "   - Dependencies are stored per source file in <objdir>/*.d and .../*.din ."
	@echo "   - A <platform> is defined by <os>_<compiler>."
	@echo "   - Explicit setting of <compiler> via \"setenv COMPILER <compiler>\""
	@echo "     *before* \"make [dbg|opt]init[scratch]\" or by expicit definition"
	@echo "     on the command line like \"make COMPILER=... <target>\"."
	@echo "   - File .lastdebug_<platform> stores last debug/optimize mode,"
	@echo "     targets without leading dbg/opt use this mode, default is debug."
	@echo

help_clean:
	@echo
	@echo "* Cleanup targets:"
	@echo
	@echo "   [dbg|opt]clean       - remove object files"
	@echo "   [dbg|opt]Clean       - remove obj files, libraries/executables, and .d files"
	@echo "   initclean            - remove all created files/dirs (current platform)"
	@echo "   distclean            - remove all created files/dirs (all platforms)"
	@echo

help_std:
	@echo
	@echo "* Standard targets:"
	@echo
	@echo "   default (or empty)   - make DEFAULT_TARGETS (current and sub-directories)"
	@echo "   [dbg|opt]lib         - make library from all LIBSOURCES"
	@echo "   [dbg|opt]so          - make shared object (this is for nodes)"
	@echo "   lib[dbg|opt]so       - make shared library (this is for libraries)"
	@echo "   <prog>               - make <prog> from <prog>.c* and library"
	@echo "   tests, test_runs     - run all tests"
	@echo "   test_progs           - make test programs (as spec'd in test/Commands)"
	@echo "   test_files           - create regression output files *.out for all tests"
	@echo "   prog.test            - run a single test (prog must occur in test/Commands)"
	@echo "   prog.out             - (re-)create a single regression output files"
	@echo

help_aux:
	@echo
	@echo "* Auxiliary targets:"
	@echo
	@echo "   showvars             - display some internal make variables"
	@echo "   showallvars          - display more internal make variables"
	@echo "   showcompvars         - display compiler variables"
	@echo "   showpathvars         - display makefile variables (paths mostly)"
	@echo
	@echo "   linkbin              - link binaries      into platform binary  dir"
	@echo "   linklib              - link shared object into platform library dir"
	@echo "   linklibso            - link shared library (see libso above)"
	@echo "   linkplg              - link shared object into platform plugins dir"
	@echo

help_top:
	@echo
	@echo "* Toplevel targets:"
	@echo
	#@echo "   joinlibs             - merge all subdirectories into one library"
	@echo "   doc                  - generate doxygen docu"
	@echo "   docclean             - remove generated documentation"

###############################################################################
# Display relevant variables (makefile debugging)
###############################################################################

.PHONY:	showpathvars showvars showallvars showcompvars

showallvars: showpathvars showvars showcompvars

showvars:
	@echo Variables:
	@echo ""
	@echo "DEFAULT_TARGETS=$(DEFAULT_TARGETS)"
	@echo "LIBNAME=$(LIBNAME)"
	@echo "SONAME=$(SONAME)"
	@echo "LIBSOURCES=$(LIBSOURCES)"
	@echo "LIBHEADERS=$(LIBHEADERS)"
	@echo "LIBOBJECTS=$(LIBOBJECTS)"
	@echo "LIBTARGETS=$(LIBTARGETS)"
	@echo "LIBTARGETS_MOC=$(LIBTARGETS_MOC)"
	@echo "OBJDIR=$(OBJDIR)"
	@echo "OBJDIRBASE=$(OBJDIRBASE)"
	@echo ""
	@echo "DEP_MAKEFILES=$(DEP_MAKEFILES)"
	@echo "DEFAULT_PROGRAMS=$(DEFAULT_PROGRAMS)"
	@echo "DEFAULT_SOURCES=$(DEFAULT_SOURCES)"
	@echo "EXCLUDED_HEADERS=$(EXCLUDED_HEADERS)"

# some echo have '..' instead of ".."
# in order to escape " inside variable values - needed under DOS ;-(

showpathvars:
	@echo ""
	@echo "PROJECT_BASE=$(PROJECT_BASE)"
	@echo "BASEDIR=$(BASEDIR)"
	@echo "BASEDIRABSOLUTE=$(BASEDIRABSOLUTE)"
	@echo "BASEDIRRELATIVE=$(BASEDIRRELATIVE)"
	@echo "CURRENTDIR=$(CURRENTDIR)"
	@echo "CURRENTDIRRELFROMBASE=$(CURRENTDIRRELFROMBASE)"
	@echo "SCRATCHDIR=$(SCRATCHDIR)"
	@echo "SCRATCHOBJDIR=$(SCRATCHOBJDIR)"
	@echo "HOMESCRATCHDIR=$(HOMESCRATCHDIR)"
	@echo "PWD=$(PWD)"
	@echo "SUBDIRS_ALL=$(SUBDIRS_ALL)"
	@echo "SUBDIRS=$(SUBDIRS)"
	@echo "TESTDIRS=$(TESTDIRS)"
	@echo "TEST_PRGS=$(TEST_PRGS)"
	@echo "TEST_EXECS=$(TEST_EXECS)"
	@echo "TEST_SCRIPTS= $(filter %.sh,$(TEST_PRGS))"
	@echo "TEST_OBJS=$(TEST_OBJS)"
	@echo "TEST_OUTS=$(TEST_OUTS)"
	@echo "OS=$(OS)"
	@echo "OSCOMP=$(OSCOMP)"
	@echo "PLATFORMINCLUDE=$(PLATFORMINCLUDE)"
	@echo "MAKEFLAGS=$(MAKEFLAGS)"

showcompvars:
	@echo ""
	@echo "CC=$(CC)"
	@echo "LD=$(LD)"
	@echo "CALLFLAGS=$(CALLFLAGS)"
	@echo "CXX=$(CXX)"
	@echo "CXXALLFLAGS=$(CXXALLFLAGS)"
	@echo 'CINCLUDEPATH=$(CINCLUDEPATH)'
	@echo 'CXXINCLUDEPATH=$(CXXINCLUDEPATH)'
	@echo "LIBDIRS=$(LIBDIRS)"
	@echo "INCDIRS_C=$(INCDIRS_C)"
	@echo "INCDIRS_CXX=$(INCDIRS_CXX)"
	@echo "LIBS=$(LIBS)"
	@echo 'LDPATH=$(LDPATH)'
	@echo 'LDFLAGS=$(LDFLAGS)'
	@echo "LDALLFLAGS=$(LDALLFLAGS)"
ifeq "$(OS)" "cygwin11"
	@echo 'INCLUDE=$(INCLUDE)'
	@echo 'LIB=$(LIB)'
	@echo 'PATH=$(PATH)'
endif

