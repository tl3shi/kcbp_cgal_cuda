#! @file
#
#  @brief
#  Special include for Cygwin
#
#  @author Gabriel Zachmann
#
# @todo
#   Alles, was mit C zu tun hat, rausschmeissen.
#   Wir wollen auch C-Files als C++ compilieren! [GZ]
#

###############################################################################
# Variables used internally
###############################################################################

### General directories #######################################################

ifeq "$(INTEL_DIR)" ""
$(error environment variable INTEL_DIR is not set! \
This should have been done by Coll/internal/Tools/cygwin/share/bashrc!)
endif
ifeq "$(MSVC_DIR)" ""
$(error environment variable MSVC_DIR is not set! \
This should have been done by Coll/internal/Tools/cygwin/share/bashrc!)
endif
ifeq "$(WIN_PATH)" ""
$(error environment variable WIN_PATH is not set! \
This should have been done by Coll/internal/Tools/cygwin/share/bashrc!)
endif
ifeq "$(WIN_LIB_DIR)" ""
$(error environment variable WIN_LIB_DIR is not set! \
This should have been done by Coll/internal/Tools/cygwin/share/bashrc!)
endif

### Include directories #######################################################

# Use Microsoft STL temporary
#INCDIR_STL			?= $(WIN_LIB_DIR)/ICL/stdlib_0608
INCDIR_STL			?= 

INCDIRS_C			= . $(BASEDIR)/internal/include

INCDIRS_CXX			= $(INCDIRS_C) $(INCDIR_STL)

INCDIRS_DEP_C		= 

INCDIRS_DEP_CXX		= $(INCDIR_STL)

### Library directories #######################################################

LIBDIRS				= $(WIN_LIB_DIR)/ICL/stdlib_0608/Release
SYSTEMLIBDIRS		= 

### Defines ###################################################################

DEFINES_SOURCE		:= -DWIN32 -D_WINDOWS -D_WIN32_WINNT=0x0400 -DWINVER=0x0400
# WIN32 might be needed for OpenSG header files
DEFINES_SYSTEM		:= 
DEFINES_PLATFORM	:=

DEFINES				:= $(DEFINES_SOURCE)	\
					   $(DEFINES_SYSTEM)	\
					   $(DEFINES_PLATFORM)

### Warnings ##################################################################

WARNINGS			:= -W5
WARNINGS_OFF_CXX	:= $(WARNINGS) -Qwd985,981,193
WARNINGS_OFF_C		:= $(WARNINGS_OFF_CXX)
WARNINGS_OFF_LD		:= -warn:3
WARNINGS_OFF_FLEX	:= -Qwd111,810,383
WARNINGS_OFF_BISON	:= 				# additional woffs for bison-generated code
WARNINGS_OFF_DEP	:= -W0			# no warnings during dependency making
									# because there is no stderr under Windoze

# Ignored warnings (woff):
# 985: identifier "..." was truncated in debug information
# 981: operands are evaluated in unspecified order
# 193: zero used for undefined preprocessing identifier
# 111: statement is unreachable
# 810: conversion from "..." to "..." may lose significant bits
# 383: value copied to temporary, reference to temporary used

### Architecture Flags ########################################################

ISA					?=

### Compiler-Flags ############################################################

FLAGS_CXX			:= $(ISA) \
					   -Qansi -MDd -Gi- -GR -Qrestrict \
					   -Qoption,cpp,--new_for_init -TP
# -GX- : for now, we switch off exceptions
# -TP  : treat all files as C++ Files, so we can define namespaces in C source
# --new_for_init : makes scope of 'for ( int i ... )' local to loop.

### Linker-Flags ##############################################################

FLAGS_LD			:= $(ISA) \
					   -subsystem:console -incremental:no \
					   -opt:noref

### Shared Object Flags #######################################################

FLAGS_SO_C			:=
FLAGS_SO_CXX		:= $(FLAGS_SO_C)
FLAGS_SO_LD			 = -dll	-map -mapinfo:exports,fixups \
						$(addprefix -DEF:,$(wildcard $(notdir $(basename $@)).def))

### Debug / Optimize ##########################################################

ifeq ($(DEBUG),$(DEBUG_MARK))
CC_OPTIM			:= -Od -Zi -Z7 -GZ
LD_OPTIM			:= -debug
else
CC_OPTIM			:= -O2 -G7 -GF -Ob2 -ipo -DNDEBUG
LD_OPTIM			:= -opt:ref
#-Ob2	inline any function, at the compiler's discretion
#-rcd	enable fast float-to-int conversions

# -DNDEBUG switches off the assert() macro
endif

### Flags for generating dependencies #########################################

DEPFLAGS_C			= -QM
DEPFLAGS_CXX		= -QM

###############################################################################
# Variables used externally
###############################################################################

### These may also be modified externally #####################################

# INCDIRS_C			# instead of CINCLUDEPATH
# INCDIRS_CXX		# instead of CXXINCLUDEPATH
# LIBDIRS			# instead of LDPATH

### Initialize library lists ##################################################

LIBS_PRE			:=
LIBS_PROJECT		:= 
LIBS_POST1			:=
LIBS_POST2			:=
LIBS_POST3			:=
# Use Microsoft STL temporary
#LIBS_POST4			:= libCio 
LIBS_POST4			:= user32 kernel32 winmm gdi32 wsock32 

### Binaries ##################################################################

# Achtung der ar erzeugt keine Template Instanzen,
# sollte man vielleicht durch CC ersetzen.

LD					:= "$(INTEL_COMP_DIR)/bin/xilink" -nologo # -verbose
AR					:= $(LD)
CC					:= "$(INTEL_COMP_DIR)/bin/icl"
CXX					:= $(CC)
# -verbose gibt aus,
# aus welcher Lib welche Symbole zur Symbol-Resolution geholt werden.
FLEX				:= $(TOOLSDIR_OS)/bin/flex
BISON				:= $(TOOLSDIR_OS)/bin/bison
LIBDIR_BISON		:= $(TOOLSDIR_OS)/share/
DOXYGEN				:= $(TOOLSDIR_OS)/bin/doxygen
MOC					:= echo# will be redefined in Qt.mak
CTAGS				:= ctags 

PRINTF				:= /bin/printf
STRIP				:= echo
LNS					:= cp
MV					:= mv
RM					:= rm -f
RMR					:= rm -rf
DIFF				:= /bin/diff

### Flags used in subdir.mak (some defaults are in common.mak) #################

BISONCXXFLAGS		:= -DMSDOS
FLEXCXXFLAGS		:= -DYY_NEVER_INTERACTIVE -I$(BASEDIR)/internal/include

CC_OUT_OPT			:= -Fo# no space here! [GZ]
LD_OUT_OPT			:= -out:# no space here! [GZ]
AR_OUT_OPT			:= -out:
ARFLAGS				:= -lib

EXESUFFIX			:= .exe					# dot is necessary here [GZ]
OBJSUFFIX			:= .obj
LIBSUFFIX			:= lib
LIBPREFIX			:=
SOPREFIX			:=
SOSUFFIX			:= .dll

### Includes / libs used in targets (see subdir.mak) ##########################

cygpath			= $(foreach p, $(1), "$(shell cygpath -w $(p))")
# "" is important! [GZ]

prefix_cygpath	= $(subst @, ,$(addprefix $(1),$(call cygpath, $(subst \ ,@,$(2))) ))
# this subst and back-subst are needed because $(foreach) and $(addprefix)
# treat space as a word separator, no matter whether or not space is escaped
# by \ ;-(

CINCLUDEPATH		= $(call prefix_cygpath,-I,$(INCDIRS_C)  )
CXXINCLUDEPATH		= $(call prefix_cygpath,-I,$(INCDIRS_CXX))

LDPATH				= $(call prefix_cygpath,-LIBPATH:,$(LIBDIR) $(LIBDIRS) $(SYSTEMLIBDIRS))
LDLIBS				= $(addsuffix .$(LIBSUFFIX),$(LIBS))

### Flags used in target depend (see subdir.mak) ##############################

DEP_CINCLUDEPATH	= $(addprefix -I,$(INCDIRS_DEP_C))
DEP_CXXINCLUDEPATH	= $(addprefix -I,$(INCDIRS_DEP_CXX))

