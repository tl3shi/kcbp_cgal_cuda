#! @file
#
#  @brief
#  Special include for Linux with icc 7.x
#
#  setenv COMPILER icc
#
#  @author Gabriel Zachmann
#
#  @see getos.sh

###############################################################################
# Variables used internally
###############################################################################

### Include directories #######################################################

# STL 
INCDIR_STL			?= 
LIBDIR_STL			?= 

INCDIRS_C			= .

INCDIRS_CXX			= $(INCDIRS_C) \
					  $(INCDIR_STL)

### Library directories #######################################################

LIBDIRS				= /usr/X11R6/lib

### Disable standard include path #############################################

NOSTDINC_C			=
NOSTDINC_CXX		= #-nostdinc++

### Flags for generating dependencies #########################################

DEPFLAGS_C			= -M
DEPFLAGS_CXX		= $(DEPFLAGS_C)

### Defines ###################################################################

DEFINES_SOURCE		=
DEFINES_SYSTEM		= #-D_REENTRANT -D_THREAD_SAFE #-D_GNU_SOURCE -DLINUX=2.2
DEFINES_PLATFORM	= #-D_LITTLE_ENDIAN

DEFINES				= $(DEFINES_SOURCE)	\
					  $(DEFINES_SYSTEM)	\
					  $(DEFINES_PLATFORM)

### Warnings ##################################################################

WARNINGS			= -w2 -Wall -wd810,1418,981,383,1419
#  810: conversion from "long" to "float" may lose significant bits
# 1418: external definition with no prior declaration
#  981: operands are evaluated in unspecified order
#  383: value copied to temporary, reference to temporary used
# 1419: external declaration in primary source file


WARNINGS_OFF_CXX	= $(WARNINGS)
WARNINGS_OFF_C		= $(WARNINGS_OFF_CXX)
WARNINGS_OFF_LD		=
WARNINGS_OFF_FLEX	:=
WARNINGS_OFF_BISON	:= 				# additional woffs for bison-generated code
WARNINGS_OFF_DEP	:=              # we don't need warnings while making dep's

### Architecture Flags ########################################################

ISA                 :=


### Compiler-Flags ############################################################

FLAGS_C				= -ansi $(ISA)
FLAGS_CXX			= $(FLAGS_C)

### Linker-Flags ##############################################################

FLAGS_LD			= $(ISA)

### Shared Object Flags #######################################################

FLAGS_SO_C			=
FLAGS_SO_CXX		= $(FLAGS_SO_C)
FLAGS_SO_LD			= -shared				# generate shared object

### Debug / Optimize ##########################################################

ifeq ($(DEBUG),$(DEBUG_MARK))
CC_OPTIM			:= -g -O0 # -ftrapv
# -gdwarf-2 compiles macro def's into the .o files
LD_OPTIM			:= $(CC_OPTIM)
else
WARNINGS			+= -w
CC_OPTIM			:= -O3 -ipo -restrict -rcd -DNDEBUG -ansi_alias
LD_OPTIM			:= $(CC_OPTIM)
# -opt_report_file optim_report.txt
#       Erzeugt Info, welche Optimierungen vorgenommen wurden;
#       Achtung: Info wird immer hinten angehaengt, der File kann also
#       gross werden!
# -DNDEBUG switches off the assert() macro
# -Ob2	inline any function, at the compiler's discretion
# -rcd	enable fast float-to-int conversions
# -ipo	enable multi-file IP optimizations
# -restrict enable the 'restrict' keyword for disambiguating pointers
# -ansi_alias Arrays are not accessed out of bounds; pointers are not cast to
#             non-pointer types, and vice-versa; references to objects of
#             two different scalar types cannot alias.
# -O3 implies -unroll.
# -pc32  set internal FPU precision to 24 bit significand

endif

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
LIBS_POST4			:= imf pthread

# "imf": math lib optimized for P3 and P4; see section "Libraries / Default
#        Libraries" in the Intel Compiler User Guide.

### Binaries ##################################################################

AR					:= /usr/bin/ar
CXX					:= icc
CC					:= $(CXX)
LD					:= $(CXX)
FLEX				:= flex
BISON				:= bison
LIBDIR_BISON		:= $(TOOLSDIR_OS)/share/
DOXYGEN				:= doxygen
MOC					:= echo# wird in Qt.mak gesetzt
CTAGS				:= ctags

STRIP				:= strip
LNS					:= ln -s
MV					:= mv
RM					:= rm -f
RMR					:= rm -rf
DIFF				:= /usr/bin/diff

### Flags used in targets (see subdir.mak) ####################################

ARFLAGS				:= cr

### Includes / libs used in targets (see subdir.mak) ##########################

CINCLUDEPATH		= $(NOSTDINC_C)   $(addprefix -I,$(INCDIRS_C))
CXXINCLUDEPATH		= $(NOSTDINC_CXX) $(addprefix -I,$(INCDIRS_CXX))

LDPATH				= $(addprefix -L,$(LIBDIR) $(LIBDIRS) $(SYSTEMLIBDIRS))
LDLIBS				= $(addprefix -l,$(LIBS))

