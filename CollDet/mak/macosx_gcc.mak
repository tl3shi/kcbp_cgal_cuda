#! @file
#
#  @brief
#  Special include for Mac OS X
#
#  This Makefile include is written for use with GNU make and is included
#  on the ca OS X platform.
#
#  @author Gabriel Zachmann
#
#  @see getos.sh
#

###############################################################################
# Variables used internally
###############################################################################

# include $(PROJECT_BASE)/mak/deps.mak

### Include directories #######################################################

# STL 
INCDIR_STL		?= 
LIBDIR_STL		?=

INCDIRS_C			= . $(OPTIONAL_INCPATH)

INCDIRS_CXX			= $(INCDIRS_C) \
					  $(INCDIR_STL)

### Library directories #######################################################

LIBDIRS				= . $(OPTIONAL_LIBDIRS)
SYSTEMLIBDIRS		= 

### Disable standard include path #############################################

NOSTDINC_C			=
NOSTDINC_CXX		= #-nostdinc++

### Flags for generating dependencies #########################################

DEPFLAGS_C			= -M
DEPFLAGS_CXX		= -M

### Defines ###################################################################

#Probleme mit Attribut -dM bei g++ (3.2) ('missing seperator')
DEFINES_SOURCE		= 
DEFINES_SYSTEM		= #-D_REENTRANT -D_THREAD_SAFE #-D_GNU_SOURCE -DLINUX=2.2
DEFINES_PLATFORM	= #-D_LITTLE_ENDIAN

DEFINES				= $(DEFINES_SOURCE)	\
					  $(DEFINES_SYSTEM)	\
					  $(DEFINES_PLATFORM)

### Warnings ##################################################################

WARNINGS			= -Wall -Wextra -Wpointer-arith -Wcast-qual \
					  -Wcast-align -Wconversion -Woverloaded-virtual \
					  -Wsign-compare -Wnon-virtual-dtor \
					  -Woverloaded-virtual -Wfloat-equal \
					  -Wno-system-headers \
                      -Wdisabled-optimization \
                      -Wold-style-cast -Wshadow

WARNINGS_OFF_CXX	= $(WARNINGS) -Wno-reorder
WARNINGS_OFF_C		= $(WARNINGS_OFF_CXX) -Wmissing-prototypes
WARNINGS_OFF_LD		=
WARNINGS_OFF_FLEX	:=
WARNINGS_OFF_BISON	:= 				# additional woffs for bison-generated code
WARNINGS_OFF_DEP	:=              # we don't need warnings while making dep's

### Architecture Flags ########################################################

ISA					:= 

### Compiler-Flags ############################################################

FLAGS_C				= $(ISA) -ansi $(OPTIONAL_CFLAGS)
FLAGS_CXX			= $(FLAGS_C)

### Linker-Flags ##############################################################

FLAGS_LD			= $(ISA)

### Shared Object Flags #######################################################

FLAGS_SO_C			=
FLAGS_SO_CXX		= $(FLAGS_SO_C)
FLAGS_SO_LD			= $(FLAGS_LD) -dynamiclib -Wl,-single_module -flat_namespace

### Debug / Optimize ##########################################################

ifeq "$(DEBUG)" "$(DEBUG_MARK)"
CC_OPTIM			:= -g3 -ggdb # -ftrapv
LD_OPTIM			:= $(CC_OPTIM)
else
WARNINGS			+= -w
CC_OPTIM			:= -O3 \
					  -funroll-loops -felide-constructors \
					  -ffast-math \
					  -DNDEBUG \
					  -ftree-vectorize -msse3 \
					  -funroll-loops

					  #-fprofile-use -fbranch-probabilities -freorder-functions
					  #-fprofile-generate	# scheint nichts zu bringen

					  #-fsee -fipa-pta  # testen; erst in gcc 4.2 vorhanden
LD_OPTIM			:= $(CC_OPTIM)
# O3 comprises -finline-functions
# -DNDEBUG switches off the assert() macro
# -ftree-vectorize = loop vectorization
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
LIBS_PROJECT		:= $(OPTIONAL_LIBS)
LIBS_POST1			:=
LIBS_POST2			:=
LIBS_POST3			:=
LIBS_POST4			:= # pthread

### Binaries ##################################################################

AR					:= /usr/bin/ar
CXX					:= g++
#CXX					:= g++-mp-4.2	# von MacPorts -- langsamer!
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

### Flags used in subdir.mak (some defaults are in common.mak) #################

SOPREFIX			:= lib
SOSUFFIX			:= .dylib

### Flags used in targets (see subdir.mak) ####################################

ARFLAGS				:= cr

### Includes / libs used in targets (see subdir.mak) ##########################

CINCLUDEPATH		= $(NOSTDINC_C)   $(addprefix -I,$(INCDIRS_C))
CXXINCLUDEPATH		= $(NOSTDINC_CXX) $(addprefix -I,$(INCDIRS_CXX))

LDPATH				= $(addprefix -L,$(LIBDIR) $(LIBDIRS) $(SYSTEMLIBDIRS))
LDLIBS				= $(addprefix -l,$(LIBS))

