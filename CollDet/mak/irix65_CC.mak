#! @file
#
#  @brief
#  Special include for IRIX 6.5 with system compilers
#

###############################################################################
# Variables used internally
###############################################################################

### Determine compiler version ################################################

ifeq "$(MAKELEVEL)" "0"
# do this only once

CV := $(shell CC -version 2>&1 | cat)
CV := $(word $(words $(CV)),$(CV))
CV := $(subst ., ,$(CV))
CV := $(wordlist 1,1,$(CV))$(wordlist 2,2,$(CV))$(wordlist 3,3,$(CV))
COMPILERVERSION := $(strip $(CV))

export CV COMPILERVERSION

endif

### Directories to customize ##################################################

INCDIR_STL		?= 
LIBDIR_STL		?= 

### Include directories #######################################################

INCDIRS_C			= .

INCDIRS_CXX			= $(INCDIRS_C) \
					   $(INCDIR_STL)

### Library directories #######################################################

LIBDIRS				= . $(LIBDIR_STL)
SYSTEMLIBDIRS		= 

### Flags for generating dependencies #########################################

DEPFLAGS_C			= -M
DEPFLAGS_CXX		= -M

### Defines ###################################################################

DEFINES_SOURCE		:= -D_IRIX_SOURCE
DEFINES_SYSTEM		:= -DIRIX=$(shell expr substr `uname -r` 1 1)
DEFINES_PLATFORM	:=

DEFINES				:= $(DEFINES_SOURCE)	\
					  $(DEFINES_SYSTEM)	\
					  $(DEFINES_PLATFORM)

### Warnings ##################################################################

WARNINGS			:= -fullwarn
WARNINGS_OFF_CXX	:= $(WARNINGS) -woff 1110,3201,1424
WARNINGS_OFF_C		:= $(WARNINGS_OFF_CXX)
WARNINGS_OFF_FLEX	:= 
WARNINGS_OFF_BISON	:= 				# additional woffs for bison-generated code
WARNINGS_OFF_DEP	:= -woff all	# we don't need warnings while making dep's

# Ignored linker warnings:
# LD:
#   84: <lib> is not used for resolving any symbol
#   85: definition of <symbol> in <file> preempts that definition in <lib>
#  127: Two shared objects with the same soname, ... Ignoring the latter.
#  134: weak definition of <symbol> in <file> preempts that
#		weak definition in <lib>
# CXX:
# 1110: The indicated statement is not reachable
# 3201: The parameter "..." was never referenced
# 1375: The destructor for base class "xxx" is not virtual
# 1424: The template parameter ".." is not used ...
# 3331: Returning a reference to local variable.

# Turn some warnings into errors
WARNINGS_OFF_CXX	+= -diag_error 1548,1116,1681,1552,3303,1682,1551,1197,1174,3331

# we can't make 1375 a diag_error yet, because of Qt [GZ]

### Architecture Flags ########################################################

ISA					?= -mips4				# -mips1 .. -mips4

### Compiler-Flags ############################################################

FLAGS_C				:= $(ISA) \
					   -xansi -use_readonly_const -float_const

FLAGS_CXX			:= $(FLAGS_C) \
					   -no_auto_include \
					   -LANG:ansi-for-init-scope=on \
                       -LANG:restrict -LANG:std \
					   -FE:template_in_elf_section

### Linker-Flags ##############################################################

FLAGS_LD			:= $(ISA) -all -demangle

### Shared Object Flags #######################################################

FLAGS_SO_C			:=
FLAGS_SO_CXX		:= $(FLAGS_SO_C)
FLAGS_SO_LD			:= -shared 				# generate shared object

### Debug / Optimize ##########################################################

ifeq ($(DEBUG),$(DEBUG_MARK))
CC_OPTIM			:= -g -INLINE:none
LD_OPTIM			:= $(CC_OPTIM)
else
CC_OPTIM			:= -OPT:Olimit=8192 -O3 -TARG:madd=ON \
					   -TENV:X=4 \
					   -DNDEBUG
# Macht inlining sowieso [GZ].
# X=4 erlaubt ziemlich heftige code movements, wobei alle
# FPE's und sogar memory exceptions ignoriert werden.
# Muss man evtl. wieder rausnehmen [GZ].
# -DNDEBUG switches off the assert() macro
LD_OPTIM			:= $(CC_OPTIM)
endif

# Folgende Optionen koennen zwar etwas runtime performance kosten;
# trotzdem sollte man ab und zu mal damit comipilieren, 
# um noch mehr Bugs vom Compiler finden zu lassen.
# [setenv DEBUG_CODE 1]
ifneq "$(DEBUG_CODE)" "0"
CC_OPTIM			+= -DEBUG:trap_uninitialized:div_check=3 \
					   -DEBUG:varargs_interface_check:varargs_prototypes \
					   -DEBUG:verbose_runtime \
					   -TARG:exc_min=OZV
# -DEBUG:subscript_check scheint nicht viel zu tun [GZ]
endif

### AR Flags ##################################################################

ARFLAGS				:= -o

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
LIBS_POST4			:= Cio m

### Binaries ##################################################################

AR					:= /usr/bin/CC -ar
CXX					:= /usr/bin/CC
CC					:= $(CXX)
LD					:= $(CXX)
FLEX				:= $(TOOLSDIR_OS)/bin/flex
BISON				:= $(TOOLSDIR_OS)/bin/bison
LIBDIR_BISON		:= $(TOOLSDIR_OS)/share/
DOXYGEN				:= $(TOOLSDIR_OS)/bin/doxygen
MOC					:= echo# wird in Qt.mak gesetzt
CTAGS				:= ctags 

STRIP				:= strip
LNS					:= ln -s
MV					:= mv
RM					:= rm -f
RMR					:= rm -rf
DIFF				:= /bin/diff

MOC					:= echo					# will be redefined in qt?.mak

### Includes / libs used in targets (see subdir.mak) ##########################

CINCLUDEPATH		= $(addprefix -I,$(INCDIRS_C))
CXXINCLUDEPATH		= $(addprefix -I,$(INCDIRS_CXX))

LDPATH				= $(addprefix -L,$(LIBDIR) $(LIBDIRS) $(SYSTEMLIBDIRS))
LDLIBS				= $(addprefix -l,$(LIBS))

### ii files ##################################################################

II_FILESDIR			= $(call addObjectDir, ii_files)
