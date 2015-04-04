# Package/project makefile include for OSG.

# The flags needed by/for OpenSG should periodically checked with
#
#    osg-config --cflags
#    osg-config Win32 --lflags
#    osg-config Win32 --llib


#REQUIRED_PACKAGES		+=

OSGPOOL					?=

INCDIRS_CXX				+= /usr/local/include

FLAGS_CXX				+= -DOSG_WITH_GLUT 

ifeq ($(COMPILER),icc)
FLAGS_CXX               += -DOSG_SUPPORT_NO_GEO_INTERFACE \
                           -D__INTEL_COMPILER_VERSION=800
endif

ifeq "$(DEBUG)" "$(DEBUG_MARK)"
FLAGS_CXX				+= -DOSG_DEBUG
LIBDIRS					+= /usr/local/lib/dbg
else
LIBDIRS					+= /usr/local/lib/opt
endif
LIBDIRS					+= /usr/X11R6/lib

ifeq "$(OS)" "macosx"
LDLOCALFLAGS			+= -framework OpenGL -framework Foundation
endif

LIBS_POST1	            += OSGSystem OSGBase

ifeq "$(OS)" "cygwin11"
LIBS_POST3            	+=  opengl32 glu32
else
ifeq "$(OS)" "macosx"
LIBS_POST3            	+=  GLU Xmu Xi Xt X11
else
LIBS_POST3            	+=  GLU GL Xmu Xi Xt X11
endif
endif

