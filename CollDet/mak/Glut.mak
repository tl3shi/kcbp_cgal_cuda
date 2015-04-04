
ifeq "$(OS)" "cygwin11"
GLUT_DIR				:= $(WIN_LIB_DIR)/glut
else

GLUT_DIR				:= /usr

endif

ifneq "$(OS)" "macosx"
INCDIRS_CXX				+= $(GLUT_DIR)/include
LIBDIRS					+= $(GLUT_DIR)
endif

ifeq "$(OS)" "cygwin11"
LIBS_POST2				+= glut32
else
ifeq "$(OS)" "macosx"
LDLOCALFLAGS			+= -framework GLUT
else
LIBS_POST2				+= glut
endif
endif

