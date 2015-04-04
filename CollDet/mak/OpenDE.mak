# .mak file for package Open Dynamic Environment

ODEROOT         = /home/weller/OpenDE/ode-0.5/

INCDIRS_CXX		+= $(ODEROOT)/include
LIBDIRS			+= $(ODEROOT)/lib

LIBS_POST1		+= ode

