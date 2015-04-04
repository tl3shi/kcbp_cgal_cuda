#! @file
#
#  @brief
#  makefile include for qhull headers and libs
#
#  @author Gabriel Zachmann
#

QHULL			?= $(BASEDIR)/common/qhull
INCDIRS_CXX		+= $(QHULL)
LIBS_POST1		+= qhull


