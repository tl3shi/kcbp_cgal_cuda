#! @file
#
#  @brief
#    Makefile part for generating documentation using doxygen
#

ifeq "$(BASEDIR)" ""
$(error Makefile variable BASEDIR must be set first)
endif

ifeq "$(DOXYGEN)" ""
DOXYGEN := doxygen
endif

# This must match the OUTPUT_DIRECTORY in colldet.doxygen.cfg!!
DOCDIR := $(BASEDIR)/doc

.PHONY: doc docclean

doc:
	$(MAKE) docclean
	mkdir $(DOCDIR)
	$(DOXYGEN) $(BASEDIR)/mak/colldet.doxygen.cfg

docclean:
	$(RMR) $(DOCDIR)

