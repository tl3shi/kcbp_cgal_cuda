#! @file
#
#  @brief
#  Standard include for package makefiles
#
#  This file must be included in directories which actually contain sources.
#

###############################################################################
# Check that common.mak was included
###############################################################################

ifneq "$(HAVE_COMMON_MAK)" "1"
$(error common.mak has to be included first)
endif

###############################################################################
# Internal targets (needed to extract automatic targets)
###############################################################################

INTERNALTARGETS		+= default \
					   default_l \
					   lib   dbglib   optlib \
					   lib_l dbglib_l optlib_l \
					   so   dbgso   optso \
					   so_l dbgso_l optso_l \
					   chkobjdir chkscratchdir \
					   bisonclean flexclean mocclean uicclean \
					   linkclean auxclean \
					   clean   dbgclean   optclean \
					   clean_l dbgclean_l optclean_l \
					   Clean   dbgClean   optClean \
					   Clean_l dbgClean_l optClean_l \
					   distclean   initclean \
					   distclean_l initclean_l \
					   $(OBJDIRBASE) mkobjdir mkobjdirscratch \
					   init   dbginit   optinit \
					   init_l dbginit_l optinit_l \
					   initscratch   dbginitscratch   optinitscratch \
					   initscratch_l dbginitscratch_l optinitscratch_l \
					   depend   dbgdepend   optdepend \
					   depend_l dbgdepend_l optdepend_l \
					   linkbin linkscript linklib linkplg \
					   linkinclude linkdata linkexamples \
					   install \
					   install_l

.SECONDARY:
# don't remove any intermediate target

###############################################################################
# Automatically exclude default sources from library sources
###############################################################################

ALLSOURCES			:= $(call getAllSourceFiles)
ALLHEADERS			:= $(call getAllHeaderFiles)
ALLOBJECTS			:= $(call addObjectDir, \
						 $(call cnvSourceToObject, $(ALLSOURCES)))

ifneq "$(DEFAULT_TARGETS)" ""
DEFAULT_PROGRAMS	= $(filter-out $(INTERNALTARGETS), $(DEFAULT_TARGETS))
DEFAULT_SOURCES		= $(addsuffix .cpp, $(DEFAULT_PROGRAMS)) \
					  $(addsuffix .cxx, $(DEFAULT_PROGRAMS)) \
					  $(addsuffix .cc , $(DEFAULT_PROGRAMS)) \
					  $(addsuffix .c  , $(DEFAULT_PROGRAMS))
DEFAULT_HEADERS		= $(addsuffix .hpp, $(DEFAULT_PROGRAMS)) \
					  $(addsuffix .h  , $(DEFAULT_PROGRAMS))

DEFAULT_SOURCES		:= $(filter $(DEFAULT_SOURCES), $(ALLSOURCES))
DEFAULT_HEADERS		:= $(filter $(DEFAULT_HEADERS), $(ALLHEADERS))

EXCLUDED_SOURCES	+= $(DEFAULT_SOURCES)
EXCLUDED_HEADERS	+= $(DEFAULT_HEADERS)
endif

###############################################################################
# Automatically exclude test sources from library sources
###############################################################################

EXCLUDED_SOURCES	+= test% moc_%.cpp
#  M$VC puts the files moc_*.cpp in the source dir
EXCLUDED_HEADERS	+= test%

###############################################################################
# Library sources/headers
###############################################################################


LIBSOURCES			= $(filter-out $(EXCLUDED_SOURCES), $(ALLSOURCES))
LIBHEADERS			= $(filter-out $(EXCLUDED_HEADERS), $(ALLHEADERS))

LIBSOURCES			+= $(INCLUDED_SOURCES)
LIBHEADERS			+= $(INCLUDED_HEADERS)

LIBTARGETS			=

###############################################################################
# Handle bison sources
###############################################################################

BISONPREFIX			?= bison_
BISONSUFFIX			?= .cpp

ifeq ($(NO_AUTO_BISON),)
LIBSOURCES_BISON	:= $(call getBisonSources)
endif

ifneq ($(LIBSOURCES_BISON),)
LIBTARGETS_BISON	:= $(call addObjectDir, \
						$(patsubst %.y, $(BISONPREFIX)%$(BISONSUFFIX),	\
										$(LIBSOURCES_BISON)))
LIBHEADERS_BISON	:= $(call addObjectDir, \
						$(patsubst %.y, $(BISONPREFIX)%.h,		\
										$(LIBSOURCES_BISON)))
LIBOUTPUTS_BISON	:= $(call addObjectDir, \
						$(patsubst %.y, $(BISONPREFIX)%.output,	\
										$(LIBSOURCES_BISON)))
LIBINTERNAL_BISON	:= $(call addObjectDir, \
						$(patsubst %.y, $(BISONPREFIX)%.tab.c,	\
										$(LIBSOURCES_BISON)) 	\
						$(patsubst %.y, $(BISONPREFIX)%.tab.h,	\
										$(LIBSOURCES_BISON)))	\
					   $(LIBHEADERS_BISON) \
					   $(LIBOUTPUTS_BISON)

LIBTARGETS			+= $(LIBTARGETS_BISON)
INCDIRS_C			+= $(OBJDIR)

$(LIBHEADERS_BISON): %.h: %$(BISONSUFFIX)
endif

###############################################################################
# Handle flex sources
###############################################################################

FLEXPREFIX			?= flex_
FLEXSUFFIX			?= .cpp

ifeq ($(NO_AUTO_FLEX),)
LIBSOURCES_FLEX		:= $(call getFlexSources)
endif

ifneq ($(LIBSOURCES_FLEX),)
LIBTARGETS_FLEX		:= $(call addObjectDir, \
						$(patsubst %.l, $(FLEXPREFIX)%$(FLEXSUFFIX), \
					   					$(LIBSOURCES_FLEX)))

LIBTARGETS			+= $(LIBTARGETS_FLEX)
endif

###############################################################################
# Handle Qt moc sources
###############################################################################

MOCPREFIX			?= moc_

LIBSOURCES_MOC		:= $(call getMocSources)

ifneq "$(LIBSOURCES_MOC)" ""
LIBTARGETS_MOC		:= $(call addObjectDir, \
					   $(patsubst %.h, $(MOCPREFIX)%.cpp, \
					   				$(LIBSOURCES_MOC)))

LIBTARGETS			+= $(LIBTARGETS_MOC)
endif

###############################################################################
# Handle Qt uic sources
###############################################################################

UICPREFIX			?= uic_
UICSUFFIX_CPP		?= .cpp
UICSUFFIX_H			?= .h

LIBSOURCES_UIC		:= $(call getUicSources)

ifneq ($(LIBSOURCES_UIC),)
LIBTARGETS_UIC		:= $(call addObjectDir, \
						$(patsubst %.ui, $(UICPREFIX)%$(UICSUFFIX_CPP), \
					   			$(LIBSOURCES_UIC)))

LIBHEADERS_UIC		:= $(call addObjectDir, \
						$(patsubst %.ui, $(UICPREFIX)%$(UICSUFFIX_H), \
					   			$(LIBSOURCES_UIC)))

# Note: uic-generated sources have to be moc'ed also
LIBTARGETS_UIC		+= $(call addObjectDir, \
						$(patsubst %.ui, $(MOCPREFIX)$(UICPREFIX)%.cpp, \
								$(LIBSOURCES_UIC)))

LIBTARGETS			+= $(LIBTARGETS_UIC)
LIBHEADERS			+= $(LIBHEADERS_UIC)
INCDIRS_CXX		    += $(OBJDIR)

endif

###############################################################################
# Automatic target generation
###############################################################################

EXCLUDED_TARGETS	= $(INTERNALTARGETS) \
					  $(LIBSOURCES_BISON) \
					  $(LIBSOURCES_FLEX) \
					  $(LIBSOURCES_MOC) \
					  $(LIBSOURCES_UIC) \
					  $(LIBTARGETS) \
					  $(LIBSOURCES) \
					  $(LIBHEADERS)

AUTOTARGET			:= $(filter-out $(EXCLUDED_TARGETS), $(MAKECMDGOALS))

ifneq ($(AUTOTARGET),)

AUTOTARGET_COUNT	:= $(words $(AUTOTARGET))

ifneq ($(AUTOTARGET_COUNT),1)
AUTOTARGET_WARNING	:= "Warning: More than one automatic target, ignoring all but first."
AUTOTARGET			:= $(firstword $(AUTOTARGET))
else
AUTOTARGET_WARNING	:=
AUTOTARGET			:= $(strip $(AUTOTARGET))
endif

AUTOTARGET_SOURCE	:= $(filter $(AUTOTARGET).cpp, $(ALLSOURCES))

ifeq ($(AUTOTARGET_SOURCE),)
AUTOTARGET_SOURCE	:= $(filter $(AUTOTARGET).cxx, $(ALLSOURCES))
endif

ifeq ($(AUTOTARGET_SOURCE),)
AUTOTARGET_SOURCE	:= $(filter $(AUTOTARGET).cc, $(ALLSOURCES))
endif

ifeq ($(AUTOTARGET_SOURCE),)
AUTOTARGET_SOURCE	:= $(filter $(AUTOTARGET).c, $(ALLSOURCES))
endif

ifneq ($(AUTOTARGET_SOURCE),)
AUTOTARGET_OBJ		:= $(call addObjectDir, \
					   $(call cnvSourceToObject, $(AUTOTARGET_SOURCE)))
AUTOTARGET_PRG		:= $(call addObjectDir, $(AUTOTARGET))
EXCLUDED_SOURCES	+= $(AUTOTARGET_SOURCE)
else
AUTOTARGET_WARNING	:=
AUTOTARGET			:=
AUTOTARGET_OBJ		:=
AUTOTARGET_PRG		:=
endif

endif	# AUTOTARGET

###############################################################################
# Library objects
###############################################################################

LIBOBJECTS			= $(call addObjectDir, \
					  $(call cnvSourceToObject, $(LIBSOURCES)))

LIBOBJECTS			+= $(call cnvSourceToObject, $(LIBTARGETS))

###############################################################################
# Library name/link
###############################################################################

ifneq "$(strip $(LIBOBJECTS))" ""

LIBNAMENOPATH		?= $(call createLibName, $(CURRENTDIR))
LIBNAME				= $(call addObjectDir, $(LIBNAMENOPATH))

endif

###############################################################################
# Shared object name/link
###############################################################################

ifneq "$(strip $(LIBOBJECTS))" ""

SONAMENOPATH		?= $(call createSOName, $(CURRENTDIR))
SONAME				= $(call addObjectDir, $(SONAMENOPATH))

endif

###############################################################################
# Includes for required packages
###############################################################################

# "Statistics"

INCLUDED_PACKAGES	:=

# createIncludePackages: create list of exisiting .mak-files for corresponding packages
# createDefaultPackages: create list of packages w/o corresponding .mak-files

createIncludeMakefiles	= $(addprefix $(MAKDIR)/, $(addsuffix .mak, $(1)))
createIncludePackages	= $(strip \
							$(call createIncludeMakefiles, $(1)))
createDefaultPackages	= $(strip \
						  $(foreach p, $(1), \
							$(if $(wildcard \
									$(call createIncludeMakefiles, $(p))),\
								 ,$(p))))

# Remember that REQUIRED_PACKAGES may only contain paths when referencing 
# default packages outside the current scope (i.e. parent dir or project base)!

# chkExistingDir: check if given directory exists
# getdirDefaultPackages: look for directories in current dir, in parent dir
#						 and in project base

chkExistingDir			= $(if $(shell test -d $(1) && echo "ok"),$(1),)
getdirDefaultPackages	= $(foreach p, $(1), \
							$(if $(call chkExistingDir, $(p)),\
								 $(p),\
								 $(if $(call chkExistingDir, ../$(p)),\
								 ../$(p),\
								 $(call chkExistingDir, $(BASEDIR)/$(p)))))

# addPackage: add a package to the REQUIRED_PACKAGES variable

addPackage				= $(filter-out $(INCLUDED_PACKAGES) \
									   $(REQUIRED_PACKAGES), $(1))

# In order to allow some "recursion" here, i.e. additions to the 
# variable REQUIRED_PACKAGES in included files, we use multiple 
# include commands allowing "nested" includes driven by REQUIRED_PACKAGES.

ifneq ($(strip $(REQUIRED_PACKAGES)),)

# make list unique
REQUIRED_PACKAGES	:= $(sort $(REQUIRED_PACKAGES))
INCLUDED_PACKAGES	+= $(REQUIRED_PACKAGES)
DEFAULT_PACKAGES	:= $(call createDefaultPackages, $(REQUIRED_PACKAGES))
INCLUDE_PACKAGES	:= $(call createIncludePackages, $(REQUIRED_PACKAGES))
REQUIRED_PACKAGES	:=
ifneq "$(DEFAULT_PACKAGES)" ""
ifneq ($(DEBUG_MAKE),)
$(warning 1. $(MAKDIR)/DefaultPackage.sh: DEFAULT_PACKAGES=$(DEFAULT_PACKAGES))
endif
$(shell $(SHELL) $(MAKDIR)/DefaultPackage.sh "$(DEFAULT_PACKAGES)" $(MAKDIR_ABS) $(BASEDIRABSOLUTE) $(OBJDIRBASE))
# now there are possibly more .mak-files
endif
ifneq ($(INCLUDE_PACKAGES),)
ifneq ($(DEBUG_MAKE),)
$(warning 1. include $(INCLUDE_PACKAGES))
endif
include $(INCLUDE_PACKAGES)
endif
REQUIRED_PACKAGES	:= $(filter-out $(INCLUDED_PACKAGES), $(REQUIRED_PACKAGES))

ifneq ($(strip $(REQUIRED_PACKAGES)),)

# make list unique
REQUIRED_PACKAGES	:= $(sort $(REQUIRED_PACKAGES))
INCLUDED_PACKAGES	+= $(REQUIRED_PACKAGES)
DEFAULT_PACKAGES	:= $(call createDefaultPackages, $(REQUIRED_PACKAGES))
INCLUDE_PACKAGES	:= $(call createIncludePackages, $(REQUIRED_PACKAGES))
REQUIRED_PACKAGES	:=
ifneq "$(DEFAULT_PACKAGES)" ""
ifneq ($(DEBUG_MAKE),)
$(warning 2. $(MAKDIR)/DefaultPackage.sh: DEFAULT_PACKAGES=$(DEFAULT_PACKAGES))
endif
$(shell $(SHELL) $(MAKDIR)/DefaultPackage.sh "$(DEFAULT_PACKAGES)" $(MAKDIR_ABS) $(BASEDIRABSOLUTE) $(OBJDIRBASE))
# now there are possibly more .mak-files
endif
ifneq ($(INCLUDE_PACKAGES),)
ifneq ($(DEBUG_MAKE),)
$(warning 2. include $(INCLUDE_PACKAGES))
endif
include $(INCLUDE_PACKAGES)
endif
REQUIRED_PACKAGES	:= $(filter-out $(INCLUDED_PACKAGES), $(REQUIRED_PACKAGES))

ifneq ($(strip $(REQUIRED_PACKAGES)),)

# make list unique
REQUIRED_PACKAGES	:= $(sort $(REQUIRED_PACKAGES))
INCLUDED_PACKAGES	+= $(REQUIRED_PACKAGES)
DEFAULT_PACKAGES	:= $(call createDefaultPackages, $(REQUIRED_PACKAGES))
INCLUDE_PACKAGES	:= $(call createIncludePackages, $(REQUIRED_PACKAGES))
REQUIRED_PACKAGES	:=
ifneq "$(DEFAULT_PACKAGES)" ""
ifneq ($(DEBUG_MAKE),)
$(warning 3. $(MAKDIR)/DefaultPackage.sh: DEFAULT_PACKAGES=$(DEFAULT_PACKAGES))
endif
$(shell $(SHELL) $(MAKDIR)/DefaultPackage.sh "$(DEFAULT_PACKAGES)" $(MAKDIR_ABS) $(BASEDIRABSOLUTE) $(OBJDIRBASE))
# now there are possibly more .mak-files
endif
ifneq ($(INCLUDE_PACKAGES),)
ifneq ($(DEBUG_MAKE),)
$(warning 3. include $(INCLUDE_PACKAGES))
endif
include $(INCLUDE_PACKAGES)
endif
REQUIRED_PACKAGES	:= $(filter-out $(INCLUDED_PACKAGES), $(REQUIRED_PACKAGES))

ifneq ($(strip $(REQUIRED_PACKAGES)),)

# make list unique
REQUIRED_PACKAGES	:= $(sort $(REQUIRED_PACKAGES))
INCLUDED_PACKAGES	+= $(REQUIRED_PACKAGES)
DEFAULT_PACKAGES	:= $(call createDefaultPackages, $(REQUIRED_PACKAGES))
INCLUDE_PACKAGES	:= $(call createIncludePackages, $(REQUIRED_PACKAGES))
REQUIRED_PACKAGES	:=
ifneq "$(DEFAULT_PACKAGES)" ""
ifneq ($(DEBUG_MAKE),)
$(warning 4. $(MAKDIR)/DefaultPackage.sh: DEFAULT_PACKAGES=$(DEFAULT_PACKAGES))
endif
$(shell $(SHELL) $(MAKDIR)/DefaultPackage.sh "$(DEFAULT_PACKAGES)" $(MAKDIR_ABS) $(BASEDIRABSOLUTE) $(OBJDIRBASE))
# now there are possibly more .mak-files
endif
ifneq ($(INCLUDE_PACKAGES),)
ifneq ($(DEBUG_MAKE),)
$(warning 4. include $(INCLUDE_PACKAGES))
endif
include $(INCLUDE_PACKAGES)
endif
REQUIRED_PACKAGES	:= $(filter-out $(INCLUDED_PACKAGES), $(REQUIRED_PACKAGES))

ifneq ($(strip $(REQUIRED_PACKAGES)),)

# make list unique
REQUIRED_PACKAGES	:= $(sort $(REQUIRED_PACKAGES))
INCLUDED_PACKAGES	+= $(REQUIRED_PACKAGES)
DEFAULT_PACKAGES	:= $(call createDefaultPackages, $(REQUIRED_PACKAGES))
INCLUDE_PACKAGES	:= $(call createIncludePackages, $(REQUIRED_PACKAGES))
REQUIRED_PACKAGES	:=
ifneq "$(DEFAULT_PACKAGES)" ""
ifneq ($(DEBUG_MAKE),)
$(warning 5. $(MAKDIR)/DefaultPackage.sh: DEFAULT_PACKAGES=$(DEFAULT_PACKAGES))
endif
$(shell $(SHELL) $(MAKDIR)/DefaultPackage.sh "$(DEFAULT_PACKAGES)" $(MAKDIR_ABS) $(BASEDIRABSOLUTE) $(OBJDIRBASE))
# now there are possibly more .mak-files
endif
ifneq ($(INCLUDE_PACKAGES),)
ifneq ($(DEBUG_MAKE),)
$(warning 5. include $(INCLUDE_PACKAGES))
endif
include $(INCLUDE_PACKAGES)
endif
REQUIRED_PACKAGES	:= $(filter-out $(INCLUDED_PACKAGES), $(REQUIRED_PACKAGES))

ifneq ($(REQUIRED_PACKAGES),)

$(error Too many levels of recursion in REQUIRED_PACKAGES. \
The following packages have not been processed:$(REQUIRED_PACKAGES). \
Please append at least one level of recursion in subdir.mak)

endif
endif
endif
endif
endif
endif

ifneq ($(DEBUG_MAKE),)
$(warning Included packages: $(strip $(INCLUDED_PACKAGES)))
endif


###############################################################################
# Library paths
###############################################################################

LIBS				= $(LIBS_PRE)		\
					  $(LIBS_PROJECT)	\
					  $(LIBS_POST1)		\
					  $(LIBS_POST2)		\
					  $(LIBS_POST3)		\
					  $(LIBS_POST4)
# kein := hier! [GZ]

###############################################################################
# Flags used in rules
###############################################################################

CFLAGS				= $(FLAGS_C) $(FLAGS_SO_C) $(CC_OPTIM)		\
					  $(WARNINGS_OFF_C) $(DEFINES)

CXXFLAGS			= $(FLAGS_CXX) $(FLAGS_SO_CXX) $(CC_OPTIM)	\
					  $(WARNINGS_OFF_CXX) $(DEFINES)

LDFLAGS				= $(FLAGS_LD) $(LD_OPTIM) $(WARNINGS_OFF_LD)

LDSOFLAGS			= $(FLAGS_SO_LD) $(LDFLAGS)

###############################################################################
# Default build rules
###############################################################################

.SUFFIXES:	.y .l .cpp .cxx .cc .c .h .hpp	\
			.tab.cpp .lex.cpp				\
			$(OBJSUFFIX) .$(LIBSUFFIX) $(SOSUFFIX)

#--- bison --------------------------------------------------------------------

BISONFLAGS			?= -y -d -v

define bison-compile-msg
	@echo "$(SEP1)"; echo "--- BISONing $< to $@"; echo "$(SEP1)"
endef

define bison-compile
	BISON_SIMPLE=$(LIBDIR_BISON)bison.simple; export BISON_SIMPLE; \
	BISON_HAIRY=$(LIBDIR_BISON)bison.hairy; export BISON_HAIRY; \
	$(BISON) $(BISONFLAGS) -p $(shell echo $< | sed 's/\(...\).*/\1/')_ \
		-b $(OBJDIR)/$(BISONPREFIX)$* $<
	$(MV) $(OBJDIR)/$(BISONPREFIX)$*.tab.c $@
	$(MV) $(OBJDIR)/$(BISONPREFIX)$*.tab.h $(OBJDIR)/$(BISONPREFIX)$*.h
endef

$(OBJDIR)/$(BISONPREFIX)%$(BISONSUFFIX): %.y
	$(bison-compile-msg)
	$(bison-compile)

#--- flex ---------------------------------------------------------------------

FLEXFLAGS			?= -Ce -s -7

define flex-compile-msg
	@echo "$(SEP1)"; echo "--- FLEXing $< to $@"; echo "$(SEP1)"
endef

define flex-compile
	$(FLEX) $(FLEXFLAGS) -t -P$(shell echo $< | sed 's/\(...\).*/\1/')_ $< \
		| sed -e 's/^static \(.*\)()$$/static \1(void)/' \
		> $@
endef

$(OBJDIR)/$(FLEXPREFIX)%$(FLEXSUFFIX): %.l
	$(flex-compile-msg)
	$(flex-compile)

#--- moc (qt meta object compiler) --------------------------------------------

define moc-compile-msg
	@echo "$(SEP1)"; echo "--- Mocing $< to $@"; echo "$(SEP1)"
endef

ifneq "$(OS)" "cygwin11"
MOCPATHPREFIX := -p $(CURDIR)
endif
# '-p $(CURDIR)' is needed because otherwise the generated C source would
# contain #include "../../blub.h", which fails if obj/<platform>
# is a symbolic link! [GZ]
# '-p $(CURDIR)' doesn't work on windows. But we have no links on windows,
# so we can omit the -d option on windows.

define moc-compile
    $(MOC) $(MOCPATHPREFIX) $< -o $@
endef

$(OBJDIR)/$(MOCPREFIX)%.cpp: %.h
	$(moc-compile-msg)
	$(moc-compile)

$(OBJDIR)/$(MOCPREFIX)%.cpp: $(OBJDIR)/%.h
	$(moc-compile-msg)
	$(moc-compile)

#--- uic (qt user interface compiler) -----------------------------------------

define uic-compile-msg
	@echo "$(SEP1)"; echo "--- UICing $< to $@"; echo "$(SEP1)"
endef

define uic-compile-declaration
	$(UIC) -o $@ $<
endef

define uic-compile-implementation
	$(UIC) -o $@ -impl $< $*.ui
endef

$(OBJDIR)/$(UICPREFIX)%$(UICSUFFIX_H): %.ui
	$(uic-compile-msg)
	$(uic-compile-declaration)

$(OBJDIR)/$(UICPREFIX)%$(UICSUFFIX_CPP): $(OBJDIR)/$(UICPREFIX)%$(UICSUFFIX_H) %.ui
	$(uic-compile-msg)
	$(uic-compile-implementation)

#--- C++ compile --------------------------------------------------------------

# Note: Assign these with "?=" such that they can be redefined in makefile
#		includes (else the reread of subdir.mak due to remade *.d files
#		also overwrites any redefinition in mak/*.mak files!).

CXXALLFLAGS			?= $(strip	$(CXXFLAGS) \
								$(CXXLOCALFLAGS) \
								$(CXXINCLUDEPATH))

define cxx-compile-msg
	@echo "$(SEP1)"; echo "--- Compiling $< to $@"; echo "$(SEP1)"
endef

define cxx-compile
	$(CXX) $(CXXALLFLAGS) $(CC_OUT_OPT)$@ -c $<
endef

define cxx-moc-compile
	$(cxx-compile)
endef
# falls man in Zukunft zusaetzliche Flags (z.b. zusaetzliche
# woff's) speziell fuer moc-code angeben moechte/muss.

define cxx-bison-compile
	$(CXX) $(CXXALLFLAGS) $(BISONCXXFLAGS) $(WARNINGS_OFF_BISON) \
		$(CC_OUT_OPT)$@ -c $<
endef

define cxx-flex-compile
	$(CXX) $(CXXALLFLAGS) $(FLEXCXXFLAGS) $(WARNINGS_OFF_FLEX) \
		$(CC_OUT_OPT)$@ -c $<
endef

$(OBJDIR)/$(BISONPREFIX)%$(OBJSUFFIX): $(OBJDIR)/$(BISONPREFIX)%$(BISONSUFFIX)
	$(cxx-compile-msg)
	$(cxx-bison-compile)

$(OBJDIR)/$(FLEXPREFIX)%$(OBJSUFFIX): INCDIRS_C += $(BASEDIR)/internal/include
$(OBJDIR)/$(FLEXPREFIX)%$(OBJSUFFIX): $(OBJDIR)/$(FLEXPREFIX)%$(FLEXSUFFIX)
	$(cxx-compile-msg)
	$(cxx-flex-compile)

$(OBJDIR)/$(MOCPREFIX)%$(OBJSUFFIX): $(OBJDIR)/$(MOCPREFIX)%.cpp
	$(cxx-compile-msg)
	$(cxx-moc-compile)

$(OBJDIR)/$(UICPREFIX)%$(OBJSUFFIX): $(OBJDIR)/$(UICPREFIX)%$(UICSUFFIX_CPP)
	$(cxx-compile-msg)
	$(cxx-compile)

$(OBJDIR)/%$(OBJSUFFIX): %.cpp
	$(cxx-compile-msg)
	$(cxx-compile)

$(OBJDIR)/%$(OBJSUFFIX): %.cxx
	$(cxx-compile-msg)
	$(cxx-compile)

$(OBJDIR)/%$(OBJSUFFIX): %.cc
	$(cxx-compile-msg)
	$(cxx-compile)

#--- C++ dependencies ---------------------------------------------------------

# Note: See note at cxx-compile.

DEP_CXX				= $(CXX)
DEP_CXXFLAGS		= $(FLAGS_CXX) $(FLAGS_SO_CXX) \
					  $(WARNINGS_OFF_DEP) $(DEFINES)
DEP_CXXINCLUDEPATH	= $(CXXINCLUDEPATH)
DEP_CXXALLFLAGS		= $(strip	$(DEPFLAGS_CXX) \
								$(DEP_CXXFLAGS) \
								$(CXXLOCALFLAGS) \
								$(DEP_CXXINCLUDEPATH))

# Note:
# Remove standard include paths and External paths and others,
# and create a dependency line for each remaining include file.
# Caution: remember that $(OBJSUFFIX) contains a '.'! [GZ]
# so $*$(OBJSUFFIX) would also IMRoute, if $* = IM !
# Note on cygwin version: until now, we assume that the remaining paths
# don't contain spaces (MSVC path has been removed by sed).
# Should the need arise, the remaining paths from the while can be 
# piped into a second sed.

ifeq "$(OS)" "cygwin11"
define sed-depend
	sed -e '/:.*\\Microsoft /d' \
		-e '/:.*\\Intel\\/d' \
		-e '/:.*\\stdlib/d' \
		-e '/:.*\\glut/d' \
		-e '/:.*\\common\\/d' | \
	while read -r obj path; \
	do \
		echo $@ $(dir $@)$$obj `cygpath -u "$$path"`; \
	done
endef
else
define sed-depend
	sed -e 's:/usr/include/[^ ]* ::g' \
		-e 's:/usr/lib/[^ ]* ::g' \
		-e 's:[ ^]/.*/OpenSG/[^ ]*[ $$]::g' \
		-e 's:/opt/intel/[^ ]* ::g' \
		-e '/^[ 	]*\\/d' \
		-e '1s"^"$@ $(dir $@)"'
endef
endif

define cxx-depend
	@set -e; \
	doit=0; \
	inclfile=$(subst $(DEPSUFFIX),.din,$@); \
	if [[ ! -f $$inclfile || ! -f $@ ]]; \
	then \
		doit=1; \
		sed -e '/^[ 	]*#[ 	]*include/!d' $< > $$inclfile; \
	else \
		sed -e '/^[ 	]*#[ 	]*include/!d' $< > $$inclfile.new; \
		if [[ ! -s $$inclfile.new ]]; \
		then \
			doit=1; \
		else \
			if diff $$inclfile.new $$inclfile > /dev/null; \
			then \
				doit=0; \
				rm -f $$inclfile.new; \
			else \
				doit=1; \
				rm -f $$inclfile; \
				mv $$inclfile.new $$inclfile; \
			fi; \
		fi; \
	fi; \
	if [[ $$doit = 1 ]]; \
	then \
		echo "$(SEP1)"; echo "--- Generating deps $@"; echo "$(SEP1)"; \
		echo '$(DEP_CXX) $(DEP_CXXALLFLAGS) $<' ; \
		$(DEP_CXX) $(DEP_CXXALLFLAGS) $< | \
			$(sed-depend) \
			> $@; \
	else \
		touch $@; \
	fi
endef

$(OBJDIR)/%$(DEPSUFFIX): $(OBJDIR)/%.cpp
	$(cxx-depend)

$(OBJDIR)/%$(DEPSUFFIX): $(OBJDIR)/%.cxx
	$(cxx-depend)

$(OBJDIR)/%$(DEPSUFFIX): $(OBJDIR)/%.cc
	$(cxx-depend)

$(OBJDIR)/%$(DEPSUFFIX): %.cpp
	$(cxx-depend)

$(OBJDIR)/%$(DEPSUFFIX): %.cxx
	$(cxx-depend)

$(OBJDIR)/%$(DEPSUFFIX): %.cc
	$(cxx-depend)

#--- C compile ----------------------------------------------------------------

# Note: See note at cxx-compile.

# TODO: alles raus, was mit C zu tun hat, und wie C++ behandeln! [GZ]
CALLFLAGS			?= $(strip	$(CFLAGS) \
								$(CLOCALFLAGS) \
								$(CINCLUDEPATH))

define cc-compile-msg
	@echo "$(SEP1)"; echo "--- Compiling $< to $@"; echo "$(SEP1)"
endef

define cc-compile
	$(CC) $(CXXALLFLAGS) $(CC_OUT_OPT)$@ -c $<
endef

$(OBJDIR)/%$(OBJSUFFIX): $(OBJDIR)/%.c
	$(cc-compile-msg)
	$(cc-compile)

$(OBJDIR)/%$(OBJSUFFIX): %.c
	$(cc-compile-msg)
	$(cc-compile)

#--- C dependencies -----------------------------------------------------------

# there should be a way to factor out cc-depend/cxx-depend, since
# they are identical except for $(CALLFLAGS)/$(CXXALLFLAGS)! [GZ]

# Note: See note at cxx-compile.

DEP_CC				= $(CC)
DEP_CFLAGS			= $(FLAGS_C) $(FLAGS_SO_C) \
					  $(WARNINGS_OFF_DEP) $(DEFINES)
DEP_CINCLUDEPATH	= $(CINCLUDEPATH)
DEP_CALLFLAGS		= $(strip	$(DEPFLAGS_C) \
								$(DEP_CFLAGS) \
								$(CLOCALFLAGS) \
								$(DEP_CINCLUDEPATH))

define cc-depend
	@set -e; \
	doit=0; \
	inclfile=$(subst $(DEPSUFFIX),.din,$@); \
	if [[ ! -f $$inclfile || ! -f $@ ]]; \
	then \
		doit=1; \
		sed -e '/^[ 	]*#[ 	]*include/!d' $< > $$inclfile; \
	else \
		sed -e '/^[ 	]*#[ 	]*include/!d' $< > $$inclfile.new; \
		if [[ ! -s $$inclfile.new ]]; \
		then \
			doit=1; \
		else \
			if diff $$inclfile.new $$inclfile > /dev/null; \
			then \
				doit=0; \
				rm -f $$inclfile.new; \
			else \
				doit=1; \
				rm -f $$inclfile; \
				mv $$inclfile.new $$inclfile; \
			fi; \
		fi; \
	fi; \
	if [[ $$doit = 1 ]]; \
	then \
		echo "$(SEP1)"; echo "--- Generating deps $@"; echo "$(SEP1)"; \
		echo '$(DEP_CC) $(DEP_CXXALLFLAGS) $<' ; \
		$(DEP_CC) $(DEP_CXXALLFLAGS) $< | \
			$(sed-depend) \
			> $@; \
	else \
		touch $@; \
	fi
endef

$(OBJDIR)/%$(DEPSUFFIX): $(OBJDIR)/%.c
	$(cc-depend)

$(OBJDIR)/%$(DEPSUFFIX): %.c
	$(cc-depend)

#--- library ------------------------------------------------------------------

define symlink-create
	-$(RM) $@
	$(LNS) $< $@
endef

define lib-create-msg
	@echo "$(SEP2)"; echo "+++ Creating library $@"; echo "$(SEP2)"
endef

define lib-create
	$(AR) $(ARFLAGS) $(AR_OUT_OPT)$@ $(LIBOBJECTS)
endef

ifneq "$(strip $(LIBOBJECTS))" ""

$(OBJDIR)/$(LIBPREFIX)%$(LIBSUFFIX): $(LIBOBJECTS)
	$(lib-create-msg)
	$(lib-create)

%$(LIBSUFFIX): $(OBJDIR)/%$(LIBSUFFIX)
	$(symlink-create)

endif

#--- executable ---------------------------------------------------------------

LDALLFLAGS			= $(strip $(LDFLAGS) \
					  $(LDLOCALFLAGS) \
					  $(LDPATH))

define exe-create-msg
	@echo "$(SEP3)"; echo "=== Linking executable $@"; echo "$(SEP3)"
endef


define exe-create
	$(LD) $(LDALLFLAGS) $< $(filter-out $<, $(LIBOBJECTS)) $(LDLIBS) $(LD_OUT_OPT)$@
endef

$(OBJDIR)/%: $(OBJDIR)/%$(OBJSUFFIX) $(LIBOBJECTS)
	$(exe-create-msg)
	$(exe-create)


%$(EXESUFFIX): $(OBJDIR)/%
	$(symlink-create)
	@if [[ "$(CYGWIN)" = *ntea* ]]; \
	then \
		chmod ug+x $@; \
	fi
	@if [[ "$(AUTOTARGET_WARNING)" != "" ]]; then \
	 	echo "!!! $(AUTOTARGET_WARNING)"; \
	 fi

#--- shared object ------------------------------------------------------------

LDSOALLFLAGS		= $(strip $(LDSOFLAGS) \
					  $(LDSOLOCALFLAGS) \
					  $(LDPATH) )
define so-create-msg
	@echo "$(SEP3)"; echo "=== Linking shared object $@"; echo "$(SEP3)"
endef

define so-create
	$(LD) $(LDSOALLFLAGS) $(LIBOBJECTS) $(LDLIBS) $(LD_OUT_OPT)$@
endef

ifneq "$(strip $(LIBOBJECTS))" ""

$(OBJDIR)/%$(SOSUFFIX): $(LIBOBJECTS)
	$(so-create-msg)
	$(so-create)

%$(SOSUFFIX): $(OBJDIR)/%$(SOSUFFIX)
	$(symlink-create)

endif

###############################################################################
# Default targets
###############################################################################

.PHONY: default \
		default_l

default: default_l

# Note: These are usually names of executables to be built automagically.

ifneq "$(DEFAULT_TARGETS)" ""

DEFAULT_PROGRAMS_OBJ	= $(call addObjectDir, \
						  $(addsuffix $(OBJSUFFIX), $(DEFAULT_PROGRAMS)))
DEFAULT_PROGRAMS_PRG	= $(call addObjectDir, $(DEFAULT_PROGRAMS))

# split DEFAULT_TARGETS into 2 variables, so that we can add the
# EXESUFFIX to the programs [GZ]
DEFAULT_TARGETS_PROG    = $(addsuffix $(EXESUFFIX), \
						   $(filter-out $(INTERNALTARGETS), $(DEFAULT_TARGETS)))
DEFAULT_TARGETS_INTERNAL = $(filter 	$(INTERNALTARGETS), $(DEFAULT_TARGETS))

default_l: chkobjdir \
		   $(DEFAULT_TARGETS_PROG) $(DEFAULT_TARGETS_INTERNAL)

else
ifeq ($(NO_DEFAULT_TARGETS),)

default_l:
	@echo
	@echo "No default targets available. Please define the variable"
	@echo "\tDEFAULT_TARGETS"
	@echo "in your Makefile."
	@echo

endif
endif

###############################################################################
# Automatic target
###############################################################################

# Note: This is used to build an executable based on a source file that is not
#		included in the lib (i.e. listed in EXCLUDED_SOURCES).

ifneq "$(AUTOTARGET)" ""

$(AUTOTARGET): chkobjdir $(AUTOTARGET_PRG)

endif

###############################################################################
# Library targets
###############################################################################

.PHONY: lib   dbglib   optlib \
		lib_l dbglib_l optlib_l

lib dbglib optlib: %: %_l

ifeq "$(NO_LIB_L_TARGET)" ""

lib_l: chkobjdir $(LIBNAME)
	$(create-lastdebug-file)

endif

dbglib_l: DEBUG:=$(DEBUG_MARK)
optlib_l: DEBUG:=$(OPTIM_MARK)
dbglib_l optlib_l:
	$(create-lastdebug-file)
	@$(MAKE) lib_l

###############################################################################
# Shared object targets
###############################################################################

.PHONY: so      dbgso      optso \
		so_l    dbgso_l    optso_l

so dbgso optso: %: %_l

so_l: chkobjdir $(SONAME)
	$(create-lastdebug-file)

dbgso_l: DEBUG:=$(DEBUG_MARK)
optso_l: DEBUG:=$(OPTIM_MARK)

dbgso_l optso_l:
	$(create-lastdebug-file)
	@$(MAKE) so_l

###############################################################################
# Check directory targets
###############################################################################

.PHONY: chkobjdir chkscratchdir

chkobjdir:
	@if [[ ! -d $(OBJDIR) ]]; then \
	   echo ;\
	   echo "The object directory \"$(OBJDIR)\" is missing." ;\
	   echo "To create it, you have two options:" ;\
	   echo ;\
	   echo "  1. Use \"make optinit\" or \"make init\" to create the object directory here." ;\
	   echo ;\
	   echo "  2. Use \"make initscratch\" to create the object directory as a symbolic link" ;\
	   echo "     to your scratch area. Therefore, you first need to create a symbolic link" ;\
	   echo ;\
	   echo "       $(SCRATCHDIR)" ;\
	   echo ;\
	   echo "     pointing to your personal scratch area (usually a directory" ;\
	   echo "     on the local hard disk of your workstation)." ;\
	   echo ;\
	   exit 1 ;\
	 fi

chkscratchdir:
	@if [[ ! -d $(SCRATCHDIR) ]]; then \
		echo ;\
		echo "The scratch directory \"$(SCRATCHDIR)\" is missing." ;\
		echo ;\
		echo "Please create a symbolic link" ;\
		echo ;\
		echo "       $(SCRATCHDIR)" ;\
		echo ;\
		echo "pointing to your personal scratch area (usually a local directory)." ;\
		echo ;\
		echo "See doc/design/make.html for more info.";\
		exit 1 ;\
	 elif [[ ! -w $(SCRATCHDIR) ]]; then \
		echo "The scratch directory \"$(SCRATCHDIR)\" must be writable!"; \
		exit 1; \
	 fi

###############################################################################
# clean
###############################################################################

.PHONY: bisonclean flexclean mocclean uicclean linkclean auxclean \
		clean   dbgclean   optclean \
		clean_l dbgclean_l optclean_l

bisonclean:
ifneq "$(LIBSOURCES_BISON)" ""
	$(RM) $(LIBTARGETS_BISON) $(LIBINTERNAL_BISON)
endif

flexclean:
ifneq "$(LIBSOURCES_FLEX)" ""
	$(RM) $(LIBTARGETS_FLEX)
endif

mocclean:
ifneq "$(LIBSOURCES_MOC)" ""
	$(RM) $(LIBTARGETS_MOC) $(MOCPREFIX)%.cpp
endif

uicclean:
ifneq "$(LIBSOURCES_UIC)" ""
	$(RM) $(LIBTARGETS_UIC) $(LIBHEADERS_UIC)
endif

linkclean:
ifneq "$(DEFAULT_PROGRAMS)" ""
	@-for t in $(DEFAULT_PROGRAMS); \
	  do \
		echo $(RM) $$t; \
			 $(RM) $$t; \
	  done
endif
ifneq "$(strip $(LIBOBJECTS))" ""
	$(RM) $(LIBDIR)/$(SONAMENOPATH)
endif

auxclean:
	$(RM) core so_locations

clean dbgclean optclean: %: %_l

dbgclean_l: DEBUG:=$(DEBUG_MARK)
optclean_l: DEBUG:=$(OPTIM_MARK)

dbgclean_l optclean_l clean_l:
	$(RM) $(ALLOBJECTS)
ifneq ($(II_FILESDIR),)
	$(RMR) $(II_FILESDIR)
endif

###############################################################################
# distclean
###############################################################################

.PHONY: distclean   initclean \
		distclean_l initclean_l

distclean initclean: %: %_l

distclean_l: initclean_l
	@-if [[ -w $(SCRATCHDIR) ]]; then \
		echo rmdir $(SCRATCHOBJDIRDEBUGBASE); \
	    	 rmdir $(SCRATCHOBJDIRDEBUGBASE) >/dev/null 2>&1 || $(ERRIGNORE); \
		echo rmdir $(SCRATCHOBJDIROPTIMBASE); \
	    	 rmdir $(SCRATCHOBJDIROPTIMBASE) >/dev/null 2>&1 || $(ERRIGNORE); \
	  fi
	$(RMR) $(OBJDIRBASE)
	$(RM) .lastdebug*

initclean_l: dbgClean_l optClean_l
	@-if [[ -w $(SCRATCHDIR) ]]; then \
		echo $(RMR) $(SCRATCHOBJDIRDEBUG); \
			 $(RMR) $(SCRATCHOBJDIRDEBUG); \
		echo $(RMR) $(SCRATCHOBJDIROPTIM); \
			 $(RMR) $(SCRATCHOBJDIROPTIM); \
	  fi
	$(RMR) $(OBJDIRDEBUG)
	$(RMR) $(OBJDIROPTIM)
	$(RM) $(LASTDEBUG_FILE)

###############################################################################
# mkobjdir
###############################################################################

.PHONY: mkobjdir mkobjdirscratch

$(OBJDIRBASE):
	@if [[ ! -d $(OBJDIRBASE) ]]; then \
		echo mkdir $(OBJDIRBASE); \
			 mkdir $(OBJDIRBASE); \
	 fi

mkobjdir $(OBJDIR): $(OBJDIRBASE)
	@if [[ -w $(OBJDIR) && -d $(OBJDIR) && ! -L $(OBJDIR) ]]; then \
		echo "$(SEP1)"; \
		echo "--- Use existing obj dir $(CURRENTDIRRELFROMBASE)/$(OBJDIR)"; \
		echo "$(SEP1)"; \
	else \
		if [[ -w $(SCRATCHOBJDIR) && -d $(SCRATCHOBJDIR) ]]; then \
			echo "$(SEP3)"; \
			echo "=== Use existing scratch obj dir"; \
			echo "=== $(SCRATCHOBJDIR)"; \
			echo "=== (to create a normal obj dir, run \"$(MAKE) initclean\" first)"; \
			echo "$(SEP3)"; \
			echo $(RM) $(OBJDIR); \
				 $(RM) $(OBJDIR); \
			echo $(LNS) $(SCRATCHOBJDIR) $(OBJDIR); \
				 $(LNS) $(SCRATCHOBJDIR) $(OBJDIR); \
		else \
			echo "$(SEP4)"; \
			echo "*** Creating new obj dir $(CURRENTDIRRELFROMBASE)/$(OBJDIR)"; \
			echo "$(SEP4)"; \
			echo $(RM) $(OBJDIR); \
				 $(RM) $(OBJDIR); \
			echo mkdir $(OBJDIR); \
				 mkdir $(OBJDIR); \
		fi; \
	fi

mkobjdirscratch: chkscratchdir $(OBJDIRBASE)
	@if [[ -w $(OBJDIR) && -d $(OBJDIR) && ! -L $(OBJDIR) ]]; then \
		echo "$(SEP3)"; \
		echo "=== Use existing obj dir $(CURRENTDIRRELFROMBASE)/$(OBJDIR)"; \
		echo "=== (to create a scratch obj dir, run \"$(MAKE) initclean\" first)"; \
		echo "$(SEP3)"; \
	else \
		if [[ -w $(SCRATCHOBJDIR) && -d $(SCRATCHOBJDIR) ]]; then \
			echo "$(SEP1)"; \
			echo "--- Use existing scratch obj dir"; \
			echo "--- $(SCRATCHOBJDIR)"; \
			echo "$(SEP1)"; \
		else \
			echo "$(SEP4)"; \
			echo "*** Creating new scratch obj dir"; \
			echo "*** $(SCRATCHOBJDIR)"; \
			echo "$(SEP4)"; \
			echo mkdir -p $(SCRATCHOBJDIR); \
				 mkdir -p $(SCRATCHOBJDIR); \
		fi; \
		echo $(RM) $(OBJDIR); \
			 $(RM) $(OBJDIR); \
		echo $(LNS) $(SCRATCHOBJDIR) $(OBJDIR); \
			 $(LNS) $(SCRATCHOBJDIR) $(OBJDIR); \
	fi

###############################################################################
# init
###############################################################################

.PHONY:	init   dbginit   optinit \
		init_l dbginit_l optinit_l \
		initscratch   dbginitscratch   optinitscratch \
		initscratch_l dbginitscratch_l optinitscratch_l

init initscratch: %: dbg%
dbginit optinit dbginitscratch optinitscratch: %: %_l

define create-lastdebug-file
	@echo "DEBUG:=$(DEBUG)" > $(LASTDEBUG_FILE)
endef

dbginit_l: DEBUG := $(DEBUG_MARK)
optinit_l: DEBUG := $(OPTIM_MARK)
dbginit_l optinit_l: mkobjdir linkclean auxclean
	$(create-lastdebug-file)
	@$(MAKE) DEBUG=$(DEBUG) depend_l

dbginitscratch_l: DEBUG := $(DEBUG_MARK)
optinitscratch_l: DEBUG := $(OPTIM_MARK)
dbginitscratch_l optinitscratch_l: mkobjdirscratch linkclean auxclean
	$(create-lastdebug-file)
	@$(MAKE) DEBUG=$(DEBUG) depend_l


###############################################################################
# depend
###############################################################################

.PHONY: depend   dbgdepend   optdepend
		depend_l dbgdepend_l optdepend_l

depend dbgdepend optdepend: %: %_l

ifeq "$(NO_AUTO_DEPEND)" ""

DEP_MAKEFILES		= $(call cnvObjectToDependency, $(LIBOBJECTS))

DEP_MAKEFILES		+= $(call addObjectDir, \
						$(call cnvObjectToDependency, \
						$(call cnvSourceToObject, $(DEFAULT_SOURCES))))

depend_l: $(DEP_MAKEFILES)

# create LIBTARGETS (i.e. output of flex/bison/moc/uic) *before* dependencies
# (allows for correct dependencies even when C/C++ sources are auto-generated)
ifneq "$(LIBTARGETS)" ""
$(DEP_MAKEFILES): $(LIBTARGETS)
endif

endif	# !NO_AUTO_DEPEND

dbgdepend_l: DEBUG := $(DEBUG_MARK)
optdepend_l: DEBUG := $(OPTIM_MARK)
dbgdepend_l optdepend_l:
	@$(MAKE) DEBUG=$(DEBUG) depend_l

###############################################################################
# Clean
###############################################################################

.PHONY: Clean dbgClean optClean \
		Clean_l dbgClean_l optClean_l

Clean dbgClean optClean: %: %_l

dbgClean_l: DEBUG:=$(DEBUG_MARK)
optClean_l: DEBUG:=$(OPTIM_MARK)

dbgClean_l optClean_l Clean_l: \
	clean_l bisonclean flexclean mocclean linkclean auxclean
ifneq ($(LIBNAME),)
	$(RM) $(LIBNAME)
endif
ifneq ($(SONAME),)
	$(RM) $(SONAME)
endif
ifneq ($(DEFAULT_PROGRAMS),)
	$(RM) $(DEFAULT_PROGRAMS_OBJ) $(DEFAULT_PROGRAMS_PRG)
endif
ifneq ($(TEST_PRGS),)
	$(RM) $(filter-out %.sh, $(TEST_PRGS))
endif
ifneq ($(DEP_MAKEFILES),)
	$(RM) $(DEP_MAKEFILES)
	$(RM) $(subst .d,.din,$(DEP_MAKEFILES))
endif

###############################################################################
# installation directories
###############################################################################

$(INSTDIRS):
	mkdir -p $@

###############################################################################
# links
###############################################################################

.PHONY: linkbin linkscript linklib linkplg linkinclude linkdata linkexamples

define symlink-create-msg
	@echo "$(SEP4)"; echo "*** Installing $@"; echo "$(SEP4)"
endef

define symlink-create-absolute
	@if [[ -f $< ]]; then \
		echo $(RM) $@;	\
			 $(RM) $@;	\
		echo $(LNS) $(CURDIR)/$< $@;	\
			 $(LNS) $(CURDIR)/$< $@;	\
	 fi
endef

define symlink-create-samedir
	-$(RM) $@
	$(LNS) $(notdir $<) $@
endef

#--- linkbin: link binaries into projects's platform binary dir ---------------

ifneq ($(DEFAULT_PROGRAMS),)

DEFAULT_PROGRAM_LINKS	= $(addprefix $(BINDIR)/, $(DEFAULT_PROGRAMS))

$(DEFAULT_PROGRAM_LINKS): $(BINDIR)/%: $(OBJDIR)/% $(BINDIR)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linkbin:	$(DEFAULT_PROGRAM_LINKS)

else

linkbin:
	echo "Error in target linkbin: there are no DEFAULT_PROGRAMS to link."
	exit 1

endif

#--- linkscript: link scripts into projects's binary dir ----------------------

ifneq ($(INSTALL_SCRIPTS),)

INSTALL_SCRIPT_LINKS	= $(addprefix $(BINDIR_SCRIPT)/, $(INSTALL_SCRIPTS))

$(INSTALL_SCRIPT_LINKS): $(BINDIR_SCRIPT)/%: % $(BINDIR_SCRIPT)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linkscript:	$(INSTALL_SCRIPT_LINKS)

else

linkscript:
	echo "Error in target linkscript: there are no INSTALL_SCRIPTS to link."
	exit 1

endif

#--- linklib: link shared object into project's platform lib dir --------------

$(LIBDIR)/$(SONAMENOPATH): $(SONAME) $(LIBDIR)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linklib:	$(LIBDIR)/$(SONAMENOPATH)

#--- linkplg: link shared object into project's platform plugins dir (GUI!) ---

$(PLGDIR)/$(SONAMENOPATH): $(SONAME) $(PLGDIR)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linkplg:	$(PLGDIR)/$(SONAMENOPATH)

#--- linkinclude: link includes into projects's platform include dir ----------

ifneq ($(INSTALL_INCLUDES),)

INSTALL_INCLUDE_LINKS	= $(addprefix $(INCDIR)/, $(INSTALL_INCLUDES))

$(INSTALL_INCLUDE_LINKS): $(INCDIR)/%: % $(INCDIR)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linkinclude:	$(INSTALL_INCLUDE_LINKS)

else

linkinclude:
	echo "Error in target linkinclude: there are no INSTALL_INCLUDES to link."
	exit 1

endif

#--- linkdata: link data files into projects's platform data dir --------------

INSTALL_DATA		?= $(call getAllFiles)
INSTALL_DATA_DIR	?= $(DATDIR)/$(patsubst Data/%,%,$(CURRENTDIRRELFROMBASE))
INSTALL_DATA_LINKS	:= $(addprefix $(INSTALL_DATA_DIR)/, $(INSTALL_DATA))

$(INSTALL_DATA_DIR):
	mkdir -p $@

$(INSTALL_DATA_LINKS): $(INSTALL_DATA_DIR)/%: % $(INSTALL_DATA_DIR)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linkdata:	$(INSTALL_DATA_LINKS)

#--- linkexamples: link examples into projects's platform example dir ---------

INSTALL_EXAMPLES		?= $(call getAllFiles)
INSTALL_EXAMPLES_DIR	?= $(EXADIR)/$(patsubst examples/%,%,$(CURRENTDIRRELFROMBASE))
INSTALL_EXAMPLES_LINKS	:= $(addprefix $(INSTALL_EXAMPLES_DIR)/, $(INSTALL_EXAMPLES))

$(INSTALL_EXAMPLES_DIR):
	mkdir -p $@

$(INSTALL_EXAMPLES_LINKS): $(INSTALL_EXAMPLES_DIR)/%: % $(INSTALL_EXAMPLES_DIR)
	$(symlink-create-msg)
	$(symlink-create-absolute)

linkexamples:	$(INSTALL_EXAMPLES_LINKS)

###############################################################################
# Installation
###############################################################################

.PHONY: install \
		install_l

install: install_l

ifneq ($(INSTALL_TARGETS),)

install_l:	$(INSTALL_TARGETS)

else
ifeq ($(NO_INSTALL_TARGETS),)

install_l:
	@echo
	@echo "No installation targets available. Please define the variable"
	@echo "\tINSTALL_TARGETS"
	@echo "in your Makefile."
	@echo

endif
endif

###############################################################################
# Dependencies
###############################################################################

IS_CLEAN_TARGET		:= $(strip \
						   $(filter %clean %clean_l %Clean %Clean_l \
						   			show%vars help%, $(MAKECMDGOALS)) \
						)
# don't load dependency rules,
# if we only want to make clean or see 'make help', etc.

ifeq  "$(NO_AUTO_DEPEND)" ""

ifeq  "$(IS_CLEAN_TARGET)" ""
ifneq "$(wildcard $(OBJDIR))" ""
ifneq "$(DEP_MAKEFILES)" ""
$(DEP_MAKEFILES):
-include $(DEP_MAKEFILES)
ifneq "$(DEBUG_MAKE)" ""
$(warning include $(DEP_MAKEFILES))
endif
endif
endif
endif

endif
