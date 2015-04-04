#! @file
#
#  @brief
#  Standard include for toplevel makefiles
#
#  This file must be included in directories with further subdirectories.
#

###############################################################################
# Check that common.mak was included
###############################################################################

ifneq "$(HAVE_COMMON_MAK)" "1"
$(error common.mak has to be included first)
endif

###############################################################################
# Internal targets (needed to extract automatic targets in subdir.mak)
###############################################################################

INTERNALTARGETS		+= $(RECURSIVE_TARGETS) \
					   $(RECURSIVE_TARGETS_R) \
					   $(SUBDIR_TARGETS) \
					   default \
					   default_r \
					   $(SUBDIRS) \
					   commit update \
					   help help_top

###############################################################################
# Automatic subdir definition when SUBDIRS is empty
###############################################################################

ifeq ($(SUBDIRS),)

# .svn, ., and obj are never reasonable subdirs
EXCLUDED_SUBDIRS	+= .svn obj .

SUBDIRS_ALL			:= $(shell ls -1F | grep '/$$' | sed -e 's:/$$::')

SUBDIRS				:= $(foreach s, $(SUBDIRS_ALL), \
						$(if $(wildcard $(s)/Makefile),$(s)) )

SUBDIRS				:= $(filter-out $(EXCLUDED_SUBDIRS), $(SUBDIRS))

endif

###############################################################################
# Recursive targets
###############################################################################

RECURSIVE_TARGETS		= default lib dbglib optlib so dbgso optso clean dbgclean optclean Clean dbgClean optClean distclean initclean init dbginit optinit initscratch dbginitscratch optinitscratch depend dbgdepend optdepend tests test_progs test_runs test_files install

RECURSIVE_TARGETS_R		= $(addsuffix _r, $(RECURSIVE_TARGETS))

# Note: These are already handled in common.mak.
EXCLUDED_SUBDIR_TARGETS	= test.tests test.test_progs \
						  test.test_runs test.test_files 

# For each subdir listed in SUBDIRS and each target listed in RECURSIVE_TARGETS
# create a new target <subdir>/<target>.

SUBDIR_TARGETS			= $(filter-out $(EXCLUDED_SUBDIR_TARGETS), \
							$(foreach s, $(SUBDIRS), \
								$(addprefix $(s)., $(RECURSIVE_TARGETS))))

.PHONY: $(RECURSIVE_TARGETS) $(RECURSIVE_TARGETS_R)
.PHONY: $(SUBDIR_TARGETS) $(EXCLUDED_SUBDIR_TARGETS)
.PHONY: default default_r $(SUBDIRS)

$(RECURSIVE_TARGETS_R): %_r: $(foreach s, $(SUBDIRS), $(addprefix $(s)., %))

$(RECURSIVE_TARGETS): %: %_r

$(SUBDIR_TARGETS):
	$(MAKE) -C $(wordlist 1,2,$(subst ., ,$@))

# Note: These are just no-ops to satisfy make ...
$(EXCLUDED_SUBDIR_TARGETS):

default:	default_r

default_r: $(addsuffix .default, $(SUBDIRS))

$(SUBDIRS):
	$(MAKE) -C $@

###############################################################################
# Joined library
###############################################################################

JOINLIBOBJS := $(addsuffix /$(OBJDIR)/*, $(SUBDIRS) )
JOINLIBOBJS := $(filter-out $(EXCLUDED_JOINLIB_OBJS), $(JOINLIBOBJS))
JOINLIBOBJS += $(INCLUDED_JOINLIB_OBJS)
JOINLIBNAME	?= $(call createLibName, $(CURRENTDIR))

joinlibs:
	@for d in $(SUBDIRS); \
	do \
		$(MAKE) -C $$d lib; \
	done
	@$(PRINTF) "$(SEP2)\n+++ %-71s +++\n$(SEP2)\n" "Creating library $(JOINLIBNAME)"
	$(AR) $(ARFLAGS) $(JOINLIBNAME) $(JOINLIBOBJS)

.PHONY: joinlibs

###############################################################################
# Command to generate sub-directory dependencies
###############################################################################

# Note: The following dependencies are generated with this command:
#
#		$(1): $(2)
#		$(1).<target>: $(2).<target>
#
#		$(2) can be a list of directories where the suffix .<target> is added
#		to each member of the list

subdir_depend		= $(1) $(foreach t, $(RECURSIVE_TARGETS), \
								$(addsuffix .$(t), $(1))): \
							   		$(1)%: $(addsuffix %, $(2))


