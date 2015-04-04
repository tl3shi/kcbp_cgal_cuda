#!/bin/sh

#  @file
#
#  @brief
#  Shell script to create short id of current operation system
#
#  This shell script is used to create a short id string which uniquely
#  determines the current operating system. It is used to find out which
#  Makefile includes are needed for the standard settings.
#

UNAME_RELEASE=`(uname -r) 2>/dev/null` || UNAME_RELEASE=unknown
UNAME_SYSTEM=`(uname -s) 2>/dev/null` || UNAME_SYSTEM=unknown
#echo "${UNAME_SYSTEM}:${UNAME_RELEASE}"

case "${UNAME_SYSTEM}:${UNAME_RELEASE}" in
	Linux:1*)
		# Linux with glibc1 (Kernel 1.x)
		echo "linux1"
		exit 0
		;;
	Linux:2*)
		# Linux with glibc2 (Kernel 2.x)
		echo "linux2"
		exit 0
		;;
	IRIX*:6.2)
		echo "irix62"
		exit 0
		;;
	IRIX*:6.5)
		echo "irix65"
		exit 0
		;;
	SunOS:5.7)
		echo "solaris27"
		exit 0
		;;
	CYGWIN*:1.1.*)
		echo "cygwin11"
		exit 0
		;;
	HP-UX:*11.00)
		echo "hpux11"
		exit 0
		;;
	Darwin:*)
		echo "macosx"
		exit 0
		;;
esac

echo "unknown"
exit 1
