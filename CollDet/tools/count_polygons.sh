#!/bin/bash -p

# Count number of polygons in a VRML file
# 
# Counts the true number of polygons in a VRML file (osgSceneViewer
# instead counts the number of *triangles* it has *rendered*, which can
# be significantly more!)
#
# Bug: not fool-proof! (it's not a VRML parser)
# Note: the escaped newline must be preserved.
#
# GZ 2007

sed -e '/coordIndex \[/,/]/!d' "$1" |
sed 's/-1,[ 	]*/-1,\
/g' |
sed '/-1/!d' |
wc -l

