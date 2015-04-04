#!/bin/bash -p

# Zip the source of CollDet


z="/tmp/CollDet.zip"
rm -f $z

d=`dirname $0`

cd $d/..
zip $z -r CollDet -x '*/.svn/*' \
	-x 'CollDet/zip_it.sh' -x '*/.lastdebug_*' -x '*/obj/*' \
	-x 'CollDet/install*'

echo Created zip archive $z 

