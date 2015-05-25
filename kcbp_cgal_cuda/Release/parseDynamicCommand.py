# coding=utf8
import sys
import kCBPParseDynamic

if len(sys.argv) < 2:
    print "input filename"
    exit()

inputfilename = sys.argv[1]
kCBPParseDynamic.process(inputfilename)

print 'done.\n'

