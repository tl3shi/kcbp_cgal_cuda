# coding=utf8
import sys
import kDopParseDynamic

if len(sys.argv) < 2:
    print "input filename"
    exit()
inputfilename = sys.argv[1]
kDopParseDynamic.process(inputfilename)



