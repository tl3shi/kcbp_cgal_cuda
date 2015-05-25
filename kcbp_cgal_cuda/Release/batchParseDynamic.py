# coding=utf8
import sys
import kCBPParseDynamic

subdir = 'dynamick46model10-round2/'
for model in ['dinosaur']:#, 'hand', 'happy_buddha']:
    for i in range(5):
        inputfilename = subdir + model + '-k46-model10.AABB.' + str(i+1) + '.log' 
        kCBPParseDynamic.process(inputfilename)
        inputfilename = subdir + model + '-k46-model10.gjk.' + str(i+1) + '.log' 
        kCBPParseDynamic.process(inputfilename)

print 'done.\n'


