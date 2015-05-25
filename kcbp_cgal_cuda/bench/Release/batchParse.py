# coding=utf8
import sys
import kDopParseDynamic

subdir = 'dynamick46model10-round2/'
for model in ['dinosaur']:
    for i in range(5):
        inputfilename = subdir + model + '-k46-model10.' + str(i+1) + '.log'
        kDopParseDynamic.process(inputfilename)
