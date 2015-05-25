# coding=utf8
import sys
import kDopParseDynamic

for model in ['apple', 'bunny2']:
    for i in range(5):
        inputfilename = model + '-k46-model10.' + str(i+1) + '.log'
        kDopParseDynamic.process(inputfilename)



