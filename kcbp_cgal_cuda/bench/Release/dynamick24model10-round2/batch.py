# coding=utf8
import sys
import kDopParseDynamic

for model in ['dinosaur']:#['apple', 'bunny2', 'hand', 'happy_buddha']:
    for i in range(5):
        inputfilename = model + '-k24-model10.' + str(i+1) + '.log'
        kDopParseDynamic.process(inputfilename)



