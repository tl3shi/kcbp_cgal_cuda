# coding=utf8
import sys
import kDopParseDynamic

for model in ['dinosaur']:
    for i in range(5):
        inputfilename = model + '-k24-model2.' + str(i+1) + '.log'
        kDopParseDynamic.process(inputfilename)



