# coding=utf8
import sys

if len(sys.argv) < 2:
    print "input filename"
    exit()

inputfilename = sys.argv[1]
resultfilename = inputfilename + '.result.csv'
inputfile = open(inputfilename, 'r')
result = {} 

features_key  = ['init(kDOPTree)', 'CD(kDOPTree)', 'count(Box)', 'count(real)', 'dopBuildCount'] 
features_value= ['build time(kDOP):', 'total collision time(kDOP)', 'box collision size', 'collision pairs', 'build dop num needed:']

if len(features_key) != len(features_value) :
    print 'key value Error!'
    exit()

for line in inputfile.readlines():
    
    if line.find('plus fixed one,total') > 0: #find n
        k = line.split('=')[1][:-2]
        if k not in result.keys():
            result[k] = {}
            for key in features_key:
                result[k][key] = 1
    for i in range(len(features_key)):
        if line.find(features_value[i]) >= 0:
            tmp = line[line.find(':') + 1 : len(line) - 1]
            result[k][features_key[i]] = tmp

print result
resultfile = open(resultfilename, 'w')

#header of CSV
resultfile.write('n,')
for key in features_key:
    resultfile.write(str(key) + ',')
resultfile.write('\n')

for key in result.keys():
    value = result[key]
    resultfile.write(str(key) + ',')
    for i in range(len(features_key)):
        resultfile.write(str(value[features_key[i]]) + ',')
    resultfile.write('\n')

resultfile.flush()
resultfile.close()

print 'done.\n'



