# coding=utf8
import sys

if len(sys.argv) < 2:
    print "input filename"
    exit()

inputfilename = sys.argv[1]
resultfilename = inputfilename + '.result.csv'
inputfile = open(inputfilename, 'r')
result = {} 

features_key  = ['CT(kDop)', 'CT(kCBP)', 'CT(Convexhull)', 'init(AABB)', 'CD(kDop)', 'CD(kCBP)', 'CD(Convexhull)', 'hitRate(kDop)', 'hitRate(kCBP)', 'hitRate(Convexhull)', 'DP(model)']
features_value= ['kDOP for models time', 'kCBP for models time', 'convexhull for models time', 'build AABB time', 'CD time(KDOP_Approximate)', 'CD time(KCBP_GJK)', 'CD time(Convexhull_GJK)', 'KDOP_Approximate hitRate', 'KCBP_GJK hitRate', 'Convexhull_GJK hitRate', 'Collisions of model pairs']

if len(features_key) != len(features_value) :
    print len(features_key); print len(features_value)
    print 'key value Error!'
    exit()

for line in inputfile.readlines():
    if line.find('objfile') >= 0: #find model
        k = line[line.find(':') + 1 : len(line) - 1]
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
resultfile.write('model,')
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
