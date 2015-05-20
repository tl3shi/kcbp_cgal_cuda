# coding=utf8
import sys

if len(sys.argv) < 2:
    print "input filename"
    exit()

inputfilename = sys.argv[1]
resultfilename = inputfilename + '.result.csv'
inputfile = open(inputfilename, 'r')
result = {} 

features_key  = ['CT(Box)', 'CT(Sphere)', 'CT(kDop)', 'CT(kCBP)', 'CT(Convexhull)', 'init(Box)', 'init(Sphere)', 'init(kDop)', 'init(kCBP)', 'init(Convexhull)', 'CD(Box)', 'CD(Sphere)', 'CD(kDop)', 'CD(kCBP)', 'CD(Convexhull)', 'hitRate(Box)', 'hitRate(Sphere)', 'hitRate(kDop)', 'hitRate(kCBP)', 'hitRate(Convexhull)', 'initAfterBox(kDop)', 'initAfterBox(kCBP)', 'initAfterBox(Convexhull)', 'CDAfterBox(kDop)', 'CDAfterBox(kCBP)', 'CDAfterBox(Convexhull)', 'ModelCDNum'] 
features_value= ['boundingbox for models time', 'boundingSphere for models time', 'kDOP for models time', 'kCBP for models time', 'convexhull for models time', 'Box init time', 'Sphere init time', 'kDOP init time', 'kCBP init time', 'Convexhull init time', 'Box directly cd time', 'Sphere directly cd time', 'kDOP cd time', 'kCBP cd time', 'Convexhull cd time', 'Box hit rate', 'Sphere hit rate', 'kDOP hit rate', 'kCBP hit rate ', 'Convexhull hit rate', 'kDOP init time(AfterBox)', 'kCBP init time(AfterBox)', 'Convexhull init time(AfterBox)', 'kDOP cd time(AfterBox)', 'kCBP cd time(AfterBox)', 'Convexhull cd time(AfterBox)','Model collision pair by Box'] 

if len(features_key) != len(features_value) :
    print len(features_key); print len(features_value)
    print 'key value Error!'
    exit()

for line in inputfile.readlines():
    if line.find('models/') >= 0: #find model
        k = line[7:line.find('.obj')]
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
