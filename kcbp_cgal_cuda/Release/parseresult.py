# coding=utf8
import sys

if len(sys.argv) < 2:
    print "input filename"
    exit()

inputfilename = sys.argv[1]
resultfilename = inputfilename + '.result.csv'
inputfile = open(inputfilename, 'r')
result = {} 

features_key  = ['ach', 'cluster', 'projection', 'duality', 'aabbBuild', 'bboxBuild', 'CD(aabb)', 'init(kCBP)', 'CD(kCBP)', 'init(GJK)', 'CD(GJK)', 'count(kCBP)', 'count(GJK)', 'count(real)'] 
features_value= ['ach banchtest', 'cluster time', 'Projection time',  'duality mapping time', 'build AABB time :', 'build boundingbox time', 'total collision time(AABB directly)', 'build AABB time 4 KCBP :', 'total collision time(KCBP AABB filter)', 'init GJK time 4 KCBP', 'total collision time(KCBP GJK filter)', 'kcbp collision size(AABB filter)','kcbp collision size(GJK filter)', 'collision pairs :'] 

if len(features_key) != len(features_value) :
    print 'key value Error!'
    exit()

for line in inputfile.readlines():
    
    if line.find('ModelNum = ') == 0: #find n
        k = line[len('ModelNum = '):len(line)-1]
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



