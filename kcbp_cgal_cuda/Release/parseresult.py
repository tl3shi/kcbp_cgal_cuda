# coding=utf8
import sys

if len(sys.argv) < 2:
    print "input filename"
    exit()

inputfilename = sys.argv[1]
resultfilename = inputfilename + '.result.detail.csv'
inputfile = open(inputfilename, 'r')
result = {} 

features_key  = ['ach', 'cluster', 'projection', 'duality', 'aabbBuild', 'aabbBuildCount', 'init(kCBP)', 'CD(kCBP)', 'init(GJK)', 'CD(GJK)','kcbpModelCheckCount', 'count(Box)', 'count(kCBP)', 'count(GJK)', 'count(real)'] 
features_value= ['ach banchtest', 'cluster time', 'Projection time',  'duality mapping time', 'build AABB time :','build AABB count:', 'build AABB time 4 KCBP :', 'total collision time(KCBP AABB filter)', 'init GJK time 4 KCBP', 'total collision time(KCBP GJK filter)', 'kcbp model check count', 'box collision size:', 'kcbp collision size(AABB filter)','kcbp collision size(GJK filter)', 'collision pairs :'] 

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


simpleresultfilename = inputfilename + '.result.simple.csv'
simpleresultfile = open(simpleresultfilename, 'w')
#header of CSV simple
simpleresultfile.write('n, ConstructKCBP, ConstructModelAABB, init-CD(kCBP-aabb), init-CD(kCBP-GJK), count(kCBP), count(real)\n')

for key in result.keys():
    value = result[key]
    simpleresultfile.write(str(key) + ',')
    simpleresultfile.write(str(float(value['ach']) + float(value['cluster']) + float(value['projection']) + float(value['duality'])) + ',') # ConstructKCBP
    simpleresultfile.write(str(float(value['aabbBuild'])  / float(value['aabbBuildCount']) * float(value['kcbpModelCheckCount'])) + ',') # ConstructModelAABB
    simpleresultfile.write(str(float(value['init(kCBP)']) + float(value['CD(kCBP)'])) + ',') 
    simpleresultfile.write(str(float(value['init(GJK)'])  + float(value['CD(GJK)'])) + ',') 
    simpleresultfile.write(str(value['count(kCBP)']) + ',' + str(value['count(real)']) + '\n') 

simpleresultfile.flush()
simpleresultfile.close()

print 'done.\n'

