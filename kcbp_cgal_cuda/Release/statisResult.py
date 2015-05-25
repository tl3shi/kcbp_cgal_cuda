subdir = 'dynamick46model10-round2/'
for model in ['dinosaur']:#, 'apple', 'hand', 'happy_buddha']:
    colResultGJK = []
    colResultAABB = []
    header = []
    for i in range(1, 6):
        gjkfilename = subdir + model + '-k46-model10.gjk.' + str(i) + '.log.result.simple.csv'
        aabbfilename = subdir + model + '-k46-model10.AABB.' + str(i) + '.log.result.simple.csv'
        gjklines = open(gjkfilename, 'r').readlines()
        aabblines = open(aabbfilename, 'r').readlines()
        linesLen = len(aabblines)
        if len(header) == 0:
            header = [x.strip() for x in aabblines[0].split(',')]
        if len(gjklines) != len(aabblines):
            print 'Error, when parse model: ' + subdir + model
            exit()
        for lno in range(1, linesLen):
            if i == 1: 
                colResultGJK.append(gjklines[lno].split(','))
                colResultAABB.append(aabblines[lno].split(','))
            else:
                colResultGJK[lno-1] = [float(x)+float(y) for x, y in zip(colResultGJK[lno-1], gjklines[lno].split(','))]
                colResultAABB[lno-1] = [float(x)+float(y) for x, y in zip(colResultAABB[lno-1], aabblines[lno].split(','))]

    for i in range(len(colResultGJK)):
        colResultGJK[i] = [float(x)/5.0 for x in colResultGJK[i]]
        colResultAABB[i] = [float(x)/5.0 for x in colResultAABB[i]]

    resultfilename = subdir + model + '-k46-model10.log.stat.csv'
    resultfile = open(resultfilename, 'w')
    resultfile.write(','.join(header))
    resultfile.write(',,')
    resultfile.write(','.join(header))
    resultfile.write('\n')
    for gjk, aabb in zip(colResultGJK, colResultAABB):
        resultfile.write(','.join(format(x, '.2f') for x in aabb))
        resultfile.write(',,')
        resultfile.write(','.join(format(x, '.2f') for x in gjk))
        resultfile.write('\n')
    print 'output: ' + resultfilename + '\n'

print 'done'
