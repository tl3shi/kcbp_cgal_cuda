#bunny2.ks.3.log.result.simple.csv
#for model in ['bunny2']: #, 'apple', 'budda', 'dinosaur', 'alice']:
for model in ['apple', 'happy_buddha', 'dinosaur', 'hand']:
    header = []
    colResult = []
    for i in range(1, 6):
        filename = model + '.ks.' + str(i) + '.log.result.simple.csv'
        lines = open(filename, 'r').readlines()
        linesLen = len(lines)
        if len(header) == 0:
            header = [x.strip() for x in lines[0].split(',')]
        for lno in range(1, linesLen):
            if i == 1: 
                colResult.append(lines[lno].split(','))
            else:
                colResult[lno-1] = [float(x)+float(y) for x, y in zip(colResult[lno-1], lines[lno].split(','))]

    for i in range(len(colResult)):
        colResult[i] = [float(x)/5.0 for x in colResult[i]]

    resultfilename = model + '.ks.log.stat.csv'
    resultfile = open(resultfilename, 'w')
    resultfile.write(','.join(header))
    resultfile.write('\n')
    for line in colResult:
        resultfile.write(','.join(format(x, '.2f') for x in line))
        resultfile.write('\n')
    print 'output: ' + resultfilename + '\n'

print 'done'
