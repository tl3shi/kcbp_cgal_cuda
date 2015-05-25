for model in ['hand', 'bunny2', 'apple', 'happy_buddha']:
#if True:
#    model = 'bunny2'
    colResultkdop = []
    header = []
    for i in range(1, 6):
        kdopfilename = model + '-k24-model2.' + str(i) + '.log.result.csv'
        kdoplines = open(kdopfilename, 'r').readlines()
        linesLen = len(kdoplines)
        if len(header) == 0:
            header = [x.strip() for x in kdoplines[0].split(',')]
        if len(kdoplines) == 0:
            print 'Error, when parse resultfile: ' + kdopfilename 
            exit()
        for lno in range(1, linesLen):
            if i == 1: 
                colResultkdop.append([(x.strip()) for x in kdoplines[lno].split(',') if len(x.strip()) > 0])
            else:
                colResultkdop[lno-1] = [float(x)+float(y) for x, y in zip(colResultkdop[lno-1], kdoplines[lno].split(','))]

    for i in range(len(colResultkdop)):
        colResultkdop[i] = [float(x)/5.0 for x in colResultkdop[i]]

    resultfilename = model + '-k24-model2.log.stat.csv'
    resultfile = open(resultfilename, 'w')
    resultfile.write(','.join(header))
    resultfile.write('\n')
    for kdop in colResultkdop:
        resultfile.write(','.join(format(x, '.2f') for x in kdop))
        resultfile.write('\n')
    print 'output: ' + resultfilename + '\n'

print 'done'
