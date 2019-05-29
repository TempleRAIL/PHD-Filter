fileNameTemplate = 'OSPA_cm{arg1}_{arg2}.txt'
repeat = 5
colNum = 5
minLNum = None
for r in [1,2,3]:
    average = [[] for i in range(colNum)]
    for i in range(1, repeat+1):
        f = open(fileNameTemplate.format(arg1=r, arg2=i), 'r')
        for lineNum, lineTxt in enumerate(f):
            row = [float(x) for x in lineTxt.split(' ')]
            if lineNum == 0:
                timeBase = row[0]-(row[0]%1)
            if lineNum >= len(average[0]):
                [a.append(0) for a in average]
            row[0] -= timeBase
            for col, elm in enumerate(row):
                average[col][lineNum] += (elm/repeat)
        f.close()
        lineNum += 1
        if minLNum is None:
            minLNum = lineNum
        elif lineNum < minLNum:
            minLNum = lineNum
        f = open(fileNameTemplate.format(arg1=r, arg2='average'), 'w')
        for l in range(minLNum):
            for c in average:
                f.write('{} '.format(c[l]))
            f.write('\n')
        f.close()

