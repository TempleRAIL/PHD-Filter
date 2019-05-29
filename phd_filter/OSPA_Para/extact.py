from math import isnan
objects = ['chair', 'person', 'table']
objColIdx = {'person':2, 'chair':2, 'table':2}
fileNameTemplate = 'OSPA_{obj}_cm{r}_{i}.txt'
repeat = 5
colNum = 5
for obj in objects:
    colIdx = objColIdx[obj]
    for r in [1,2,3]:
        minLNum = None
        skipNum = 0
        tmp = 0
        average = []
        t = []
        for i in range(1, repeat+1):
            f = open(fileNameTemplate.format(obj=obj, r=r, i=i), 'r')
            for lineNum, lineTxt in enumerate(f):
                row = [x for x in lineTxt.split(' ')]
                row[0] = float(row[0])
                row[colIdx] = float(row[colIdx])  
                if isnan(row[colIdx]):
                    if lineNum > 20:
                        row[colIdx] = tmp # use previous number if we get a nan in the middle of a column
                    else:
                        if lineNum > skipNum:
                            skipNum = lineNum
                if lineNum == 0:
                    timeBase = row[0]-(row[0]%1)
                if lineNum >= len(average):
                    average.append(0)
                    t.append(0)
                row[0] -= timeBase
                t[lineNum] = row[0] #the time axis will be from the last file in the iteration
                #row[colIdx] may be a nan
                average[lineNum] += (row[colIdx]/repeat)
                tmp = row[colIdx]     
            f.close()
            lineNum += 1
            if minLNum is None:
                minLNum = lineNum
            elif lineNum < minLNum:
                minLNum = lineNum
            f = open(fileNameTemplate.format(obj=obj, r=r, i='average'), 'w')
            for l in range(skipNum+1, minLNum):
                for c in [t, average]:
                    f.write('{} '.format(c[l]))
                f.write('\n')
            f.close()

