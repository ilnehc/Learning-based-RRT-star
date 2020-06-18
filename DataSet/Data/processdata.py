import numpy as np

dataroute = []
for i in range(507):
    print(i)
    routedir = './output_selfmap2_raw/route' + str(i) + '.csv'
    startenddir = './output_selfmap2_raw/startend' + str(i) + '.csv'
    startend = np.loadtxt(startenddir, delimiter = ",", skiprows = 1)
    start = startend[:,0].T
    end = startend[:, 1].T
    route = np.loadtxt(routedir, delimiter = ",", skiprows = 1)
    N = route.shape[0]
    for m in range(0, 15):
        node_temp = int((m + 1) * N / 15) - 1
        temp = []
        for j in range(3):
            temp.append(start[j])
        for j in range(3):
            temp.append(route[node_temp][j])
        for j in range(3):
            temp.append(end[j])
        dataroute.append(temp)

dataroute = np.array(dataroute)
print(dataroute.shape)
np.savez('output', dataroute = dataroute)