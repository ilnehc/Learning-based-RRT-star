import numpy as np


def constants():
    XDIM = 600
    YDIM = 350
    #XDIM = 400
    #YDIM = 250
    OBS = [(0, 200, 100, 50), (300, 200, 100, 50)]

    #OBS = [(0, 280, 280, 70), (320, 280, 280, 70), (175, 65, 30, 110), (395, 115, 110, 40)]
    OBS = [(0, 280, 280, 70), (320, 280, 280, 70), (95, 145, 110, 30), (345, 65, 60, 60), (445, 195, 110, 40)]
    return XDIM, YDIM, OBS

def readnpy():
    XDIM = 600
    YDIM = 350
    #XDIM = 400
    #YDIM = 250
    #points = np.load('sample06.npy')
    points = np.load('0423_back_sample10.npy')
    points = togamecoor(points, XDIM, YDIM)
    start = points[0, :]
    end = points[1, :]
    samples = points[2:, :]
    #print(start, end)
    return start, end, samples


def togamecoor(points, XDIM=400, YDIM=250):
    points[:, 0] = XDIM / 2 + points[:, 0] * 10
    points[:, 1] = YDIM - points[:, 1] * 10
    points[:, 2] = np.arctan2(np.sin(points[:, 2]), np.cos(points[:, 2])) + np.pi
    points[:, 2] = points[:, 2] * 180 / np.pi
    return points


def tomapcoor(points, XDIM=400, YDIM=250):
    points[:, 0] = (points[:, 0] - XDIM/2) / 10
    points[:, 1] = (YDIM - points[:, 1]) / 10