#!/usr/bin/env python

# rrtstar.py
# This program generates a 
# asymptotically optimal rapidly exploring random tree (RRT* proposed by Sertac Keraman, MIT) in a rectangular region.
#
# Originally written by Steve LaValle, UIUC for simple RRT in
# May 2011
# Modified by Md Mahbubur Rahman, FIU for RRT* in
# January 2016

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2
from lineIntersect import *
from readpoints import *
import time
import csv

# constants
# XDIM = 640
# YDIM = 480
XDIM, YDIM, OBS = constants()
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 2000
RADIUS = 15
# OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50)]
startpoint, endpoint, samples = readnpy()
N = len(samples)
p_rand = 1


def obsDraw(pygame, screen):
    blue = (176, 196, 222)
    #OBS1 = [(0, 280, 280, 70), (320, 280, 280, 70), (180, 70, 20, 100), (400, 120, 100, 30)]
    #OBS1 = [(0, 280, 280, 70), (320, 280, 280, 70), (100, 150, 100, 20), (350, 70, 50, 50), (450, 200, 100, 30)]
    for o in OBS:
        pygame.draw.rect(screen, blue, o)
    green = (34, 139, 34)
    #for sample in samples:
    #    pygame.draw.circle(screen, green, (int(sample[0]), int(sample[1])), 0)


def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))


def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)


def chooseParent(nn, newnode, nodes):
    for p in nodes:
        if checkIntersect(p, newnode, OBS) and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and p.cost + dist(
                [p.x, p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
    newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
    newnode.parent = nn
    return newnode, nn


def path_length(nodes):
    length = 0
    for i in range(len(nodes) - 1):
        length += dist([nodes[i].x, nodes[i].y], [nodes[i + 1].x, nodes[i + 1].y])
    return length


def reWire(nodes, newnode, pygame, screen):
    white = 255, 240, 200
    black = 20, 20, 40
    for i in range(len(nodes)):
        p = nodes[i]
        if checkIntersect(p, newnode, OBS) and p != newnode.parent and dist([p.x, p.y], [newnode.x,
                                                                                         newnode.y]) < RADIUS and newnode.cost + dist(
                [p.x, p.y], [newnode.x, newnode.y]) < p.cost:
            pygame.draw.line(screen, white, [p.x, p.y], [p.parent.x, p.parent.y])
            p.parent = newnode
            p.cost = newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y])
            nodes[i] = p
            pygame.draw.line(screen, black, [p.x, p.y], [newnode.x, newnode.y])
    return nodes


def drawSolutionPath(start, goal, nodes, pygame, screen):
    pink = 200, 20, 240
    nn = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
    while nn != start:
        pygame.draw.line(screen, pink, [nn.x, nn.y], [nn.parent.x, nn.parent.y], 5)
        nn = nn.parent


def get_path(start, goal, nodes):
    ret_nodes = []
    nn = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
    while nn != start:
        ret_nodes.append(nn)
        nn = nn.parent
    return ret_nodes


class Cost:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord


class Node:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord


def main(imgno):
    # initialize and prepare screen
    # a=checkIntersect()
    # print(a)
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRTstar')
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame, screen)
    t = time.time()
    nodes = []

    # nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
    #nodes.append(Node(0.0, 0.0))  # Start in the corner
    nodes.append(Node(startpoint[0], startpoint[1]))
    start = nodes[0]
    #goal = Node(630.0, 470.0)
    goal = Node(endpoint[0], endpoint[1])
    for i in range(NUMNODES):
        #rand = Node(random.random() * XDIM, random.random() * YDIM)
        if random.random() < p_rand:
            rand = Node(random.random() * XDIM, random.random() * YDIM)
        else:
            idx = random.randint(0, N - 1)
            rand = Node(samples[idx, 0], samples[idx, 1])
        nn = nodes[0]
        for p in nodes:
            if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn.x, nn.y], [rand.x, rand.y]):
                nn = p
        if dist([nn.x, nn.y], [rand.x, rand.y]) < EPSILON:
            continue
        interpolatedNode = step_from_to([nn.x, nn.y], [rand.x, rand.y])

        newnode = Node(interpolatedNode[0], interpolatedNode[1])
        if checkIntersect(nn, rand, OBS):

            [newnode, nn] = chooseParent(nn, newnode, nodes)

            nodes.append(newnode)
            pygame.draw.line(screen, black, [nn.x, nn.y], [newnode.x, newnode.y])
            nodes = reWire(nodes, newnode, pygame, screen)
            pygame.display.update()
            # print i, "    ", nodes
            if dist([newnode.x, newnode.y], [goal.x, goal.y]) < RADIUS:
                break
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Leaving because you requested it.")
    elapsed = time.time() - t
    expanded = len(nodes)
    path = get_path(start, goal, nodes)
    path_len = path_length(path)
    drawSolutionPath(start, goal, nodes, pygame, screen)
    pygame.display.update()
    print(expanded, elapsed, path_len)
    pygame.image.save(screen, "./images/sample06/rrt-star/random/image" + str(imgno) + ".jpg")
    #with open(r'./result-0423sample10/rrt-star/random-epsilon/rrt-star-0423sample10-random-epsilon.csv', 'a', newline='') as file:
    #    writer = csv.writer(file)
    #    writer.writerow([expanded, elapsed, path_len])
    pygame.quit()


# if python says run, then we should run
if __name__ == '__main__':
    for i in range(20):
        main(i)
        # running = True
        # while running:
        #    for event in pygame.event.get():
        #        if event.type == pygame.QUIT:
        #              running = False
