#!/usr/bin/python

import os
import yaml
import math
from PIL import Image

max_depth = 10.0
radius = 500.0

def getImageData(im):
    lines = []

    for i in range(im.height):
        lines.append([])

    grey_data = list(im.getdata(band=0))
    count = 0
    for i in grey_data:
        lines[int(count/im.width)].append(float(i)/255.0)
        count = count + 1

    return lines

def getPathPositions(im_depths, y_shift, x_shift):
    path_data = []

    for y in range(len(im_depths)):
        path_data.append([])
        for x in range(len(im_depths[y])):
            path_data[y].append((x - x_shift, y - y_shift, im_depths[y][x] * max_depth))

    return path_data

def getSphereProjection(point, radius):
    x = point[0]
    y = point[1]
    z = math.sqrt(radius * radius - y * y - x * x)

    length = math.sqrt(x*x + y*y + z*z)

    vx = x/length
    vy = y/length
    vz = z/length

    print vx
    return (x, y, z, vx, vy, vz)

def getSphericalPathPositions(path_data):
    sphere_data = []

    for y in range(len(im_depths)):
        sphere_data.append([])
        for x in range(len(im_depths[y])):
            sphere_point_data = getSphereProjection(path_data[y][x], radius)
            sphere_data[y].append(sphere_point_data)

    return path_data

im = Image.open("/home/dniewinski/Desktop/HuskySmall.png")
im_depths = getImageData(im)
path_data = getPathPositions(im_depths, int(im.width/2), int(im.height/2))
sphere_data = getSphericalPathPositions(path_data)

print "Read Image <" + str(im.width) + "," + str(im.height) + ">"
