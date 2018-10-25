#!/usr/bin/python

import os
import yaml
import math
import rospy
from PIL import Image
import tf.transformations
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

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

def getPathPositions(im_depths, x_shift, y_shift, max_depth):
    path_data = []

    for y in range(len(im_depths)):
        path_data.append([])
        for x in range(len(im_depths[y])):
            path_data[y].append((x - x_shift, y - y_shift, im_depths[y][x] * max_depth))

    return path_data

def getSphereProjection(point, radius, flat_tooling=True):
    x = point[0]
    y = point[1]
    z = math.sqrt(radius * radius - y * y - x * x) * -1

    if flat_tooling:
        rx = 0.0
        ry = 0.0
        rz = 0.0
        z = z + point[2]
    else:
        rx = 0.0
        ry = 0.0
        rz = 0.0
        z = z + point[2]

    return (x, y, z, rx, ry, rz)

def getSphericalPathPositions(path_data, radius):
    sphere_data = []

    for y in range(len(path_data)):
        sphere_data.append([])
        for x in range(len(path_data[y])):
            sphere_point_data = getSphereProjection(path_data[y][x], radius)
            sphere_data[y].append(sphere_point_data)

    return sphere_data

def getPoseArray(path_data):
    PA = PoseArray()

    for point in path_data:
        new_pose = Pose()
        new_pose.position.x = point[0]
        new_pose.position.y = point[1]
        new_pose.position.z = point[2]
        new_quat = tf.transformations.quaternion_from_euler(point[3], point[4], point[5])
        new_pose.orientation.x = new_quat[0]
        new_pose.orientation.y = new_quat[1]
        new_pose.orientation.z = new_quat[2]
        new_pose.orientation.w = new_quat[3]

        PA.poses.append(new_pose)

    return PA

def main():
    rospy.init_node('image_pather', anonymous=True)
    pub = rospy.Publisher('cut_path', PoseArray, queue_size=10)

    image_file = rospy.get_param('~image_file', "/home/dniewinski/Desktop/HuskySmall.png")
    max_depth = rospy.get_param('~max_depth', 10.0)
    radius = rospy.get_param('~radius', 300.0)

    rospy.loginfo("Reading " + image_file)
    im = Image.open(image_file)
    rospy.loginfo("Read Image <" + str(im.width) + "," + str(im.height) + ">")
    im_depths = getImageData(im)
    path_data = getPathPositions(im_depths, im.width/2.0, im.height/2.0, max_depth)
    sphere_data = getSphericalPathPositions(path_data, radius)

    row_num = 0
    for row in sphere_data:
        if row_num % 2 == 0:
            rospy.loginfo("Sending row " + str(row_num) + " >>>>>")
        else:
            rospy.loginfo("Sending row " + str(row_num) + " <<<<<")
            row.reverse()

        PA = getPoseArray(row)
        pub.publish(PA)
        row_num = row_num + 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
