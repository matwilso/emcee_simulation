#!/usr/bin/env python
# Author: Matt Wilson
import rospy
import xml.etree.ElementTree as ET
import rospkg
import sys


# THIS IS OBSOLETE

# ros package object to find the directory path
rospack = rospkg.RosPack()
path = rospack.get_path('emcee_description')


# def main():
tree = ET.parse(path+"/urdf/emcee.urdf.xacro")
root = tree.getroot()

if len(sys.argv) != 2:
    print "Usage: link"
    print "Link you want to change"
    exit()
link_name = sys.argv[1]

links = root.findall("link")
for link in links:
    if (link.attrib['name'] == link_name):
        arm_link = link

inertial = arm_link.find('inertial')
mass_xml = inertial.find('mass')
mass = float(mass_xml.attrib['value'])
inertia_xml = inertial.find('inertia')
inertia = inertia_xml.attrib
volume = input("Volume (from MeshLab): ")

for key in inertia:
    inertia[key] = str(float(inertia[key]) * mass / volume)

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
