#!/usr/bin/env python
import os
import rospy
from PIL import Image
def di():
    home = os.getenv('HOME')
    map_config_add =home + '/projects/robot/XR03_SOFTWARE/03_Source/04_Robot/catkin_ws/src/nav/maps/map.config'
    electron_map_config_add =home + '/projects/robot/XR03_SOFTWARE/03_Source/04_Robot/catkin_ws/src/nav/maps/elec_map.config'
    map_add =home + '/projects/robot/XR03_SOFTWARE/03_Source/04_Robot/catkin_ws/src/nav/maps/'
    map_wall_add = home + "/projects/robot/XR03_SOFTWARE/03_Source/04_Robot/system/map_wall.txt"
    fileObj = open(map_wall_add,'w')
    if os.path.exists(map_config_add):
        if os.path.exists(electron_map_config_add):
            map_name_file = open(map_config_add,'r')
            electron_map_name_file = open(electron_map_config_add,'r')
            line = map_name_file.readline()
            line_revise = electron_map_name_file.readline()
            if not line:
                exit()
            if not line_revise:
                exit()
            line = line.strip('yaml\n')+"pgm"
            line_revise = line_revise.strip('yaml\n')+"pgm"
            original_map_add = map_add + line
            revise_map_add = map_add + line_revise
            if os.path.exists(original_map_add):
                if os.path.exists(revise_map_add):
                    im1, im2 = Image.open(original_map_add), Image.open(revise_map_add)
                    width, height = im1.size
                    width_electron,height_electron = im2.size
                    if width==width_electron and height==height_electron:
                        fileObj.write(str(height))
                        fileObj.write("\n")
                        for i in range(0, width):
                            for j in range(0, height):
                                if im1.getpixel((i,j))!=im2.getpixel((i,j)):
                                    fileObj.write(str(i))
                                    fileObj.write("\n")
                                    fileObj.write(str(j))
                                    fileObj.write("\n")
    fileObj.write("over")  
    fileObj.close()
    exit()
if __name__ == '__main__':
    try:
        rospy.init_node('elec_map')
        di()
    except rospy.ROSInterruptException:
        pass

