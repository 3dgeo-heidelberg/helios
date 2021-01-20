#########################
#
# description:
# > generate combined HELIOS survey AND scene XML file containing (1) the object files in a given folder and (2) a circular arrangement of scan positions around the object files
#
# run script with command
# python HELIOS_sceneGenerator_x.py [folder with .obj files] [path to ground plane] [name of generated scene] [number of objects to distribute] [number of circular segments] [radius of scan positions around center of scene]
#
# example:
# python HELIOS_sceneGenerator_x.py C:\HELIOS\data\sceneparts\\test C:\HELIOS\data\sceneparts\\basic\groundplane\groundplane.obj sceneSim01 10 5 15
#
# Resulting .xml file will be saved in same folder as the .obj files
#
#########################
#
# the .xml files generated with this script were last tested for HELIOS version 2016-09-18
#
# workflow of script:
# > list all .obj-files in a given folder
# > write a HELIOS scene XML file with a random distribution of the .obj-files on a grid
# > write meta file containing the local coordinates of the scan positions and the distributed .obj files (scan positions get an ID 1,2,...; .obj file positions all have the ID -1)
#
# XML parameters generated dynamically:
# - the scanned .obj models and their position
# - circular arrangement of given number of scan positions
#
#
# XML parameters NOT generated dynamically:
# - scanning device, platform
# - scanner settings
# - extent and resolution of raster for randomly placing .obj models (see variables coordListX, coordListY)
# - rotation and scaling of .obj models
# 
# Todo 2016-10-20:
# - relative paths to .obj files
# - choice of scan position distribution pattern (circular - raster - ...)
#
#########################