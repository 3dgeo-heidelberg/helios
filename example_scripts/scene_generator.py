#!/usr/bin/python

# (c) 2016 Sebastian Bechtold, Martin Haemmerle, Heidelberg University
# updated 2024-06-07 by Hannah Weiser, Heidelberg University

#########################
#
# description:
# > generate combined HELIOS survey AND scene XML file containing (1) the object files in a given folder and (2) a circular arrangement of scan positions around the object files
#
# run script with command
# python scene_generator.py [folder with .obj files] [path to ground plane] [name of generated scene] [number of objects to distribute] [number of circular segments] [radius of scan positions around center of scene]
#
# example:
# python scene_generator.py data/sceneparts/test data/sceneparts/basic/groundplane/groundplane.obj sceneSim01 10 5 15
#
# Resulting .xml file will be saved in current working directory
#
#########################
#
# the .xml files generated with this script were last tested for HELIOS version 1.3.0
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


import sys
from pathlib import Path
import random
import math

try:
    objdir = sys.argv[1]
    groundplane = sys.argv[2]
    name_scene = sys.argv[3]
    number_objects = int(sys.argv[4])
    number_segments = int(sys.argv[5])
    radius_scan_pos_circle = float(sys.argv[6])
except:
    print("""
#######################
Description:
> generate combined HELIOS survey AND scene XML file containing (1) the object files in a given folder and (2) a circular arrangement of scan positions around the object files

run script with command
python scene_generator.py [folder with .obj files] [path to ground plane] [name of generated scene] [number of objects to distribute] [number of circular segments] [radius of scan positions around center of scene]

example:
python scene_generator.py data/sceneparts/test data/sceneparts/basic/groundplane/groundplane.obj sceneSim01 10 5 15

Resulting .xml file will be saved in current working directory
#########################
""")
    quit()


# list .obj files in given folder
list_objfiles = [p.name for p in Path(objdir).glob("*.obj")]

if len(list_objfiles) == 0:
    print("No .obj files in given folder - stopping script...")
    sys.exit(0)
else:
    print(f"Found {len(list_objfiles)} .obj files in given folder.")

# write first part of XML file containing the scan positions

xml_file = f'{name_scene}_{number_segments}SP_{str(radius_scan_pos_circle).replace(".", "dot")}rad_surveySceneCombi.xml'
datafile = open(xml_file, "w")
metafile = open(
    f'{name_scene}_{number_segments}SP_{str(radius_scan_pos_circle).replace(".", "dot")}rad_surveySceneCombi.xyz',
    "w",
)
metafile.write("ID;X;Y;Z;Name\n")

datafile.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<document>
	<scannerSettings id="scanner1" active="true" pulseFreq_hz="300000" verticalAngleMin_deg="-40.0" verticalAngleMax_deg="60" verticalResolution_deg="0.08" horizontalResolution_deg="0.08" trajectoryTimeInterval_s="0.05"/>
	<survey name="TLS_survey" scene="{xml_file}#{name_scene}" platform="data/platforms.xml#tripod" scanner="data/scanners_tls.xml#riegl_vz400">
""")


# construct circular scan positions
circle_scan_pos_list_x = []
circle_scan_pos_list_y = []
for segment in range(0, number_segments):
    current_angle = segment * (360 / number_segments)
    circle_scan_pos_list_x.append(
        radius_scan_pos_circle * math.cos(math.radians(current_angle))
    )
    circle_scan_pos_list_y.append(
        radius_scan_pos_circle * math.sin(math.radians(current_angle))
    )


distributed_scan_positions = []
for i in range(0, number_segments):

    distributed_scan_positions.append(f"""	
		<leg>
		<platformSettings x="{circle_scan_pos_list_x[i]}" y="{circle_scan_pos_list_y[i]}" z="0" onGround="true"/>
		<scannerSettings template="scanner1" active="true" headRotateStart_deg="0.0" headRotateStop_deg="360.0"/>
		</leg>
		""")
    metafile.write(
        f"{i};{circle_scan_pos_list_x[i]};{circle_scan_pos_list_y[i]};0.0;ScanPos{i}\n"
    )

for entry in distributed_scan_positions:
    datafile.write(entry)


datafile.write("""		
	</survey>
			   """)


# write XML part with ground plane

datafile.write(f"""
	<scene id="{name_scene}" name="scene">		
	
		<part>
			<filter type="objloader">
				<param type="string" key="filepath" value="{groundplane}" />
			</filter>

			<filter type="scale">
				<param type="double" key="scale" value="100" />
			</filter> 
		</part>
		
		""")


# preparing lists for translating objects randomly


def frange(list_name, x, y, jump):
    while x < y:
        list_name.append(x)
        x += jump


coord_list_x = []
frange(coord_list_x, -10.0, 10.0, 0.5)

coord_list_y = []
frange(coord_list_y, -10.0, 10.0, 0.5)

if len(coord_list_x) < number_objects or len(coord_list_y) < number_objects:
    print(
        f"\nUse larger coordinate range or smaller coordinate steps to generate enough coordinates for distributing all {number_objects} objects."
    )
    print("Quit...")
    quit()

transl_x = random.sample(coord_list_x, number_objects)
transl_y = random.sample(coord_list_y, number_objects)


# writing second part of XML file: distributing the .obj files

if number_objects > len(
    list_objfiles
):  # preparing index list in case the given number of objects to distribute is higher than the number of .obj files in the given folder
    if number_objects % len(list_objfiles) == 0:
        list_objfiles = int(number_objects / len(list_objfiles)) * list_objfiles
    else:
        list_objfiles = (int(number_objects / len(list_objfiles)) + 1) * list_objfiles


distributed_objects = []
for i in range(0, number_objects):

    # maybe add up-axis?
    distributed_objects.append(f"""
		<part>
		
			<filter type="objloader">
				<param type="string" key="filepath" value="{objdir + '/' + list_objfiles[i]}"/>
			</filter>

			
			<filter type="translate">
				<param type="vec3" key="offset" value="{transl_x[i]};{transl_y[i]};0.0"/>
			</filter>
			
		</part>
		
""")

    metafile.write(f"-1;{transl_x[i]};{transl_y[i]};0.0;{list_objfiles[i]}\n")


for entry in distributed_objects:
    datafile.write(entry)


datafile.write("""

	</scene>
</document>""")

datafile.close()
metafile.close()
