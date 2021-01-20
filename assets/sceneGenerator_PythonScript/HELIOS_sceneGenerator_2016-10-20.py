#!/usr/bin/python

# (c) 2016 Sebastian Bechtold, Martin Haemmerle, Heidelberg University

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



import sys, os, random, math


try:
	objdir = sys.argv[1]
	groundplane = sys.argv[2]
	name_scene = sys.argv[3]
	number_objects = int(sys.argv[4])
	numberSegments = int(sys.argv[5])
	radiusScanPosCircle = float(sys.argv[6])
except:
	print '''	
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
'''
	quit()

	
	
	
# list .obj files in given folder

os.chdir(objdir)
foldercontent = os.listdir(objdir)
list_objfiles = []
for file in foldercontent:
	file_ext = file.split(".")[-1] #get file extension of current file
	if file_ext == "obj" and os.path.isfile(file): # check whether item is a file and has right file extension
		list_objfiles.append(file)
		
if len(list_objfiles) == 0:
	print 'No .obj files in given folder - stopping script...'
	quit()

	
	

# write first part of XML file containing the scan positions	

datafile = open(name_scene + '_%02dSP_%srad_surveySceneCombi.xml' % ( numberSegments, str( radiusScanPosCircle ).replace('.','dot') ) ,'w')
metafile = open(name_scene + '_%02dSP_%srad_surveySceneCombi.xyz' % ( numberSegments, str( radiusScanPosCircle ).replace('.','dot') ) ,'w')
metafile.write('''ID;X;Y;Z;Name
''')

datafile.write('''<document>

	<survey defaultScannerSettings="profile1" name="TLS Arbaro" scene="%s" platform="data/platforms.xml#tripod" scanner="data/scanners_tls.xml#riegl_vz400">
''' % ( name_scene ) )




# construct circular scan positions

circleScanPosListX = []
circleScanPosListY = []
for segment in range(0, numberSegments):
	currentAngle = segment * (360 / numberSegments)
	circleScanPosListX.append(radiusScanPosCircle * math.cos(math.radians(currentAngle)))
	circleScanPosListY.append(radiusScanPosCircle * math.sin(math.radians(currentAngle)))


distributed_scanPositions = []
for i in range(0,numberSegments):

	distributed_scanPositions.append('''	
		<leg>
		<platformSettings x="%f" y="%f" z="-5.551115123125783E-17" onGround="false"/>
		<scannerSettings active="true" pulseFreq_hz="100000" scanAngle_deg="180.0" scanFreq_hz="120" headRotatePerSec_deg="10.0" headRotateStart_deg="0.0" headRotateStop_deg="360.0"/>
		</leg>
''' % ( circleScanPosListX[i], circleScanPosListY[i] ) )

	metafile.write('''%i;%f;%f;%f;%s%02d
''' % ( i, circleScanPosListX[i], circleScanPosListY[i], 0.0 , 'ScanPos', i ) )

for entry in distributed_scanPositions:

	datafile.write(entry)


datafile.write('''		
	</survey>
''')




# write XML part with ground plane

datafile.write('''
	<scene id="%s" name="A tree">
		
		<sunDir x="0" y="1" z="-1" />
		<skybox azimuth_deg="275" texturesFolder="textures/sky/sky6_1024"/> 
		
	
		<part>
			<filter type="objloader">
				<param type="string" key="filepath" value="%s" />
			</filter>

			<filter type="scale">
				<param type="double" key="scale" value="100" />
			</filter> 
		</part>
		
		'''	% (name_scene, groundplane) )
		

		
		
		
		
# preparing lists for translating objects randomly

def frange(listName,x, y, jump):
  while x < y:
    listName.append(x)
    x += jump

coordListX = []
frange(coordListX,-10.,10.,.5)

coordListY = []
frange(coordListY,-10.,10.,.5)

if len(coordListX) < number_objects or len(coordListY) < number_objects:
	print
	print 'Use larger coordinate range or smaller coordinate steps to generate enough coordinates for distributing all %i objects.' % (number_objects)
	print 'Quit...'
	quit()

translX = random.sample(coordListX, number_objects )
translY = random.sample(coordListY, number_objects )




# writing second part of XML file: distributing the .obj files

if number_objects > len(list_objfiles): # preparing index list in case the given number of objects to distribute is higher than the number of .obj files in the given folder
	if number_objects % len( list_objfiles ) == 0:
		list_objfiles = int( number_objects / len( list_objfiles ) ) * list_objfiles
	else:
		list_objfiles = ( int( number_objects / len( list_objfiles ) ) + 1 ) * list_objfiles
		
	
distributed_objects = []
for i in range(0,number_objects):

	distributed_objects.append('''
	
		<part>
		
			<filter type="objloader">
				<param type="string" key="filepath" value="%s"/>
				<param type="boolean" key="recomputeVertexNormals" value="true"/>
			</filter>	
			
			<!-- Be aware of transformation filter order: First rotate, THEN translate -->
			
			<filter type="rotate">
				<param type="rotation" key="rotation">				
				<rot axis="pitch" angle_deg="%f"/>
				</param>
			</filter>

			
			<filter type="translate">
				<param type="vec3" key="offset" value="%f;%f;%f"/>						
			</filter>

			<!--
			<filter type="scale">
				<param type="double" key="scale" value="0.75"/>						
			</filter>
			-->
			
		</part>
		
		
''' % (objdir + '\\' + list_objfiles[i], 90, translX[i], translY[i], 0.0 ) )

	metafile.write('''%i;%f;%f;%f;%s
''' % (-1, translX[i], translY[i], 0.0 , list_objfiles[i]) )



for entry in distributed_objects:

	datafile.write(entry)

		
datafile.write('''

	</scene>
</document>''')
			
datafile.close()
metafile.close()



sys.exit(0)
