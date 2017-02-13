# https://docs.blender.org/api/blender_python_api_2_72_release/info_quickstart.html

import bpy
import math
import itertools
import os
import numpy as np


os.chdir(os.path.dirname(__file__))
print(os.getcwd())
location = os.getcwd() + '/'
name = '.png'

os.chdir(os.path.dirname(__file__))
print(os.getcwd())

scene = bpy.data.scenes['Scene']


def returnObjectByName (passedName= ""):
    r = None
    obs = bpy.data.objects
    for ob in obs:
        if ob.name == passedName:
            r = ob
    return r

def renderToImage(location, index, name):
	fullPath = location + index + name
	#print('Rendering image ' + fullPath)
	scene.render.filepath = fullPath
	bpy.ops.render.render( write_still=True )

vw = returnObjectByName('VW Coccinelle')
camera = returnObjectByName('Camera')
lamp = returnObjectByName('Lamp')

# VW
# 
vw.location =[10.0, -10.0, 2.2]
vw.rotation_euler = [math.radians(0) , math.radians(0), math.radians(0) ]



#renderToImage(location, 'Spoel'   + sSpoelLoc + ' ' + 'Camera'  + sCameraLoc + ' ' + 'LampPos '  + str(camPosIndex)  ,  name)


