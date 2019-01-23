from os import listdir
from os.path import isfile, join
import struct
from pyglet.gl import *
from pyglet.window import key
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

'''

Author: Jenna Riseley 2017
(some code copied from Orion5-master repository published by RAWRobotics)

This script should be placed in the Orion5 master directory.
It will steal the models hidden in STLs/3dObjects.SAM and save them as STL files.

'''

# This function has been pasted from the RAWRobotics orion5 repository
# https://rawrobotics.com.au/orion5
def PolyRead(fileName, scaler = 1):
    Models = []
    filePipe = open(fileName, 'rb')
    ModelsNo = struct.unpack('I', filePipe.read(4))[0]
    for iter1 in range(ModelsNo):
        Models.append([[0, [.12, .12, .12, 1.0]], []])
        #Read the Model Header
        #read the ModelNo
        Models[-1][0][0] = struct.unpack('I', filePipe.read(4))[0]
        #read the colour
        for iter2 in range(4):
            Models[-1][0][1][iter2] = struct.unpack('f', filePipe.read(4))[0]
        # Read the number of Polygons
        FacetsNo = struct.unpack('I', filePipe.read(4))[0]
        for data in range(FacetsNo):
            try:
                Normal = [struct.unpack('f', filePipe.read(4))[0],
                          struct.unpack('f', filePipe.read(4))[0],
                          struct.unpack('f', filePipe.read(4))[0]]
                Vertex1 = [struct.unpack('f', filePipe.read(4))[0]*scaler,
                           struct.unpack('f', filePipe.read(4))[0]*scaler,
                           struct.unpack('f', filePipe.read(4))[0]*scaler]
                Vertex2 = [struct.unpack('f', filePipe.read(4))[0]*scaler,
                           struct.unpack('f', filePipe.read(4))[0]*scaler,
                           struct.unpack('f', filePipe.read(4))[0]*scaler]
                Vertex3 = [struct.unpack('f', filePipe.read(4))[0]*scaler,
                           struct.unpack('f', filePipe.read(4))[0]*scaler,
                           struct.unpack('f', filePipe.read(4))[0]*scaler]
                Models[-1][1].append([Normal, Vertex1, Vertex2, Vertex3])
            except:
                print("error reading facets")
                break
    filePipe.close()
    return Models


ModelSets = PolyRead('../stl/3dObjects.SAM', 10)

# Now all 29 models are stored in ModelSets. 
NUM_HIDDEN_MESHES = len(ModelSets)

# Because the numpy stl library can define an object from vertices only, we will only
# extract the vertices and leave calculating the normals to Python.


for MESH_OF_INTEREST in range(NUM_HIDDEN_MESHES):

    facet_count = len(ModelSets[MESH_OF_INTEREST][1]) # number of facets

    
    # Empty list for storing vertices
    vertices = []
    for iterator2 in range(len(ModelSets[MESH_OF_INTEREST][1])):
        for iterator3 in range(1, 4):
            vertices.extend(ModelSets[MESH_OF_INTEREST][1][iterator2][iterator3])



    # This is bastardised code from the controller script.
    # Now what this means is that all the vertices are stored contiguously in a list. I think.
    # Run this and inspect vertices

    # Ok, now all of the vertex information is stored perfectly contiguously in 'vertices'.
    # There are no sublists - the number of rows in 'vertices' is exactly 1.
    # So every FACET is described by NINE entries in the list 'vertices'.
    # Three position vectors for three vertices.

    # Now i need to create a numpy array
    mesh_data = np.zeros(facet_count, dtype=mesh.Mesh.dtype)

    # And for each facet (i2) in the mesh, I need to copy its vertex info into mesh_data[i][1]

    # mesh_data[i2][1][0]: 1st vertex
    # mesh_data[i2][1][1]: 2nd vertex
    # mesh_data[i2][1][2]: 3rd vertex
    for i2 in range(facet_count):
    
        i_vertex = 9*i2 # get starting index in array 'vertices'
        mesh_data[i2][1][0] = vertices[i_vertex:(i_vertex+3)] # 1st vertex in facet
        mesh_data[i2][1][1] = vertices[i_vertex+3:i_vertex+6]
        mesh_data[i2][1][2] = vertices[i_vertex+6:i_vertex+9]


    #data = np.zeros(VERTICE_COUNT, dtype=mesh.Mesh.dtype)
    revealed_mesh = mesh.Mesh(mesh_data, remove_empty_areas=False)

    filename = '../stl/Mesh_{}.stl'.format(str(MESH_OF_INTEREST))
    revealed_mesh.save(filename)



