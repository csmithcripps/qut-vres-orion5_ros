
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)


your_mesh1 = mesh.Mesh.from_file('STLs/wrist.stl')
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh1.vectors))

# Auto scale to the mesh size
scale = your_mesh1.points.flatten(-1)
axes.auto_scale_xyz(scale, scale, scale)






'''
for i in range(16,19):
    filename = 'STLs/Mesh_{}.stl'.format(str(0+i))
    your_mesh = mesh.Mesh.from_file(filename)
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
    # Auto scale to the mesh size
    scale = your_mesh.points.flatten(-1)
    axes.auto_scale_xyz(scale, scale, scale)
'''

# Show the plot to the screen
pyplot.show()
