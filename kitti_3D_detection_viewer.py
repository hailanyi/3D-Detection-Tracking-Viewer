from viewer.viewer import Viewer
import numpy as np

def show():

    points = np.fromfile('000000.bin',np.float32).reshape(-1,4)

    vi = Viewer(box_type='Kitti')

    vi.add_points(points[:,:3])

    box=[[1.404795, 1.612032, 3.772344, 2.994469, 1.532878, 13.169745, -1.570796],
         [1.413269, 1.567278, 3.158158, 2.908125, 1.583429, 19.299001, -1.511817],
         [1.417371, 1.540476, 3.504344, -6.310451, 2.495242, 46.486705, 1.561516]]

    box = np.array(box)
    new_box = np.zeros(shape=box.shape)
    new_box[:,:]=box[:,:]
    new_box[:,3] = box[:,5]
    new_box[:,4] = -box[:,3]
    new_box[:,5] = -box[:,4]
    vi.add_3D_boxes(new_box,ids=[20,13,44],box_info=['Ve: 1','Ve: 2','Ve: 3'])

    vi.show_3D()

if __name__ == '__main__':
    show()