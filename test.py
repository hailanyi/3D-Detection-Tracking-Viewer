from viewer.viewer import Viewer
import numpy as np

vi = Viewer() # set box_type='OpenPCDet' if you use OpenPCDet boxes
len_dataset = 1000

for i in range(len_dataset):
    pseudo_boxes = np.array([[i*0.05, -1, 1, 1, 1, 1, 0], [i*0.05, 1, 1, 1, 1, 1, 0]]) # your boxes
    ids = np.array([0,1]) # your boxes ids (optional)

    pseudo_points = np.random.randn(100, 3) # your points

    vi.add_points(pseudo_points, radius=4, scatter_filed=pseudo_points[:, 0])
    vi.add_3D_boxes(pseudo_boxes, ids=ids,caption_size=(0.09,0.09))
    vi.add_spheres(pseudo_boxes[:, 0:3],radius=0.03,res=10,color='red',del_after_show=False, alpha=1) # Draw motion track
    vi.show_3D() # press the Q or Enter or ESC key to view