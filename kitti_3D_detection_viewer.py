from viewer.viewer import Viewer
import numpy as np
from dataset.kitti_dataset import KittiDetectionDataset

def kitti_viewer():
    root="data/kitti_detection/training"

    dataset = KittiDetectionDataset(root)

    vi = Viewer(box_type="Kitti")

    for i in range(len(dataset)):
        P2, V2C, points, image, labels, label_names = dataset[i]

        mask = label_names=="Car"
        labels = labels[mask]
        label_names = label_names[mask]

        vi.add_points(points[:,:3],scatter_filed=points[:,2])
        vi.add_3D_boxes(labels,box_info=label_names)
        vi.add_3D_cars(labels, box_info=label_names)
        vi.add_image(image)
        vi.set_extrinsic_mat(V2C)
        vi.set_intrinsic_mat(P2)
        vi.show_2D()
        vi.show_3D()


if __name__ == '__main__':
    kitti_viewer()