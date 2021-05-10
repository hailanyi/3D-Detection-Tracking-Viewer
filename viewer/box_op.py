import numpy as np
from vedo import *

def convert_box_type(boxes,input_box_type = 'Kitti'):
    """
    convert the box type to unified box type
    :param boxes: (array(N,7)), input boxes
    :param input_box_type: (str), input box type
    :return: new boxes with box type [x,y,z,l,w,h,yaw]
    """

    assert  input_box_type in ["Kitti","OpenPCDet"], 'unsupported input box type!'

    if input_box_type == "OpenPCdet":
        return boxes

    if input_box_type == "Kitti": #(h,w,l,x,y,z,yaw) -> (x,y,z,l,w,h,yaw)
        boxes = np.array(boxes)
        new_boxes = np.zeros(shape=boxes.shape)
        new_boxes[:,:]=boxes[:,:]
        new_boxes[:,0:3] = boxes[:,3:6]
        new_boxes[:, 3] = boxes[:, 2]
        new_boxes[:, 4] = boxes[:, 1]
        new_boxes[:, 5] = boxes[:, 0]
        new_boxes[:, 6] = (np.pi - boxes[:, 6]) + np.pi / 2
        new_boxes[:, 2] += boxes[:, 0] / 2
        return new_boxes

def get_mesh_boxes(boxes,colors="red",
                   mesh_alpha=0.4,
                   ids=None,
                   show_ids=False,
                   box_info=None,
                   show_box_info=False,
                   caption_size=(0.05,0.05)):
    """
    convert boxes array to vtk mesh boxes actors
    :param boxes: (array(N,7)), unified boxes array
    :param colors: (str or array(N,3)), boxes colors
    :param mesh_alpha: boxes transparency
    :param ids: list(N,), the ID of each box
    :param show_ids: (bool), show object ids in the 3D scene
    :param box_info: (list(N,)), a list of str, the infos of boxes to show
    :param show_box_info: (bool)，show object infos in the 3D Scene
    :return: (list(N,)), a list of vtk mesh boxes
    """
    vtk_boxes_list = []
    for i in range(len(boxes)):
        box = boxes[i]
        angle = box[6]

        new_angle = (angle / np.pi) * 180

        if type(colors) is str:
            this_c = colors
        else:
            this_c = colors[i]
        vtk_box = Box(pos=(0, 0, 0), height=box[5], width=box[4], length=box[3], c=this_c, alpha=mesh_alpha)
        vtk_box.rotateZ(new_angle)
        vtk_box.pos(box[0], box[1], box[2])

        info = ""
        if ids is not None and show_ids :
            info = "ID: "+str(ids[i])+'\n'
        if box_info is not None and show_box_info:
            info+=str(box_info[i])
        if info !='':
            vtk_box.caption(info,point=(box[0],
                            box[1]-box[4]/4, box[2]+box[5]/2),
                            size=caption_size,
                            alpha=1,c=this_c,
                            font="Calco",
                            justify='cent')
            vtk_box._caption.SetBorder(False)
            vtk_box._caption.SetLeader(False)

        vtk_boxes_list.append(vtk_box)

    return vtk_boxes_list


def get_line_boxes(boxes,
                   colors,
                   show_corner_spheres=True,
                   corner_spheres_alpha=1,
                   corner_spheres_radius=0.3,
                   show_heading=True,
                   heading_scale=1,
                   show_lines=True,
                   line_width=2,
                   line_alpha=1,
                   ):
    """
    get vtk line box actors
    :param boxes: (array(N,7)), unified boxes array
    :param colors: (str or array(N,3)), boxes colors
    :param show_corner_spheres: (bool), show the corner points of box
    :param corner_spheres_alpha: (float), the transparency of corner spheres
    :param corner_spheres_radius: (float), the radius of of corner spheres
    :param show_heading: (bool), show the box heading
    :param heading_scale: (float), the arrow size of heading
    :param show_lines: (bool), show the lines of box
    :param line_width: (float), line width
    :param line_alpha: (float), line transparency
    :return: (list), a list of lines, arrows and spheres of vtk actors
    """
    lines_actors = []
    sphere_actors = []
    arraw_actors = []


    for i in range(len(boxes)):
        box = boxes[i]
        corner_points = []
        corner_points1 = []
        corner_points2 = []
        arraw_points1 = []
        arraw_points2 = []

        angle = box[6]

        new_angle = angle

        transform_mat = np.array([[np.cos(new_angle), -np.sin(new_angle), 0, box[0]],
                                  [np.sin(new_angle), np.cos(new_angle), 0, box[1]],
                                  [0, 0, 1, box[2]],
                                  [0, 0, 0, 1]])
        x = box[3]
        y = box[4]
        z = box[5]

        corner_points.append([-x / 2, -y / 2, -z / 2, 1])
        corner_points.append([-x / 2, -y / 2, z / 2, 1])
        corner_points.append([-x / 2, y / 2, z / 2, 1])
        corner_points.append([-x / 2, y / 2, -z / 2, 1])
        corner_points.append([x / 2, y / 2, -z / 2, 1])
        corner_points.append([x / 2, y / 2, z / 2, 1])
        corner_points.append([x / 2, -y / 2, z / 2, 1])
        corner_points.append([x / 2, -y / 2, -z / 2, 1])

        corner_points1.append([-x / 2, -y / 2, - z / 2, 1])
        corner_points1.append([-x / 2, -y / 2, z / 2, 1])
        corner_points1.append([-x / 2, y / 2, z / 2, 1])
        corner_points1.append([-x / 2, y / 2, -z / 2, 1])
        corner_points1.append([-x / 2, y / 2, z / 2, 1])
        corner_points1.append([-x / 2, -y / 2, z / 2, 1])
        corner_points1.append([x / 2, -y / 2, z / 2, 1])
        corner_points1.append([x / 2, y / 2, z / 2, 1])
        corner_points1.append([-x / 2, -y / 2, -z / 2, 1])
        corner_points1.append([x / 2, -y / 2, -z / 2, 1])
        corner_points1.append([x / 2, -y / 2, z / 2, 1])
        corner_points1.append([-x / 2, -y / 2, z / 2, 1])

        corner_points2.append([x / 2, -y / 2, - z / 2, 1])
        corner_points2.append([x / 2, -y / 2, z / 2, 1])
        corner_points2.append([x / 2, y / 2, z / 2, 1])
        corner_points2.append([x / 2, y / 2, -z / 2, 1])
        corner_points2.append([-x / 2, y / 2, -z / 2, 1])
        corner_points2.append([-x / 2, -y / 2, -z / 2, 1])
        corner_points2.append([x / 2, -y / 2, -z / 2, 1])
        corner_points2.append([x / 2, y / 2, -z / 2, 1])
        corner_points2.append([-x / 2, y / 2, -z / 2, 1])
        corner_points2.append([x / 2, y / 2, -z / 2, 1])
        corner_points2.append([x / 2, y / 2, z / 2, 1])
        corner_points2.append([-x / 2, y / 2, z / 2, 1])

        arraw_points1.append([0, 0, 0, 1])
        arraw_points2.append([x / 2 , 0, 0, 1])

        corner_points = np.matmul(np.array(corner_points), transform_mat.T)
        corner_points1 = np.matmul(np.array(corner_points1), transform_mat.T)
        corner_points2 = np.matmul(np.array(corner_points2), transform_mat.T)
        arraw_points1 = np.matmul(np.array(arraw_points1), transform_mat.T)
        arraw_points2 = np.matmul(np.array(arraw_points2), transform_mat.T)

        if type(colors) is not str:
            this_c = colors[i]
            corner_colors = np.tile(this_c,(corner_points.shape[0],1))
            arraw_colors = np.tile(this_c,(arraw_points1.shape[0],1))

        else:
            this_c = colors
            corner_colors = colors
            arraw_colors = colors

        lines = Lines(corner_points1[:, 0:3], corner_points2[:, 0:3], c=this_c, alpha=line_alpha, lw=line_width)

        corner_spheres = Spheres(corner_points[:,0:3], c= corner_colors, r=corner_spheres_radius,res = 12,alpha=corner_spheres_alpha)

        arraws = Arrows(arraw_points1[:,0:3],arraw_points2[:,0:3],c = arraw_colors,scale=heading_scale)
        lines_actors.append(lines)
        sphere_actors.append(corner_spheres)
        arraw_actors.append(arraws)

    return_list =[]

    if show_corner_spheres:
        return_list+=sphere_actors
    if show_heading:
        return_list+=arraw_actors
    if show_lines:
        return_list+=lines_actors

    return return_list
