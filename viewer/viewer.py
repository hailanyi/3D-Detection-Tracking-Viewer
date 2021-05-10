import numpy as np
from vedo import *
import cv2
import vtk
from .color_map import generate_objects_color_map,generate_objects_colors,generate_scatter_colors
from .box_op import convert_box_type,get_line_boxes,get_mesh_boxes

class Viewer:
    """
    default box type: "OpenPCDet", (x,y,z,l,w,h,yaw)
    """
    def __init__(self,box_type = "OpenPCdet"):
        self.objects_color_map = generate_objects_color_map('rainbow')
        self.box_type = box_type

        # data for rendering in 3D scene
        self.actors = []
        self.actors_without_del = []
        self.tracks_actors_dict = {}

        # data for rendering in 2D scene
        self.cam_intrinsic_mat = None
        self.cam_extrinsic_mat = None
        self.boxes_info = [] # (boxes:array(N,7), ids:array(N,), colors:array(N,3) or str, box_info:list(N,))
        self.points_info = [] # (boxes:array(N,3), colors:array(N,3) or str)
        self.image = None


    def set_ob_color_map(self,color_map_name='rainbow'):
        """
        seting objects colors map, all the options are same as matplotlab.pypot
        reference  https://matplotlib.org/stable/tutorials/colors/colormaps.html
        :param color_map_name: (str), the name of objects color map, such as "rainbow", "viridis","brg","gnuplot","hsv"
        :return: (list), a list of random colors
        """
        self.objects_color_map = generate_objects_color_map(color_map_name)

        return self.objects_color_map

    def set_ego_car(self,ego_car_path = "ego_car.3ds"):
        """
        setting ego car
        :param ego_car_path: (str), path of ego car model
        :return:
        """
        ego_car = load(ego_car_path)
        ego_car.pos(-0.5, 0, -1.6)
        ego_car.scale(0.9)
        self.actors_without_del+=[ego_car]

    def set_intrinsic_mat(self,intrinsic_mat):
        """
        set the camera intrinsic matrix
        :param intrinsic_mat: (array or list(4,4)), intrinsic matrix
        :return:
        """
        self.cam_intrinsic_mat = intrinsic_mat

    def set_extrinsic_mat(self,extrinsic_mat):
        """
        set the camera extrinsic matrix (velo 3D coordinates to cam 3D)
        :param extrinsic_mat: (array or list(4,4)), extrinsic matrix
        :return:
        """
        self.cam_extrinsic_mat = extrinsic_mat

    def add_points(self,points,
                   radius = 2,
                   color = (150,150,150),
                   scatter_filed=None,
                   alpha=1,
                   del_after_show='True',
                   add_to_3D_scene = True,
                   add_to_2D_scene = False):
        """
        add the points actor to viewer
        :param points: (list or array(N,3)),
        :param r: (float), radius of points to show
        :param c: (str,list(N,4),array(N,4)), color name or a list of rgba colors
        :param scatter_filed: (list(N,),array(N,)), scatter filed rendering to colors
        :param alpha:  (float), [0,1] transparency of points actor
        :param del_after_show: (bool), clear the points actor after show
        :param add_to_3D_scene: (bool)
        :param add_to_2D_scene: (bool)
        :return:
        """
        if scatter_filed is not None:
            colors = generate_scatter_colors(scatter_filed)
        else:
            colors = color

        if add_to_2D_scene:
            self.points_info.append((points,colors))

        if add_to_3D_scene:
            if del_after_show:
                self.actors.append(Points(points,r=radius,c=colors,alpha=alpha))

            else:
                self.actors_without_del.append(Points(Points,r=radius,c=colors,alpha=alpha))

    def add_spheres(self,points,
                    radius = 0.3,
                    color='red',
                    res=30,
                    scatter_filed=None,
                    alpha=0.5,
                    del_after_show='True'):
        """
        add the spheres actor to viewer
        :param points: (list or array(N,3)), the centers of spheres
        :param radius: (float), radius of points to show
        :param color: (str,list(N,4),array(N,4)), color name or a list of rgba colors
        :param res: (float), resolution of spheres
        :param scatter_filed: (list(N,),array(N,)), scatter filed rendering to colors
        :param alpha:  (float), [0,1] transparency of points actor
        :param del_after_show: (bool), clear the points actor after show
        :return:
        """

        if scatter_filed is not None:
            colors = generate_scatter_colors(scatter_filed)[:,:3]
        else:
            colors = color

        if del_after_show:
            self.actors.append(Spheres(points,r=radius,res=res,c=colors,alpha=alpha))

        else:
            self.actors_without_del.append(Spheres(Points,r=radius,res=res,c=colors,alpha=alpha))

    def add_3D_boxes(self,boxes=None,
                     ids=None,
                     box_info=None,
                     color="orange",
                     add_to_3D_scene=True,
                     mesh_alpha = 0,
                     show_corner_spheres = True,
                     corner_spheres_alpha = 1,
                     corner_spheres_radius=0.1,
                     show_heading = True,
                     heading_scale = 1,
                     show_lines = True,
                     line_width = 2,
                     line_alpha = 1,
                     show_ids = True,
                     show_box_info=True,
                     del_after_show=True,
                     add_to_2D_scene=True,
                     ):
        """
        add the boxes actor to viewer
        :param boxes: (array(N,7)), 3D boxes
        :param ids: list(N,), the ID of each box
        :param box_info: (list(N,)), a list of str, the infos of boxes to show
        :param color: (str),the default color of boxes
        :param add_to_3D_scene: (bool)
        :param mesh_alpha: (float), the transparency of box mesh
        :param show_corner_spheres: (bool), show the corner points of box
        :param corner_spheres_alpha: (float), the transparency of corner spheres
        :param corner_spheres_radius: (float), the radius of of corner spheres
        :param show_heading: (bool), show the box heading
        :param heading_scale: (float), the arrow size of heading
        :param show_lines: (bool), show the lines of box
        :param line_width: (float), line width
        :param line_alpha: (float), line transparency
        :param show_ids: (bool), show object ids in the 3D scene
        :param show_box_info: (bool)，show object infos in the 3D Scene
        :param del_after_show: (bool), clear the boxes after show
        :param add_to_2D_scene: (bool), add the boxes to images
        :return:
        """
        if boxes is None:
            return
        boxes= convert_box_type(boxes,self.box_type)
        if ids is not None:
            colors = generate_objects_colors(ids,self.objects_color_map)
        else:
            colors = color

        if add_to_2D_scene:
            self.boxes_info.append((boxes,ids,colors,box_info))

        if add_to_3D_scene:
            if del_after_show:
                self.actors += get_mesh_boxes(boxes,colors,mesh_alpha,ids,show_ids,box_info,show_box_info)
                self.actors += get_line_boxes(boxes,
                                              colors,
                                              show_corner_spheres,
                                              corner_spheres_alpha,
                                              corner_spheres_radius,
                                              show_heading,
                                              heading_scale,
                                              show_lines,
                                              line_width,
                                              line_alpha)
            else:
                self.actors_without_del += get_mesh_boxes(boxes,colors,mesh_alpha,ids,show_ids,box_info,show_box_info)
                self.actors_without_del += get_line_boxes(boxes,
                                                          colors,
                                                          show_corner_spheres,
                                                          corner_spheres_alpha,
                                                          corner_spheres_radius,
                                                          show_heading,
                                                          heading_scale,
                                                          show_lines,
                                                          line_width,
                                                          line_alpha)

    def add_3D_cars(self):

        return

    def add_image(self,im):
        """
        add images for display
        :param im: (array(W,H,3)), image array
        :return:
        """
        self.image = im
        return

    def show_3D(self,bg_color=(250,250,250)):
        """
        show objects in 3D scenes, before show_3D, you should add some objects into the current scenes
        :param bg_color: (tuple(3,) or list(3,) or str), background color of 3D scene
        :return:
        """

        show(self.actors+self.actors_without_del,bg=bg_color)

        self.actors.clear()
        self.points_info.clear()
        self.boxes_info.clear()

    def show_2D(self, ):

        assert self.cam_extrinsic_mat is None, "The cam extrinsic matrix is not set!"

        assert self.cam_intrinsic_mat is None, "The cam intrinsic matrix is not set!"

        assert self.image is None, "The image is not added!"
