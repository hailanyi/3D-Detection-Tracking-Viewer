import numpy as np
from vedo import *
import cv2
import vtk


def get_light(pos=(0,0,0),focalPoint=(0,0,0)):
    light=vtk.vtkLight()

    light.SetPosition(pos)
    light.SetFocalPoint(focalPoint)
    light.SetIntensity(0.65)

    return light

class VTKViewer():

    def __init__(self,add_ego_car=False,ego_path="ego_car.3ds",box_type="OpenPCDet"):
        self.VTK_actor_clear=[]
        self.VTK_actor=[]
        self.ego_car=None
        self.model=box_type
        self.vi = Plotter(bg=(245,245,245))
        self.car_dict={}
        self.bbs_dict={}
        self.tracks_dict={}
        self.tracks_colors={}

        self.light_actors=[]

        self.light_actors.append(get_light(pos=(400, 400, 100), focalPoint=(0, 0, 0)))
        self.light_actors.append(get_light(pos=(-400, 400, 100), focalPoint=(0, 0, 0)))
        self.light_actors.append(get_light(pos=(400, -400, 100), focalPoint=(0, 0, 0)))
        self.light_actors.append(get_light(pos=(-400, -400, 100), focalPoint=(0, 0, 0)))
        for a in self.light_actors:
            self.vi.renderer.AddLight(a)

        if add_ego_car:
            self.ego_car=load(ego_path)
            self.ego_car.pos(-0.5,0,-1.6)
            self.ego_car.scale(0.9)

        if self.ego_car is not None:
            self.VTK_actor+=[self.ego_car]

    def _get_vtk_boxes(self, boxes,color="red",alpha=0.4,model="Waymo"):

        vtk_boxes_list = []

        for i in range(len(boxes)):
            box=boxes[i]
            angle=box[6]
            new_angle=0
            z=0
            if model=="Waymo" or model == "OpenPCDet":
             new_angle = ( angle/ np.pi) * 180
             z = box[5]
            if model == "Kitti":
                new_angle=(np.pi-angle)+np.pi/2
                new_angle = (new_angle / np.pi) * 180
                z = box[5]+box[0]/2

            if type(color) is str :
                this_c=color
            else:
                this_c=color[i]

            vtk_box = Box(pos=(0, 0, 0), height=box[0], width=box[1], length=box[2], c=this_c, alpha=alpha)
            vtk_box.rotateZ(new_angle)
            vtk_box.pos(box[3], box[4], z)

            vtk_boxes_list.append(vtk_box)

        return vtk_boxes_list

    def _get_corner_arraw_points(self,boxes,color="red",color_arraw="red",radius=4,alpha=0.4,model="Waymo",l_alpha=1.,lw=1,arraw_scale=1.5):

        point_list=[]

        array_list1 = []
        array_list2 = []
        lines_actors=[]

        for i in range(len(boxes)):
            box=boxes[i]
            corner_points=[]
            corner_points1 = []
            corner_points2 = []
            arraw_points1=[]
            arraw_points2=[]

            angle = box[6]
            new_angle = 0
            trans_z=0
            if model == "Waymo" or model == "OpenPCDet":
                new_angle = angle
                trans_z = box[5]
            if model == "Kitti":
                new_angle = (np.pi - angle) + np.pi / 2
                trans_z = box[5]+box[0]/2


            transform_mat=np.array([[np.cos(new_angle), -np.sin(new_angle), 0, box[3]],
                                   [np.sin(new_angle), np.cos(new_angle), 0, box[4]],
                                   [0, 0, 1, trans_z],
                                   [0, 0, 0, 1]])
            x=box[2]
            y=box[1]
            z=box[0]

            corner_points.append([-x / 2, -y / 2, -z / 2,1])
            corner_points.append([-x / 2, -y / 2, z / 2,1])
            corner_points.append([-x / 2, y / 2, z / 2,1])
            corner_points.append([-x / 2, y / 2, -z / 2,1])
            corner_points.append([x / 2, y / 2, -z / 2, 1])
            corner_points.append([x / 2, y / 2, z / 2, 1])
            corner_points.append([x / 2, -y / 2, z / 2, 1])
            corner_points.append([x / 2, -y / 2, -z / 2,1])


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

            arraw_points1.append([x / 2, 0, 0,1])
            arraw_points2.append([x / 2+2.4, 0, 0,1])

            corner_points=np.matmul(np.array(corner_points),transform_mat.T)
            corner_points1 = np.matmul(np.array(corner_points1), transform_mat.T)
            corner_points2 = np.matmul(np.array(corner_points2), transform_mat.T)
            arraw_points1 = np.matmul(np.array(arraw_points1), transform_mat.T)
            arraw_points2 = np.matmul(np.array(arraw_points2), transform_mat.T)
            array_list1.append(arraw_points1[:,0:3])
            array_list2.append(arraw_points2[:, 0:3])
            point_list.append(corner_points[:,0:3])

            if type(color_arraw) is not str:
                this_c=color_arraw[i]
            else:
                this_c=color

            lines_actors.append(Lines(corner_points1[:,0:3], corner_points2[:,0:3], c=this_c, alpha=l_alpha, lw=lw))

        points=np.concatenate(point_list,0)
        arraw1=np.concatenate(array_list1,0)
        arraw2 = np.concatenate(array_list2, 0)


        return Spheres(points,c=color,r=radius,alpha=alpha,res=10),Arrows(arraw1,arraw2,c=color_arraw,alpha=1,scale=arraw_scale),lines_actors



    def add_points(self,points,color=[100,100,100],radius=2,alpha=1,clear=True):
        points=np.array(points)
        if clear==True:
            self.VTK_actor_clear+=[Points(points[:,:3],c=color,r=radius,alpha=alpha)]
        else:
            self.VTK_actor+=[Points(points[:,:3],c=color,r=radius,alpha=alpha)]

    def add_spheres(self,points,color="yellow",radius=0.35,alpha=1,res=4,clear=True):
        points=np.array(points)
        if clear==True:
            self.VTK_actor_clear+=[Spheres(points[:,:3],c=color,r=radius,alpha=alpha,res=res)]
        else:
            self.VTK_actor+=[Spheres(points[:,:3],c=color,r=radius,alpha=alpha,res=res)]


    def add_boxes(self,BBs,
                  color="orange",
                  alpha=0.,
                  if_corner_points=False,
                  corner_color="orange",
                  corner_radius=0.2,
                  corner_alpha=1,
                  if_arraws=False,
                  if_lines=True,
                  l_alpha=1,
                  lw=2,
                  clear=True,
                  add_dict=True,
                  arraw_scale=1.):

        BBs = np.array(BBs)
        if BBs.shape[0] == 0:
            return
        if self.model=="OpenPCDet":
            new_BBs=np.zeros(shape=BBs.shape)
            new_BBs[:,3:6]=BBs[:,0:3]
            new_BBs[:,0]=BBs[:,4]
            new_BBs[:, 1] = BBs[:, 5]
            new_BBs[:, 2] = BBs[:, 3]
            new_BBs[:, 6] = BBs[:, 6]
            BBs=new_BBs

        if clear:
            self.VTK_actor_clear+=self._get_vtk_boxes(BBs,color=color,alpha=alpha,model=self.model)
        else:
            self.VTK_actor += self._get_vtk_boxes(BBs, color=color, alpha=alpha, model=self.model)

        cornerPoints,arraws,lines=self._get_corner_arraw_points(BBs,
                                                          color=corner_color,
                                                          color_arraw=color,
                                                          radius=corner_radius,
                                                          alpha=corner_alpha,
                                                          model=self.model,
                                                          l_alpha=l_alpha,
                                                          lw=lw,
                                                          arraw_scale=arraw_scale)

        if if_corner_points and clear:
            self.VTK_actor_clear+=[cornerPoints]
        elif if_corner_points and not clear:
            self.VTK_actor += [cornerPoints]
        if if_arraws and clear:
            self.VTK_actor_clear+=[arraws]
        elif if_arraws and not clear:
            self.VTK_actor += [arraws]
        if if_lines and clear:
            self.VTK_actor_clear += lines
        elif if_lines and not clear:
            self.VTK_actor+= lines
        if add_dict:
            for i in range(len(BBs)):
                bb=BBs[i]
                id=bb[9]

                if id in self.tracks_dict.keys():
                    self.tracks_dict[id].append(bb)
                else:
                    if color is list:
                        self.tracks_colors[id] = color[i]
                    else:
                        self.tracks_colors[id] = color
                    self.tracks_dict[id] = [bb]

    def show_last_box_in_tracks(self):
        all_box=[]
        colors=[]
        for id in self.tracks_dict.keys():
            track=self.tracks_dict[id]
            all_box.append(track[-1])
            colors.append(self.tracks_colors[id])
        self.add_boxes(np.array(all_box),color=colors,clear=True,alpha=0.1)

    def show_last_car_in_tracks(self):
        all_box = []
        colors = []
        for id in self.tracks_dict.keys():
            track = self.tracks_dict[id]
            all_box.append(track[-1])
            colors.append(self.tracks_colors[id])
        self.add_cars(np.array(all_box), color=colors, clear=True, alpha=1)

    def add_cars(self,BBs,color=[],clear=True,alpha=1):

        for i in range(len(BBs)):
            bb=BBs[i]
            id=bb[9]
            ang = (np.pi - bb[6]) + np.pi / 2
            ang = int(ang / (2 * np.pi) * 360)

            size=bb[0:3]

            if id in self.car_dict.keys():
                previous_ori=self.car_dict[id].GetOrientation()[2]
                self.car_dict[id].pos(0,0,0)
                self.car_dict[id].rotateZ(ang-previous_ori)
                self.car_dict[id].pos(bb[3], bb[4], bb[5] + bb[0] / 2)
                self.tracks_dict[id].append(bb)
                if clear:
                    self.VTK_actor_clear.append(self.car_dict[id])
                else:
                    self.VTK_actor.append(self.car_dict[id])
            else:
                new_car=load('Car.obj')
                new_car.scale(size)
                new_car.rotateZ(ang)
                new_car.pos(bb[3], bb[4], bb[5] + bb[0] / 2)

                if type(color) is list:
                    this_c=color[i]
                else:
                    this_c=color

                new_car.c(this_c)
                new_car.alpha(alpha)
                self.car_dict[id]=new_car
                self.tracks_dict[id]=[bb]
                self.tracks_colors[id]=this_c
                if clear:
                    self.VTK_actor_clear.append(self.car_dict[id])
                else:
                    self.VTK_actor.append(self.car_dict[id])

    def add_ego_tube(self,ego_pose,radius=0.35,color='blue',clear=False):
        ego_pose=np.array(ego_pose)
        ego_pose[:, 5]+=1.5

        tube = Tube(ego_pose[:,3:6], c=color, r=radius)

        sp = Spheres(ego_pose[:,3:6], c=color, r=0.4)

        ego_box=ego_pose[-1]
        ego_box[5]-= 1.
        self.add_boxes([ego_box], color=color, alpha=0,l_alpha=0,add_dict=False,clear=True,arraw_scale=3)
        if clear:
            self.VTK_actor_clear.append(tube)
            self.VTK_actor_clear.append(sp)
        else:
            self.VTK_actor.append(tube)
            self.VTK_actor.append(sp)

    def add_text(self,strs,pos=(0,0,0),color=(255,0,0),s=1,clear=False,rot=0):
        text = Text(strs, c=color, s=s)
        ang = (np.pi - rot)
        text.rotateZ(int(ang / (2 * np.pi) * 360))
        text.pos(pos)
        if clear:
            self.VTK_actor_clear+=[text]
        else:
            self.VTK_actor += [text]


    def add_tracklets(self,BBs,
                  color="red",
                  alpha=0.1,
                  if_corner_points=True,
                  corner_color="red",
                  corner_radius=5,
                  corner_alpha=0.4,
                  if_arraws=True,
                  if_tail=True,
                  tail_radius=0.2,
                  if_tail_boxes=True):

        BBs = np.array(BBs)
        if BBs.shape[0] == 0:
            return
        if self.model=="OpenPCDet":
            new_BBs=np.zeros(shape=BBs.shape)
            new_BBs[:,3:6]=BBs[:,0:3]
            new_BBs[:,0]=BBs[:,4]
            new_BBs[:, 1] = BBs[:, 5]
            new_BBs[:, 2] = BBs[:, 3]
            new_BBs[:, 6:] = BBs[:, 6:]
            BBs=new_BBs

        if BBs.shape[1]>7:
            frames=(BBs.shape[1]-7)//4+1

            for i in range(frames):
                if i==0:
                    self.VTK_actor += self._get_vtk_boxes(BBs, color=color, alpha=alpha, model=self.model)
                    cornerPoints, arraws,lines = self._get_corner_arraw_points(BBs,
                                                                         color=corner_color,
                                                                         radius=corner_radius,
                                                                         alpha=corner_alpha,
                                                                         model=self.model)

                    if if_corner_points:
                        self.VTK_actor += [cornerPoints]
                    if if_arraws:
                        self.VTK_actor += [arraws]
                    if if_tail:
                        self.VTK_actor += [Spheres(BBs[:,3:6],r=tail_radius)]
                else:
                    begin_id=3+i*4
                    end_id=7+i*4
                    if if_tail:
                        self.VTK_actor += [Spheres(BBs[:,begin_id:end_id-1],r=tail_radius)]

                    new_BBs=np.zeros(shape=BBs.shape)
                    new_BBs[:,0:7]=BBs[:,0:7]
                    new_BBs[:,3:6]=BBs[:,begin_id:end_id-1]
                    new_BBs[:,6]=BBs[:,end_id-1]
                    if if_tail_boxes:
                        self.VTK_actor += self._get_vtk_boxes(new_BBs, color=color, alpha=alpha, model=self.model)
                        cornerPoints, arraws,lines = self._get_corner_arraw_points(new_BBs,
                                                                             color=corner_color,
                                                                             radius=corner_radius,
                                                                             alpha=corner_alpha,
                                                                             model=self.model)


        else:
            self.VTK_actor += self._get_vtk_boxes(BBs, color=color, alpha=alpha, model=self.model)
            cornerPoints,arraws,lines=self._get_corner_arraw_points(BBs,
                                                              color=corner_color,
                                                              radius=corner_radius,
                                                              alpha=corner_alpha,
                                                              model=self.model)

            if if_corner_points:
                self.VTK_actor+=[cornerPoints]
            if if_arraws:
                self.VTK_actor+=[arraws]
    def set_ego_car(self,box):
        if self.ego_car is None:
            return
        bb = box
        ang = (np.pi - bb[6]) + np.pi / 2
        ang = int(ang / (2 * np.pi) * 360)

        previous_ori = self.ego_car.GetOrientation()[2]
        self.ego_car.pos(0, 0, 0)
        self.ego_car.rotateZ(ang - previous_ori)
        self.ego_car.pos(bb[3], bb[4], bb[5] + bb[0] / 2)

    def add_tracks_as_tube(self,radius=0.4,alpha=0.7,color=None):
        for k in self.tracks_dict.keys():
            tracks=self.tracks_dict[k]
            tracks=np.array(tracks)
            centers = np.zeros(shape=(tracks.shape[0], 3))
            centers[:, :] = tracks[:, 3:6]
            centers[:, 2] = centers[:, 2] + tracks[:, 0] / 2
            if color is not None:
                tube = Tube(centers, c=color, r=radius, alpha=alpha)
            else:
                tube=Tube(centers,c=self.tracks_colors[k],r=radius,alpha=alpha)
            self.VTK_actor.append(tube)

    def show(self,bg=(250,250,250)):


        self.vi.show(self.VTK_actor+self.VTK_actor_clear,bg=bg,resetcam=False)
        interactive()
        self.VTK_actor_clear.clear()

def get_one_box(points, label,pose=None,model="kitti"):
    PI=np.pi
    import math
    point=np.zeros(shape=points.shape)
    point[:]=points[:]
    if model!="waymo":
        point[5]+=point[0]/2


    point_num=200
    i=1
    z_vector = np.arange(- point[0] / 2, point[0] / 2, point[0] / point_num)[0:point_num]
    w_vector = np.arange(- point[1] / 2, point[1] / 2, point[1] / point_num)[0:point_num]
    l_vector = np.arange(- point[2] / 2, point[2] / 2, point[2] / point_num)[0:point_num]

    d_z_p = -np.sort(-np.arange(0, point[0] / 2, point[0] / (point_num*2))[0:point_num])
    d_z_n = np.arange( -point[0] / 2,0, point[0] / (point_num*2))[0:point_num]


    d_w_p = -np.sort(-np.arange(0, point[1] / 2, point[1] / (point_num*2))[0:point_num])
    d_w_n = np.arange(-point[1] / 2,0,  point[1] / (point_num*2))[0:point_num]

    d_l_p = np.arange(point[2] / 2, point[2]*(3/3) , (point[2]*(3/3)-point[2] / 2) / (point_num*2))[0:point_num]


    d1 = np.zeros(shape=(point_num, 4))
    d1[:, 0] = d_w_p
    d1[:, 1] = d_l_p
    d1[:, 2] = d_z_p
    d1[:, 3] = i

    d2 = np.zeros(shape=(point_num, 4))
    d2[:, 0] = d_w_n
    d2[:, 1] = d_l_p
    d2[:, 2] = d_z_p
    d2[:, 3] = i

    d3 = np.zeros(shape=(point_num, 4))
    d3[:, 0] = d_w_p
    d3[:, 1] = d_l_p
    d3[:, 2] = d_z_n
    d3[:, 3] = i

    d4 = np.zeros(shape=(point_num, 4))
    d4[:, 0] = d_w_n
    d4[:, 1] = d_l_p
    d4[:, 2] = d_z_n
    d4[:, 3] = i

    z1 = np.zeros(shape=(point_num, 4))
    z1[:, 0] = -point[1] / 2
    z1[:, 1] = -point[2] / 2
    z1[:, 2] = z_vector
    z1[:, 3] = i
    z2 = np.zeros(shape=(point_num, 4))
    z2[:, 0] = -point[1] / 2
    z2[:, 1] = point[2] / 2
    z2[:, 2] = z_vector
    z2[:, 3] = i
    z3 = np.zeros(shape=(point_num, 4))
    z3[:, 0] = point[1] / 2
    z3[:, 1] = -point[2] / 2
    z3[:, 2] = z_vector
    z3[:, 3] = i
    z4 = np.zeros(shape=(point_num, 4))
    z4[:, 0] = point[1] / 2
    z4[:, 1] = point[2] / 2
    z4[:, 2] = z_vector
    z4[:, 3] = i
    w1 = np.zeros(shape=(point_num, 4))
    w1[:, 0]=w_vector
    w1[:, 1]=-point[2] / 2
    w1[:, 2]=-point[0] / 2
    w1[:, 3] = i
    w2 = np.zeros(shape=(point_num, 4))
    w2[:, 0] = w_vector
    w2[:, 1] = -point[2] / 2
    w2[:, 2] = point[0] / 2
    w2[:, 3] = i
    w3 = np.zeros(shape=(point_num, 4))
    w3[:, 0] = w_vector
    w3[:, 1] = point[2] / 2
    w3[:, 2] = -point[0] / 2
    w3[:, 3] = i
    w4 = np.zeros(shape=(point_num, 4))
    w4[:, 0] = w_vector
    w4[:, 1] = point[2] / 2
    w4[:, 2] = point[0] / 2
    w4[:, 3] = i
    l1 = np.zeros(shape=(point_num, 4))
    l1[:, 0] = -point[1] / 2
    l1[:, 1] = l_vector
    l1[:, 2] = -point[0] / 2
    l1[:, 3] = i
    l2 = np.zeros(shape=(point_num, 4))
    l2[:, 0] = -point[1] / 2
    l2[:, 1] = l_vector
    l2[:, 2] = point[0] / 2
    l2[:, 3] = i
    l3 = np.zeros(shape=(point_num, 4))
    l3[:, 0] = point[1] / 2
    l3[:, 1] = l_vector
    l3[:, 2] = -point[0] / 2
    l3[:, 3] = i
    l4 = np.zeros(shape=(point_num, 4))
    l4[:, 0] = point[1] / 2
    l4[:, 1] = l_vector
    l4[:, 2] = point[0] / 2
    l4[:, 3] = i

    point_mat=np.mat(np.concatenate((z1,z2,z3,z4,w1,w2,w3,w4,l1,l2,l3,l4,d1,d2,d3,d4)))#

    if model=="waymo":
        angle=point[6]-PI/2
    else:
        angle = PI - point[6]

    if pose is None:
        convert_mat = np.mat([[math.cos(angle), -math.sin(angle), 0, point[3]],
                              [math.sin(angle), math.cos(angle), 0, point[4]],
                              [0, 0, 1, point[5]],
                              [0, 0, 0, label]])

        transformed_mat = convert_mat * point_mat.T
    else:

        convert_mat = np.mat([[math.cos(angle), -math.sin(angle), 0, 0],
                              [math.sin(angle), math.cos(angle), 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        transformed_mat = convert_mat * point_mat.T
        pose_mat = np.mat([[pose[0, 0], pose[0, 1], pose[0, 2], point[3]],
                           [pose[1, 0], pose[1, 1], pose[1, 2], point[4]],
                           [pose[2, 0], pose[2, 1], pose[2, 2], point[5]],
                           [0, 0, 0, label]])
        transformed_mat = pose_mat * transformed_mat


    transformed_mat = np.array(transformed_mat.T,dtype=np.float32)

    return transformed_mat



def BBs3D_im_viewer(bbs=[],P2=None,im=None,colors=None):

    bbs=np.array(bbs)
    for i,bb in enumerate(bbs):
        pts_3d = get_one_box(bb,1)

        pts_3d_cam=np.zeros(pts_3d.shape)
        pts_3d_cam[:,:]=pts_3d

        pts_3d_cam[:, 0] = - pts_3d[:, 1]
        pts_3d_cam[:, 1] = -pts_3d[:, 2]
        pts_3d_cam[:, 2] = pts_3d[:, 0]

        img_pts = np.matmul(pts_3d_cam, P2.T)  # (N, 3)
        x, y = img_pts[:, 0] / img_pts[:, 2], img_pts[:,1] / img_pts[:, 2]

        x = np.clip(x, 2, 1239)
        y = np.clip(y, 2, 372)

        x=x.astype(np.int)
        y=y.astype(np.int)

        if type(colors) is list:

            color=[colors[i][2],colors[i][1],colors[i][0]]
        else:
            color=colors

        im[y,x]=color

        x2=x+1
        im[y, x2] = color
        y2 = y + 1
        im[y2, x] = color
        im[y2, x2] = color


        text = str(int(bb[9]))
        org = ((max(x)-min(x))//2+min(x), min(y)-5)
        fontFace = cv2.FONT_HERSHEY_DUPLEX
        fontScale = 0.7
        fontcolor = color  # BGR
        thickness = 1
        lineType = 4
        cv2.putText(im, text, org, fontFace, fontScale, fontcolor, thickness, lineType)

    cv2.imshow('im',im)
    cv2.waitKey(10)



if __name__ == '__main__':
    box=[[2.6,3,8,0,0,0,0,0,0,0]]
    box2 = [[2.6, 3, 8, 0, 0, 0, 0, 0,0,0]] #x,y,z,
    vi=VTKViewer(box_type="Kitti")

    vi.add_boxes(box,color='red',if_corner_points=True,if_arraws=True)

    vi.add_cars(box2, color='gray',alpha=0.1)

    #vi.add_points()#shape=(n,3), color=

    vi.add_spheres([[0,0,1.3]], color='red',radius=0.3,res=10)
    ego_box = [1, 1, 1, 0, 0, -1.6, 0]
    vi.set_ego_car(ego_box)
    vi.show((255,255,255))
