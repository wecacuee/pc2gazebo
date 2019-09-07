#!/usr/bin/env python3
from functools import wraps
from inspect import getargspec
from fcntl import lockf, LOCK_UN, LOCK_EX
from contextlib import contextmanager
import os.path as osp
import subprocess
from collections import namedtuple
from jinja2 import Template


import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2
from ros_numpy.point_cloud2 import pointcloud2_to_array

from votenet_catkin.srv import Votenet


def pointcloud2_to_array_xyz(pc):
    points = pointcloud2_to_array(pc)
    xyzs = np.vstack((points['x'], points['y'], points['z'])).T
    return xyzs


def relpath(fname,
            reldir=osp.dirname(__file__) or "."):
    return osp.join(reldir, fname)


@contextmanager
def lockfile(fobj):
    lockf(fobj, LOCK_EX)
    try:
        yield fobj
    finally:
        lockf(fobj, LOCK_UN)


def rospy_get_param(name, val, basic_types=(int, float, str, list, dict)):
    return (rospy.get_param("~" + name, val)
            if isinstance(val, basic_types)
            else val)


def store_args(func, get_param=rospy_get_param):
    @wraps(func)
    def wrapper(self, *arg, **kw):
        aspec = getargspec(func)
        assert aspec.varargs is None

        for aname, default in zip(aspec.args[1:], aspec.defaults):
            setattr(self, aname, get_param(aname, default))

        for aname, val in zip(aspec.args[1:], arg):
            setattr(self, aname, val)

        for k, v in kw.items():
            setattr(self, k, val)
        func(self, *arg, **kw)
    return wrapper


def write_point_cloud(points_np, dest_file):
    from plyfile import PlyData, PlyElement
    vertex = points_np.astype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face = np.array([],
                    dtype=[('vertex_indices', 'i4', (3,))])

    pld = PlyData(
        [
            PlyElement.describe(
                vertex, 'vertex',
                comments=['tetrahedron vertices']
            ),
            PlyElement.describe(face, 'face')
        ],
        text=True,
        comments=['single tetrahedron with colored faces']
    )
    with lockfile(open(dest_file, "wb")) as df:
        pld.write(df)


def run_votenet(
        input_ply,
        script_path="../ext/votenet/demo.py",
        output_ply="{input_ply_prefix}_results/000000_pred_confident_bbox.ply"):
    subprocess.call(["python", relpath(script_path), "--pc_path", input_ply])
    return output_ply.format(input_ply_prefix=osp.splitext(input_ply)[0])


def query_votenet(msg):
    votenetsrv = rospy.ServiceProxy("/Votenet", Votenet)
    response = votenetsrv(msg)
    return response.pc

def import_and_query_demo_py(points):
    from demo import point_cloud_to_detections
    return point_cloud_to_detections(points, "/tmp/pc.ply")


class VotenetWrap:
    @store_args
    def __init__(self,
                 topic_name = "velodyne_points",
                 msg_type = PointCloud2,
                 dest_file = "/dev/shm/pc2gazebo.ply"):
        self._sub = rospy.Subscriber(self.topic_name, self.msg_type, self)


    def __call__(self, msg):
        assert isinstance(msg, self.msg_type)
        #write_point_cloud(pointcloud2_to_array_xyz(msg), self.dest_file)
        #run_votenet(self.dest_file)
        #print(query_votenet(msg))
        out = import_and_query_demo_py(pointcloud2_to_array_xyz(msg))
        if out is not None:
            pointcloud, pred_map_cls = out
            create_gazebo_file(pred_map_cls[0])

        #self.detect_shapes(pointcloud2_to_array)


def rosnode_main(node_name="votenet",
         votenet_gen=VotenetWrap):
    rospy.init_node(node_name)
    votenet_gen()
    rospy.spin()


def class_name_from_idx(
        idx):
    # self.type2class={'bed':0, 'table':1, 'sofa':2, 'chair':3, 'toilet':4, 'desk':5, 'dresser':6, 'night_stand':7, 'bookshelf':8, 'bathtub':9}
    from sunrgbd.model_util_sunrgbd import SunrgbdDatasetConfig
    mapping = SunrgbdDatasetConfig().class2type
    return mapping[idx]


def available_models_by_class(
        cls,
        mapping=dict(
            bed="""
            IKEA_bed_BEDDINGE
            IKEA_bed_BRIMNES
            IKEA_bed_FJELLSE
            IKEA_bed_HEMNES
            IKEA_bed_LEIRVIK
            IKEA_bed_LILLESAND
            IKEA_bed_MALM
            IKEA_bed_MANDAL
            IKEA_bed_NORDLI
            IKEA_bed_RYKENE
            IKEA_bed_TROMSO
            IKEA_bed_VANVIK""".split(),
            bookcase="""
            IKEA_bookcase_BESTA
            IKEA_bookcase_BILLY
            IKEA_bookcase_EXPEDIT
            IKEA_bookcase_HEMNES
            IKEA_bookcase_KILBY
            IKEA_bookcase_LACK
            IKEA_bookcase_LAIVA""".split(),
            chair="""
            IKEA_chair_BERNHARD
            IKEA_chair_BORJE
            IKEA_chair_EKENAS
            IKEA_chair_EKTORP
            IKEA_chair_FUSION
            IKEA_chair_HENRIKSDAL
            IKEA_chair_HERMAN
            IKEA_chair_INGOLF
            IKEA_chair_IVAR
            IKEA_chair_JOKKMOKK
            IKEA_chair_JULES
            IKEA_chair_KAUSTBY
            IKEA_chair_KLAPPSTA
            IKEA_chair_MARIUS
            IKEA_chair_MARKUS
            IKEA_chair_NILS
            IKEA_chair_PATRIK
            IKEA_chair_POANG
            IKEA_chair_PREBEN
            IKEA_chair_REIDAR
            IKEA_chair_SIGURD
            IKEA_chair_SKRUVSTA
            IKEA_chair_SNILLE
            IKEA_chair_SOLSTA_OLARP
            IKEA_chair_STEFAN
            IKEA_chair_TOBIAS
            IKEA_chair_URBAN
            IKEA_chair_VILMAR
            IKEA_chair_VRETA""".split(),
            desk="""
            IKEA_desk_BESTA
            IKEA_desk_EXPEDIT
            IKEA_desk_FREDRIK
            IKEA_desk_GALANT
            IKEA_desk_HEMNES
            IKEA_desk_LAIVA
            IKEA_desk_LEKSVIK
            IKEA_desk_LIATORP
            IKEA_desk_MALM
            IKEA_desk_MICKE
            IKEA_desk_VALLVIK
            IKEA_desk_VIKA
            IKEA_desk_VITTSJO""".split(),
            sofa="""
            IKEA_sofa_EKTORP
            IKEA_sofa_KARLSTAD
            IKEA_sofa_KIVIK
            IKEA_sofa_KLAPPSTA
            IKEA_sofa_KLIPPAN
            IKEA_sofa_LYCKSELE
            IKEA_sofa_MANSTAD
            IKEA_sofa_SATER
            IKEA_sofa_SKOGABY
            IKEA_sofa_SOLSTA
            IKEA_sofa_TIDAFORS
            IKEA_sofa_VRETA""".split(),
            table="""
            IKEA_table_BIRKELAND
            IKEA_table_BJORKUDDEN
            IKEA_table_BJURSTA
            IKEA_table_BOKSEL
            IKEA_table_DOCKSTA
            IKEA_table_FUSION
            IKEA_table_GRANAS
            IKEA_table_HEMNES
            IKEA_table_INGATORP
            IKEA_table_INGO
            IKEA_table_ISALA
            IKEA_table_JOKKMOKK
            IKEA_table_KLINGSBO
            IKEA_table_KLUBBO
            IKEA_table_LACK
            IKEA_table_LIATORP
            IKEA_table_LINDVED
            IKEA_table_MALM
            IKEA_table_MUDDUS
            IKEA_table_NESNA
            IKEA_table_NILS
            IKEA_table_NORBO
            IKEA_table_NORDEN
            IKEA_table_NORDLI
            IKEA_table_NYVOLL
            IKEA_table_ODDA
            IKEA_table_RAST
            IKEA_table_SALMI
            IKEA_table_TOFTERYD
            IKEA_table_TORSBY
            IKEA_table_UTBY
            IKEA_table_VEJMON
            IKEA_table_VITTSJO""".split(),
            wardrobe="""
            IKEA_wardrobe_ANEBODA
            IKEA_wardrobe_DOMBAS
            IKEA_wardrobe_HEMNES
            IKEA_wardrobe_ODDA
            IKEA_wardrobe_PAX""".split()),
        aliases=dict(
            cabinet="wardrobe",
            bookshelf="bookcase",
        )):
    return mapping.get(cls, mapping.get(aliases.get(cls)))


available_locations = [
      [-5.06466,-3.30098,1.42,0,-0,0],
      [01.07000,-4.51000,0,0,-0,0],
      [-8.21,6.35,0,0,-0,0],
      [-1.37800,7.51000,0,0,-0,0],
      [-0.907958,-5.34608,0,0,-0,0],
      [-1.33100,-4.556118,0,0,-0,0],
      [3.359000,-3.69000,0,0,-0,-1.544],
      [-9.07891,-2.6341,0,0,-0,0],
      [7.23227,3.2000,0,0,-0,-1.54],
      [7.62048,0.974333,0,0,-0,-1.54],
      [-5.59884,-7.73979,0,0,-0,0],
      [-5.42816,-8.94109,0,0,-0,0],
      [9.94622,-1.61582,0,0,-0,0],
      [-4.234,6.950770,0,0,-0,0]]


def render_jinja_template(jinja_file, dst_file, var_dict):
    template = Template(open(jinja_file).read())
    template.stream(**var_dict).dump(dst_file)


ModelJinja = namedtuple("ModelJinja", ["name", "pose"])


def create_gazebo_file(pred_map_cls, template="../sdf/template.sdf.jinja",
                       R=[[1, 0, 0],
                          [0, 0,-1],
                          [0, 1, 0]]):
    models = []
    poses = available_locations.copy()
    for i, (sun_rgbd_idx, box_points, conf) in enumerate(pred_map_cls):
        cls = class_name_from_idx(sun_rgbd_idx)
        available_models = available_models_by_class(cls)
        np.random.shuffle(poses)
        if available_models is not None:
            model_name = available_models[np.random.randint(len(available_models))]
            box_mean = np.mean(box_points, axis=0)
            model = ModelJinja(name=model_name,
                               pose=tuple(poses[i]))
                               #pose=tuple(np.array(R).dot(box_mean).tolist() + [0., 0., 0.]))
            models.append(model)

    render_jinja_template(relpath(template), "/tmp/world.sdf", dict(models=models))
    return "/tmp/world.sdf"

def test_votenet_from_file_pc(pc="../data/pc.msg"):
    pcmsg = PointCloud2().deserialize(open(relpath(pc), "rb").read())
    out = import_and_query_demo_py(pointcloud2_to_array_xyz(pcmsg))
    if out is not None:
        point_cloud_out, pred_map_cls = out
        create_gazebo_file(pred_map_cls[0])

if __name__ == '__main__':
    rosnode_main()
    #test_votenet_from_file_pc()
