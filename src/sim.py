# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# RTX Lidar Anotators: https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar/annotator_descriptions.html
# Extension API docs: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/index.html
# Replicator (annotator, writer): https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html
# ROS: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_ros2_tutorials.html

# import argparse
import asyncio
import sys
import numpy as np

# parser = argparse.ArgumentParser()
# parser.add_argument("-c", "--config", type=str, default="Example_Rotary", help="Name of lidar config.")
# args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# Example for creating a RTX lidar sensor and publishing PCL data
simulation_app = SimulationApp({"headless": False})
import carb
import omni
import omni.kit.viewport.utility
import omni.graph.core as og
import omni.replicator.core as rep
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage
from omni.isaac.core.utils.prims import is_prim_path_valid, define_prim, set_prim_property, create_prim, get_prim_at_path
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.nucleus import get_assets_root_path
from pxr import Gf

# import rclpy
# from rclpy.node import Node
# import sensor_msgs.msg as sensor_msgs
# import std_msgs.msg as std_msgs
# from builtin_interfaces.msg._time import Time


# class PointCloudPublisher(Node):

#     def __init__(self):
#         super().__init__('isaac_point_cloud_publisher')

#         self.RENDER_PATH = "/Render/PostProcess/SDGPipeline"
#         self.PUB_FREQ = 20.

#         self.points = np.array([[1, 1, 1, 1]])
#         self.time = Time()
#         self.points_pub = self.create_publisher(sensor_msgs.PointCloud2, '/lidar_points', 10)
#         self.timer = self.create_timer(self.PUB_FREQ, self._lidar_callback)

#     def _lidar_callback(self):
#         p = self.RENDER_PATH + "RenderProduct_Isaac_RtxSensorCpuIsaacComputeRTXLidarPointCloud"
#         if is_prim_path_valid(p) :
#             lidar_compute_node_path = p
#         else :
#             for i in range(1, 10) :
#                 p = self.RENDER_PATH + f"Render_Product_Isaac_0{i}_RtxSensorCpuIsaacComputeRTXLidarPointCloud"
#                 if is_prim_path_valid(p) :
#                     lidar_compute_node_path = p
#                     break
        
#         compute_node = og.Controller().node(lidar_compute_node_path)
#         r_arr = compute_node.get_attribute("outputs:range").get()
#         elev_arr = compute_node.get_attribute("outputs:elevation").get()
#         azim_arr = compute_node.get_attribute("outputs:azimuth").get()
#         intensity_array = compute_node.get_attribute("outputs:intensity").get()

#         x = r_arr * np.sin(elev_arr) * np.cos(azim_arr)
#         y = r_arr * np.sin(elev_arr) * np.sin(azim_arr) * -1
#         z = r_arr * np.cos(elev_arr)

#         self.points = np.column_stack((x, y, z, intensity_array))
#         self.time = self.get_clock().now().to_msg()
#         self.point_cloud = self._create_point_cloud(self.points, self.time, 'lidar_link')
#         self.points_pub.publish(self.point_cloud)

#     def _create_point_cloud(self, points, time, parent_frame):
#         ros_dtype = sensor_msgs.PointField.FLOAT32
#         dtype = np.float32
#         itemsize = np.dtype(dtype).itemsize

#         data = points.astype(dtype).tobytes()
#         fields = [
#             sensor_msgs.PointField(name = 'x', offset = 0, datatype = ros_dtype, count = 1),
#             sensor_msgs.PointField(name = 'y', offset = 4, datatype = ros_dtype, count = 1),
#             sensor_msgs.PointField(name = 'z', offset = 8, datatype = ros_dtype, count = 1),
#             sensor_msgs.PointField(name = 'intensity', offset = 12, datatype = ros_dtype, count = 1),
#         ]
#         header = std_msgs.Header(stamp = time, frame_id = parent_frame)

#         return sensor_msgs.PointCloud2(
#             header = header,
#             height = 1,
#             width = points.shape[0],
#             is_dense = True,
#             is_bigendian = False,
#             fields = fields,
#             point_step = (itemsize * 4),
#             row_step = (itemsize * 4 * points.shape[0]),
#             data = data
#         )

# async def lidar_task():
#     publisher = PointCloudPublisher()

#     while rclpy.ok():
#         rclpy.spin_once(publisher)
#         await asyncio.sleep(0.02)

#     publisher.unregister()
#     publisher = None
        

# enable ROS bridge extension
enable_extension("omni.isaac.debug_draw")
enable_extension("omni.isaac.conveyor")
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

simulation_app.update()
# Loading the simple_room environment
# stage.add_reference_to_stage(
#     assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/warehouse"
# )
stage.add_reference_to_stage(
    "./assets/artemis_arena.usd", "/arena"
)
stage.add_reference_to_stage(
    "./assets/lance.usd", "/lance"
)
set_prim_property(
    prim_path = "/lance/lance",
    property_name = "xformOp:translate",
    property_value = (1, 1, 1) )
create_prim(
    prim_path = "/distantlight",
    prim_type = "DistantLight", 
    orientation = (0.98296, 0.12941, 0.12941, 0.01704),
    attributes = { "inputs:intensity": 1000. } )

simulation_app.update()

# lidar_config = args.config

# Create the lidar sensor that generates data into "RtxSensorCpu"
# Possible config options are Example_Rotary and Example_Solid_State
_, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/lance/lance/lidar_link/sensor",
    parent=None,
    config="SICK_multiScan136",
    translation=(0, 0, 0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
)
hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_app.update()

# Create the debug draw pipeline in the post process graph
debug_writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud" + "Buffer")
debug_writer.initialize(color = (1, 0.5, 0, 1))
debug_writer.attach([hydra_texture])
# ros_writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
# ros_writer.initialize(topicName = "/lance/lidar_scan", frameId = "lidar_link", queueSize = 1)
# ros_writer.attach([hydra_texture])

simulation_app.update()

# simulation_context.play()

# rclpy.init()

# frame = 0
while simulation_app.is_running():

    # if frame == 48:
    #     asyncio.ensure_future(lidar_task())

    # frame = frame + 1

    simulation_app.update()

# rclpy.shutdown()

# cleanup and shutdown
simulation_context.stop()
simulation_app.close()
