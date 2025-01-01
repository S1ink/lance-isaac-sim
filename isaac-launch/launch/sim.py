# RTX Lidar Anotators: https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar/annotator_descriptions.html
# Extension API docs: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/index.html
# Replicator (annotator, writer): https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html
# ROS: https://docs.omniverse.nvidia.com/isaacsim/latest/ros_ros2_tutorials.html
# Fix Ubuntu 22 nvidia drivers screwed and laggy: https://www.reddit.com/r/Ubuntu/comments/ub1zun/comment/itbrp2m/
# Python API docs: https://docs.omniverse.nvidia.com/kit/docs/

import argparse
# import asyncio
# import sys
import numpy as np

parser = argparse.ArgumentParser()
# parser.add_argument("-c", "--config", type=str, default="Example_Rotary", help="Name of lidar config.")
parser.add_argument("-g", "--gui", type=str, default="true", help="Enable Isaac Sim Gui")
parser.add_argument("-a", "--assets", type=str, default="./assets/", help="Assets path")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# Example for creating a RTX lidar sensor and publishing PCL data
headless = args.gui != "true"
simulation_app = SimulationApp({"headless": headless})

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
# from omni.isaac.sensor.ogn.python.nodes import OgnIsaacPrintRTXSensorInfo
from pxr import Gf

import rclpy
# from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from builtin_interfaces.msg._time import Time


# enable ROS bridge extension
enable_extension("omni.isaac.debug_draw")
enable_extension("omni.isaac.conveyor")
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

PHYSICS_RATE = 60 # fps
RENDER_RATE = 60
carb_settings = carb.settings.get_settings()
carb_settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
carb_settings.set_int("/app/runLoops/main/rateLimitFrequency", int(PHYSICS_RATE))
carb_settings.set_int("/persistent/simulation/minFrameRate", int(PHYSICS_RATE))
carb_settings.set("/persistent/app/omniverse/gamepadCameraControl", False)


asset_path = args.assets
try:
    stage.add_reference_to_stage(
        asset_path + "artemis_arena.usd", "/arena"
    )
    stage.add_reference_to_stage(
        asset_path + "lance.usd", "/lance"
    )
except Exception as e:
    print("Failed to load USD assets:\n\t" + e)
    exit(0)

set_prim_property(
    prim_path = "/arena/artemis_arena",
    property_name = "xformOp:translate",
    property_value = (0, 0, 0)
)
set_prim_property(
    prim_path = "/lance/lance",
    property_name = "xformOp:translate",
    property_value = (1, 1, 0.5) )
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

# Create the debug draw pipeline in the post process graph
if (not headless) :
    debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
    debug_writer.initialize(color = (0.7, 0.1, 1, 0.7))
    debug_writer.attach([hydra_texture])

lidar_annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
lidar_annotator.initialize(
    keepOnlyPositiveDistance = False,
    outputAzimuth = False,
    outputBeamId = False,
    outputDistance = False,
    outputElevation = False,
    outputEmitterId = False,
    outputIntensity = False,
    outputMaterialId = True,
    outputNormal = False,
    outputObjectId = False,
    outputTimestamp = False,
    outputVelocity = False,
    transformPoints = False
)
lidar_annotator.attach([hydra_texture])

simulation_app.update()

rclpy.init()

try:
    og.Controller.edit(
        {"graph_path": "/graphs/ROSGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime",     "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishClock",    "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("ReadIMU",         "omni.isaac.sensor.IsaacReadIMU"),
                ("PublishIMU",      "omni.isaac.ros2_bridge.ROS2PublishImu"),
                ("ReadRTF",         "omni.isaac.core_nodes.IsaacRealTimeFactor"),
                ("PublishRTF",      "omni.isaac.ros2_bridge.ROS2Publisher"),
                ("DumpJointPub",    "omni.isaac.ros2_bridge.ROS2PublishTransformTree")
            ],
            og.Controller.Keys.CONNECT: [
                # Execution connections
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut",     "PublishIMU.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishRTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DumpJointPub.inputs:execIn"),
                # Simulation time
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "DumpJointPub.inputs:timeStamp"),
                # IMU data
                ("ReadIMU.outputs:angVel",      "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc",      "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                # # Connecting the ROS2 Context to the clock publisher node so it will run under the specified ROS2 Domain ID
                # ("Context.outputs:context", "PublishClock.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Assigning topic name to clock publisher
                ("PublishClock.inputs:topicName", "/clock"),
                # # Assigning a Domain ID of 1 to Context node
                # ("Context.inputs:domain_id", 1),
                ("ReadSimTime.inputs:resetOnStop", True),

                ("ReadIMU.inputs:imuPrim",      "/lance/lance/lidar_link/imu_sensor"),
                ("PublishIMU.inputs:frameId",   "lidar_link"),
                ("PublishIMU.inputs:queueSize", 1),
                ("PublishIMU.inputs:topicName", "/lance/imu"),

                ("PublishRTF.inputs:messageName",       "Float32"),
                ("PublishRTF.inputs:messagePackage",    "std_msgs"),
                ("PublishRTF.inputs:messageSubfolder",  "msg"),
                ("PublishRTF.inputs:topicName",         "/isaac/rtf"),

                ("DumpJointPub.inputs:parentPrim",  "/lance/lance/frame_link"),
                ("DumpJointPub.inputs:targetPrims", "/lance/lance/collection_link"),
            ],
        },
    )
    og.Controller.connect(
        og.Controller.attribute("/graphs/ROSGraph/ReadRTF.outputs:rtf"),
        og.Controller.attribute("/graphs/ROSGraph/PublishRTF.inputs:data")
    )
except Exception as e:
    print(e)

simulation_app.update()

try:
    og.Controller.edit(
        {
            "graph_path" : "/graphs/ControlGraph",
            "evaluator_name" : "execution",
            "pipeline_stage" : og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PhysicsStep",     "omni.isaac.core_nodes.OnPhysicsStep"),
                ("TwistSub",        "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("GetXVel",         "omni.graph.nodes.BreakVector3"),
                ("GetRotVel",       "omni.graph.nodes.BreakVector3"),
                ("DiffController",  "omni.isaac.wheeled_robots.DifferentialController"),
                ("GetLeftVel",      "omni.graph.nodes.ArrayIndex"),
                ("GetRightVel",     "omni.graph.nodes.ArrayIndex"),
                ("LeftController",  "omni.isaac.conveyor.IsaacConveyor"),
                ("RightController", "omni.isaac.conveyor.IsaacConveyor"),
                ("DumpSub",         "omni.isaac.ros2_bridge.ROS2Subscriber"),
                ("JointToken",      "omni.graph.nodes.ConstantToken"),
                ("MakeTokenArray",  "omni.graph.nodes.ConstructArray"),
                ("ToDouble",        "omni.graph.nodes.ToDouble"),
                ("MakeVelArray",    "omni.graph.nodes.ConstructArray"),
                ("DumpController",  "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("PhysicsStep.outputs:step",    "TwistSub.inputs:execIn"),
                ("PhysicsStep.outputs:step",    "LeftController.inputs:onStep"),
                ("PhysicsStep.outputs:step",    "RightController.inputs:onStep"),
                ("PhysicsStep.outputs:step",    "DumpSub.inputs:execIn"),
                ("TwistSub.outputs:execOut",    "DiffController.inputs:execIn"),
                ("DumpSub.outputs:execOut",     "DumpController.inputs:execIn"),

                ("TwistSub.outputs:angularVelocity",        "GetRotVel.inputs:tuple"),
                ("TwistSub.outputs:linearVelocity",         "GetXVel.inputs:tuple"),
                ("GetRotVel.outputs:z",                     "DiffController.inputs:angularVelocity"),
                ("GetXVel.outputs:x",                       "DiffController.inputs:linearVelocity"),
                ("PhysicsStep.outputs:deltaSimulationTime", "DiffController.inputs:dt"),
                ("DiffController.outputs:velocityCommand",  "GetLeftVel.inputs:array"),
                ("DiffController.outputs:velocityCommand",  "GetRightVel.inputs:array"),
                ("GetLeftVel.outputs:value",                "LeftController.inputs:velocity"),
                ("GetRightVel.outputs:value",               "RightController.inputs:velocity"),
                ("PhysicsStep.outputs:deltaSimulationTime", "LeftController.inputs:delta"),
                ("PhysicsStep.outputs:deltaSimulationTime", "RightController.inputs:delta"),

                ("JointToken.inputs:value",        "MakeTokenArray.inputs:input0"),
                ("ToDouble.outputs:converted",      "MakeVelArray.inputs:input0"),
                ("MakeTokenArray.outputs:array",    "DumpController.inputs:jointNames"),
                ("MakeVelArray.outputs:array",      "DumpController.inputs:velocityCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("TwistSub.inputs:topicName", "/robot_cmd_vel"),

                ("DiffController.inputs:maxAcceleration",        10.),
                ("DiffController.inputs:maxAngularAcceleration", 20.),
                ("DiffController.inputs:maxAngularSpeed",        5.),
                ("DiffController.inputs:maxDeceleration",        10.),
                ("DiffController.inputs:maxLinearSpeed",         3.),
                ("DiffController.inputs:maxWheelSpeed",          2.5),
                ("DiffController.inputs:wheelDistance",          0.579),
                ("DiffController.inputs:wheelRadius",            1.),

                ("GetLeftVel.inputs:index",     1),
                ("GetRightVel.inputs:index",    0),

                ("LeftController.inputs:conveyorPrim",  "/lance/lance/left_track_link"),
                ("RightController.inputs:conveyorPrim", "/lance/lance/right_track_link"),

                ("DumpSub.inputs:messageName",      "Float64"),
                ("DumpSub.inputs:messagePackage",   "std_msgs"),
                ("DumpSub.inputs:messageSubfolder", "msg"),
                ("DumpSub.inputs:topicName",        "/dump_cmd_vel"),

                ("JointToken.inputs:value", "dump_joint"),

                ("DumpController.inputs:targetPrim", "/lance/lance"),
            ]
        }
    )
    og.Controller.connect(
        og.Controller.attribute("/graphs/ControlGraph/DumpSub.outputs:data"),
        og.Controller.attribute("/graphs/ControlGraph/ToDouble.inputs:value")
    )
except Exception as e:
    print(e)

simulation_app.update()

def normalize_retro(x : float):
    return 1. if x == 12. or x == 11. else 0.   # https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_sensor_materials.html

np_normalize_retro = np.frompyfunc(normalize_retro, 1, 1)

simulation_context = SimulationContext(
    physics_dt = (1. / PHYSICS_RATE),
    rendering_dt = (1. / RENDER_RATE),
    stage_units_in_meters=1.
)
simulation_app.update()

node = rclpy.create_node("isaac_sim")
pc_pub = node.create_publisher(sensor_msgs.PointCloud2, '/lance/lidar_scan', 10)

pc_ros_dtype = sensor_msgs.PointField.FLOAT32
pc_dtype = np.float32
pc_itemsize = np.dtype(pc_dtype).itemsize
pc_fields = [
    sensor_msgs.PointField(name = 'x', offset = 0, datatype = pc_ros_dtype, count = 1),
    sensor_msgs.PointField(name = 'y', offset = 4, datatype = pc_ros_dtype, count = 1),
    sensor_msgs.PointField(name = 'z', offset = 8, datatype = pc_ros_dtype, count = 1),
    sensor_msgs.PointField(name = 'reflective', offset = 12, datatype = pc_ros_dtype, count = 1),
]

PC_PUB_RATE = 20
PUB_THRESH_S = 0.002

if (headless) :
    simulation_app.update()
    simulation_context.play()

frame = 0
prev_pub_t = -1
while simulation_app.is_running():

    simulation_app.update()
    # lidar_annotator.get_data()['data']

    if( simulation_context.is_playing() ) :
        t = simulation_context.current_time
        if( (t - prev_pub_t) > (1. / PC_PUB_RATE) - PUB_THRESH_S ) :
            try:
                lidar_data = lidar_annotator.get_data()
                if( len(lidar_data['data']) > 0 ) :
                    x = np.hstack( (
                        lidar_data['data'],
                        np_normalize_retro( lidar_data['materialId'] )
                            .astype(pc_dtype).reshape(-1, 1)
                    ) )
                    # print(x)
                    # print(lidar_data['info'])
                    # print(np.shape(lidar_data['data']))
                    # print(lidar_data['materialId'])
                    # print(flush=True)

                    header = std_msgs.Header(
                        stamp = Time(sec = int(t),
                        nanosec = int(t * 1e9) % int(1e9)),
                        frame_id = "lidar_link"
                    )
                    pc_pub.publish(
                        sensor_msgs.PointCloud2(
                            header = header,
                            height = 1,
                            width = x.shape[0],
                            is_dense = True,
                            is_bigendian = False,
                            fields = pc_fields,
                            point_step = pc_itemsize * x.shape[1],
                            row_step = pc_itemsize * x.size,
                            data = x.astype(pc_dtype).tobytes()
                        ) )
                    prev_pub_t = t

            except Exception as e:
                print(e)

        # print(t, frame)
        frame = frame + 1

    rclpy.spin_once(node, timeout_sec = 0.)


# cleanup and shutdown
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()
