#########################################################################
# AdaptiveNDE Project                                                   #
# AFRL-ROS Package                                                      # 
#########################################################################
# Copyright 2023-2024 University of Dayton                              #
#                                                                       #
# Redistribution and use in source and binary forms, with or without    # 
# modification, are permitted provided that the following conditions    #
# are met:                                                              #
#                                                                       # 
# 1. Redistributions of source code must retain the above copyright     #
#    notice, this list of conditions and the following disclaimer.      #
# 2. Redistributions in binary form must reproduce the above copyright  #
#    notice, this list of conditions and the following disclaimer in    #
#    the documentation and/or other materials provided with the         #
#    distribution.                                                      #
# 3. Neither the name of the copyright holder nor the names of its      #
#    contributors may be used to endorse or promote products derived    #
#    from this software without specific prior written permission.      #
#                                                                       #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
# “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR #
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,#
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      #
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, #
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY #
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  #
#########################################################################
# This material is based in part upon work supported by the U.S. Air    #
# Force under Contract FA8650-19-D-5230.                                #
#                                                                       #
# DISTRIBUTION A. Approved for public release: distribution unlimited.  #
#########################################################################
# File:  motiontest.dgp                                                     
# Authors: Tyler Lesthaeghe
# Created: 2023-06-22                                                  
# Description:
#   Dataguzzler-Python module for communicating with ROS from Dataguzzler
# Derived From:                                                         
#                                             
#########################################################################
# Revision Log                                                          
# 
# 2023-06-22:  Initial Commit
# 2023-11-07:  Bugfixes for python version and rate limiting
# 
#########################################################################
# Development Notes
#
#
#########################################################################

import sys
import spatialnde2 as snde
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import pint
import copy
import typing
import transforms3d
import math
from threading import Thread
import traceback
from dataguzzler_python.dgpy import Module as dgpy_Module
from dataguzzler_python.dgpy import InitCompatibleThread
from dataguzzler_python.dgpy import threadsafe as dgpy_threadsafe
import rclpy
import rclpy.logging
from rclpy.node import Node
import threading
import itertools
from concurrent.futures import ThreadPoolExecutor
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from dgpython_ros2_actioninterface.action import DgPython
from tari_interfaces.action import RasterScan, JointMove, PipelineMove
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, WrenchStamped
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Quaternion, TransformStamped, Point, PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import Image, CameraInfo, JointState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_msgs.msg import IOStates
from threading import RLock
from typing import Any
from rclpy.task import Future
# from tari_interfaces.action import PoseArrayHandler
# from tari_interfaces.srv import TariNoether
# from industrial_reconstruction_msgs.srv import StartReconstruction, StopReconstruction
# from industrial_reconstruction_msgs.msg import NormalFilterParams, RGBDImageParams, TSDFVolumeParams

ur = pint.get_application_registry()
orient_dtype = [('quat', '<f4', (4,)), ('offset', '<f4', (4,))]


class ParamAttribute(object):
    Name=None
    
    def __init__(self,Name,doc):
        self.Name=Name
        self.__doc__=doc
        pass
    
    def __get__(self,obj,type=None):
        return obj._GetParam(self.Name)

    def __set__(self,obj,value):
        obj._SetParam(self.Name,value)
        pass

    pass

def add_parameters(cls):
    for paramname in cls._paramdict:            
        descr = ParamAttribute(paramname,cls._paramdict[paramname][0])
        setattr(cls,paramname,descr)
        
    return cls


class PatchRclpyIssue1123(ActionClient):

    _lock: RLock = None  # type: ignore

    @property
    def _cpp_client_handle_lock(self) -> RLock:
        if self._lock is None:
            self._lock = RLock()
        return self._lock

    async def execute(self, *args: Any, **kwargs: Any) -> None:
        with self._cpp_client_handle_lock:
            return await super().execute(*args, **kwargs)  # type: ignore

    def send_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._get_result_async(*args, **kwargs)

    def send_goal_bounded(self, goal, goal_timeout=15.0, result_timeout=120.0, attempts=3, **kwargs):
        """send_goal() that gives up instead of blocking forever.

        rclpy's stock send_goal() waits on an event with no timeout. When the
        client loses track of a goal response -- ros2/rclpy#1123, which the
        locking above makes rarer but does not eliminate -- that event is never
        set and the caller is wedged permanently. In a raster scan that means
        the scan simply stops partway with no error, which is worse than
        failing.

        Position goals are idempotent: sending the same target twice moves there
        once. So a dropped response is recoverable by resending.
        """
        last_error = None
        for attempt in range(attempts):
            event = threading.Event()

            send_goal_future = self.send_goal_async(goal, **kwargs)
            send_goal_future.add_done_callback(lambda future: event.set())

            if not event.wait(goal_timeout):
                last_error = "no goal response within %.1f s" % goal_timeout
                self._node.get_logger().warning(
                    "action %s: %s (attempt %d/%d), resending"
                    % (self._action_name, last_error, attempt + 1, attempts))
                continue

            if send_goal_future.exception() is not None:
                raise send_goal_future.exception()

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                raise RuntimeError("action %s rejected the goal" % (self._action_name,))

            result_future = goal_handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda future: result_event.set())

            if not result_event.wait(result_timeout):
                last_error = "no result within %.1f s" % result_timeout
                self._node.get_logger().warning(
                    "action %s: %s (attempt %d/%d), resending"
                    % (self._action_name, last_error, attempt + 1, attempts))
                continue

            if result_future.exception() is not None:
                raise result_future.exception()

            return result_future.result()

        raise TimeoutError("action %s: %s after %d attempts"
                           % (self._action_name, last_error, attempts))

class DgpyMultiThreadedExecutor(MultiThreadedExecutor):
    """MultiThreadedExecutor whose worker threads can touch dgpy module state.

    rclpy hands callbacks to a plain ThreadPoolExecutor. Those workers have no
    dataguzzler-python thread context, so a node callback that touches module
    attributes -- which most of the callbacks here do -- would not behave the
    way it did when everything ran on the single spinner thread. Giving each
    worker its own compatible context restores that.
    """

    def __init__(self, module, num_threads=4):
        super().__init__(num_threads=num_threads)

        counter = itertools.count()

        def init_worker():
            InitCompatibleThread(
                module, "_executor%d" % (next(counter),),
                no_thread_will_ever_wait_for_this_thread_while_holding_module_context=True)

        # Replace the pool rclpy built with an equivalent one that initialises
        # its workers. Done after super().__init__ so nothing has been submitted.
        self._executor = ThreadPoolExecutor(num_threads, initializer=init_worker)


class LivePose(object):
    """The probe's live pose, readable while the motion module is busy moving.

    `ros.t` is a module attribute, so reading it from another thread goes
    through the dgpy module context lock -- and the thread that called
    Move()/CartesianMove() holds that lock for the entire round trip. A
    point-to-point move is ~0.5 s; one raster row is several seconds. Anything
    that has to know where the probe is *while* it moves therefore gets one or
    two readings for the whole sweep, and binds its data to the wrong place.

    The TF thread pushes here instead. This is a plain object registered as
    dgpy.threadsafe, so censoring passes the real object across the context
    boundary rather than an OpaqueWrapper whose methods would take the module
    lock all over again. Fetch it once outside a move:

        pose = ros.live_pose        # cheap, needs the module context briefly
        ...
        pose.xy()                   # during a move: no module context at all

    The TF thread owns update(); everything else only reads.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._xyz = None
        self._quat = None
        self._stamp = None
        self.count = 0

    def update(self, transform_stamped, when):
        tr = transform_stamped.transform.translation
        rot = transform_stamped.transform.rotation
        # Copy the fields out rather than keeping the message: the TF thread
        # reuses it, and a reader would otherwise see it change underneath.
        with self._lock:
            self._xyz = (tr.x, tr.y, tr.z)
            self._quat = (rot.x, rot.y, rot.z, rot.w)
            self._stamp = when
            self.count += 1

    def xyz(self):
        """(x, y, z) in the world frame, or None before the first TF arrives."""
        with self._lock:
            return self._xyz

    def xy(self):
        """(x, y) in the world frame, or None before the first TF arrives."""
        with self._lock:
            if self._xyz is None:
                return None
            return (self._xyz[0], self._xyz[1])

    def snapshot(self):
        """((x,y,z), (qx,qy,qz,qw), monotonic time of the lookup), or None.

        One consistent reading -- use this rather than calling xyz() and then
        quat(), which can straddle an update.
        """
        with self._lock:
            if self._xyz is None:
                return None
            return (self._xyz, self._quat, self._stamp)

    def age(self):
        """Seconds since the last TF update, or None. A large or growing value
        means the TF thread is stuck, and any position read here is stale."""
        with self._lock:
            if self._stamp is None:
                return None
            return time.monotonic() - self._stamp


dgpy_threadsafe.register(LivePose)


class client(object):
    def __init__(self, client, serviceType, serverName):
        self.serviceType = serviceType
        self.serverName = serverName
        self.client = client
        self.interfaces = {}

    def registerInterface(self, interfacename, interface):
        self.interfaces[interfacename] = interface

    async def sendGoal(self, interface, options):
        goal_msg = self.interfaces[interface].Goal()
        for key in options:
            setattr(goal_msg, key, options[key])
            print(f"Setting {key} to {options[key]}")
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        # while not future.done():
        #     time.sleep(0.01)
        # return future.result()
        goal_handle = await future
        return goal_handle

    def sendRequest(self, interface, options=None):
        req = self.interfaces[interface].Request()
        if options:
            for key in options:
                setattr(req, key, options[key])
                print(f"Setting {key} to {options[key]}")
        future = self.client.call_async(req)
        while not future.done():
            time.sleep(0.01)
        return future.result()
    
class Colors:
    RESET = "\033[0m"
    COLOR_26 = "\033[26m"
    COLOR_27 = "\033[27m"
    COLOR_28 = "\033[28m"
    COLOR_29 = "\033[29m"
    COLOR_30 = "\033[30m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"

@add_parameters
class motion(Node, metaclass=dgpy_Module):
    module_name : typing.ClassVar[str] = None
    _thread : typing.ClassVar[Thread] = None
    _quit : typing.ClassVar[bool] = None
    _pause : typing.ClassVar[bool] = None
    _paused : typing.ClassVar[bool] = None
    _params : typing.ClassVar[dict] = None               
                                                         # Help Text, Default Value, Setter, Getter, Getter for Metadata, Metadata Entry
    _paramdict : typing.ClassVar[dict] = {}
    #'SampleRate': ('Sampling Frequency in Hz (per channel)', 1000, lambda val: val.m_as('Hz'), lambda val: val * ur.Hz, lambda val: val, lambda val: snde.metadatum('daq_params-SampleRate', val, 'Hertz')),
    #'AcquisitionType': ('Specifies continuous or finite sampling -- Valid options are "CONTINUOUS" or "FINITE"', const.AcquisitionType.FINITE, lambda val: {"CONTINUOUS": const.AcquisitionType.CONTINUOUS, "FINITE": const.AcquisitionType.FINITE}[val], lambda val: {const.AcquisitionType.CONTINUOUS: 'CONTINUOUS', const.AcquisitionType.FINITE: 'FINITE'}[val], lambda val: {const.AcquisitionType.CONTINUOUS: 'CONTINUOUS', const.AcquisitionType.FINITE: 'FINITE'}[val], lambda val: snde.metadatum('daq_params-AcquisitionType', val)),
    #'SampsPerChan': ('Number of Data Points Per Channel', 100, lambda val: val, lambda val: val, lambda val: val, lambda val: snde.metadatum('daq_params-SampsPerChan', val))}

    def __init__(self, module_name, recdb, from_frame_rel, to_frame_rel, eoat_mesh=None, depth_image_topic='/azure/depth/image_raw', camera_info='/azure/rgb/camera_info', **kwargs):
        super().__init__('dataguzzler_frame_publisher')
        self.get_logger().info("Dataguzzler Frame Publisher Module Starting")
        self.laser_scan_array_pub = self.create_publisher(PoseArray, 'laser_scan_array', 10)
        self.angled_corners_pub = self.create_publisher(PoseArray, '/yolo_poses/angled_corners_array', 10)
        self.coarse_scan_pub = self.create_publisher(PoseArray, 'coarse_scan_array', 10)
        self.object_detection_pub = self.create_publisher(PoseArray, '/yolo_poses/object_detection_array', 10)
        self.dome_pose_pub = self.create_publisher(PoseArray, 'dome_pose_array', 10)
        self.probe_end_pub = self.create_publisher(TransformStamped, '/probe_end_transform', 10)
        self.gpio_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.gpio_callback, 10)
        self.detect_sub = self.create_subscription(DetectionArray, '/yolo/detections', self.detection_callback, 10)
        self.tracking_sub = self.create_subscription(DetectionArray, '/yolo/tracking', self.tracking_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_image_topic, self.depth_image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info, self.camera_info_callback, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/msg_sync/force_msg', self.force_callback, 10)
        self.pos_sub = self.create_subscription(TransformStamped, '/msg_sync/position_msg', self.position_callback, 10)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        self.tag_sub = self.create_subscription(AprilTagDetectionArray, '/rgb/tags', self.tag_callback, 10)
        self.bridge = CvBridge()
        self.gpio_sub
        self.detections = DetectionArray()
        self.tracked_detections = DetectionArray()
        self.camera_info = CameraInfo()
        self.svd_msg = IOStates()
        self.force_msg = WrenchStamped()
        self.click_msg = PointStamped()
        self.click_msg.header.frame_id = to_frame_rel
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = to_frame_rel
        self.coarse_array = PoseArray()
        self.coarse_array.header.frame_id = to_frame_rel
        self.coordinate_array = PoseArray()
        self.coordinate_array.header.frame_id = to_frame_rel
        self.yolo_array = PoseArray()
        self.yolo_array.header.frame_id = to_frame_rel
        self.module_name = module_name
        self._params = {}
        self.from_frame_rel = from_frame_rel
        self.to_frame_rel = to_frame_rel
        self.t = TransformStamped()
        # Same pose as self.t, but readable without the module context -- see
        # LivePose. Created before the TF thread starts so it is never None.
        self.live_pose = LivePose()
        self.desired_t = None
        self.specimen_pose_channel = '/specimen_pose'
        self.specimen_pose_channel_copy = '/specimen_pose_copy'
        self.specimen_model_file = None
        self.specimen_model_tree = '/graphics/specimen/'
        self.specimen_data_channel = '/specimen_data'
        self.specimen_data_chan = None
        self.specimen_pose_chan = None
        self.specimen_model_tree_copy = None
        self.specimen_pose_channel_copy = None
        self.positioner_channel = '/probe_and_specimen'
        self.specimen_position = None
        self.positioner_chan = None
        self.probe_position = None
        self.recdb = recdb

        self.specimen_data_chan = self.recdb.define_channel(self.specimen_data_channel,"main")
        self.specimen_pose_chan = self.recdb.define_channel(self.specimen_pose_channel,"main")
        self.specimen_model_tree_copy = recdb.define_channel('/graphics/specimen_copy/', "main")
        self.specimen_pose_channel_copy = recdb.define_channel('/specimen_pose_copy', "main")
        self.positioner_chan = self.recdb.define_channel(self.positioner_channel,"main")
        
        self.graphman = snde.graphics_storage_manager("/graphics/", self.recdb.lowlevel_alloc, self.recdb.alignment_requirements,self.recdb.lockmgr, 1e-8, 2000000000)

        if eoat_mesh is not None:
            self.eoat_mesh = eoat_mesh
            self.eoat_model_tree = '/graphics/probe/'
            self.eoat_pose_channel = '/probe_pose'
            self.graphicsgroup = self.recdb.define_channel('/graphics/', 'main', False, self.graphman)
            transact = self.recdb.start_transaction()
            self.eoat_recording = snde.x3d_load_geometry(
                transact,
                self.graphman,
                self.eoat_mesh,
                0,
                'main',
                self.eoat_model_tree,
                None,
                ['reindex_vertices', 'reindex_tex_vertices']
                )
            transact.end_transaction()
            self.eoat_pose_chan = self.recdb.define_channel(self.eoat_pose_channel, "main")
            self._NewEOATPositionInTransaction(snde.snde_invalid_orientation3())

            
        else:
            self.eoat_mesh = None


        # PatchRclpyIssue1123, not the stock ActionClient: rclpy's action client
        # has no lock around the C++ client handle, so a goal sent while a
        # previous response is still being dispatched can have its response
        # matched to the wrong goal. It shows up as
        #   "Ignoring unexpected goal response. There may be more than one
        #    action server for the action 'dgpython'"
        # and a send_goal() that never returns, which wedges a scan partway
        # through. It needs repeated goals in quick succession to trigger, so a
        # single move looks fine.
        # A reentrant group, separate from the node default group the
        # subscriptions use. Everything on a node shares one mutually exclusive
        # callback group unless told otherwise, which means a goal response or a
        # result response queues behind the TF, image and detection traffic this
        # node also carries -- and TF alone runs at 200 Hz. That is what made a
        # Move() round trip cost ~0.5 s against ~20 ms of actual motion.
        self._action_callback_group = ReentrantCallbackGroup()

        self._joint_move_client = PatchRclpyIssue1123(self, JointMove, 'execute_joint_move', callback_group=self._action_callback_group)
        self._raster_scan_client = PatchRclpyIssue1123(self, RasterScan, 'execute_raster', callback_group=self._action_callback_group)
        self._pipeline_move_client = PatchRclpyIssue1123(self, PipelineMove, 'execute_pipeline_move', callback_group=self._action_callback_group)

        self._action_client = PatchRclpyIssue1123(self, DgPython, 'dgpython', callback_group=self._action_callback_group)

        # Inverse kinematics, and a way to command a joint configuration
        # directly. Both exist for one reason: a Cartesian path is followed by
        # solving IK at each step SEEDED from the previous one, so which arm
        # configuration the path starts in decides whether it can be followed to
        # the end. Every pose on a raster row can be individually reachable and
        # the row still truncate two thirds of the way along, purely because the
        # approach move left the arm in a configuration that runs out of joint
        # travel. SolveIK() picks the configuration; MoveJoints() gets the arm
        # into it. See ApproachInConfiguration().
        self._compute_ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._action_callback_group)
        # Used only by the fallback in MoveJoints(): when MoveIt cannot plan the
        # joint move for us, this is what stands between an unchecked joint
        # command and the fixturing.
        self._state_validity_client = self.create_client(
            GetStateValidity, '/check_state_validity',
            callback_group=self._action_callback_group)
        # Which trajectory controller is live differs between the simulated cell
        # and the real robot: the UR driver runs scaled_joint_trajectory_controller
        # (it honours the teach pendant speed slider), the Gazebo cell runs plain
        # joint_trajectory_controller. Hardcoding either one silently does
        # nothing on the other, so both are created and MoveJoints() picks
        # whichever is actually being served, preferring the scaled one.
        self._joint_traj_clients = [
            (name, PatchRclpyIssue1123(
                self, FollowJointTrajectory,
                '/%s/follow_joint_trajectory' % name,
                callback_group=self._action_callback_group))
            for name in ("scaled_joint_trajectory_controller",
                         "joint_trajectory_controller")]
        self._joint_traj_client = None   # resolved on first use
        #self.broadcaster = self.create_publisher(Pose, "/dataguzzler_frame_publisher", 100)

#        self.pose_array_subscriber = self.create_subscription(
#            PoseArray,
#            '/downsampled_pose_array',
#            self.pose_array_callback,
#            10)


        for param in self._paramdict:
            self._params[param] = self._paramdict[param][1]
        
        for param in kwargs:
            self._params[param] = self._paramdict[param][2](kwargs[param])

        # For now, we'll run our own thread to query ROS -- not sure how threading works in the create_subscription method
        self._spinner = Thread(target=self.SpinnerThread, daemon=True)
        self._spinner.start()
        self.RestartAcquisition()
        # self.tari_noether_client()

    def camera_info_callback(self, msg):
        self.camera_info = msg
        return

    def gpio_callback(self, msg):
        self.svd_msg = msg
        return
    
    def tracking_callback(self, msg):
        self.tracked_detection = msg
        return
    
    def force_callback(self,msg):
        self.sync_force_msg = msg

    def position_callback(self,msg):
        self.sync_pos_msg = msg
    
    def detection_callback(self, msg):
        self.detected = msg
        return
    
    def click_callback(self, msg):
        self.click_msg = msg
        return
    
    def tag_callback(self, msg):
        self.tags = msg
        return
    
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        # center_x = self.depth_image.shape[1] // 2
        # center_y = self.depth_image.shape[0] // 2
        # self.depth_value = self.depth_image[center_y, center_x]
        return

    def specimenMesh(self,x=0,y=0,z=0,ox=0,oy=0,oz=0,ow=1,specimen_mesh=None):

        try:

            if specimen_mesh is not None:

                self.specimen_model_file = specimen_mesh

                transact = self.recdb.start_transaction()

                self.specimen_recording = snde.x3d_load_geometry(
                    transact,
                    self.graphman,
                    self.specimen_model_file,
                    0,
                    'main',
                    self.specimen_model_tree,
                    None,
                    ['reindex_vertices', 'reindex_tex_vertices'] + snde.project_onto_parameterization_processing_tags
                    )
                
                transact.end_transaction()

                f = math.sqrt(ow*ow + 
                              ox*ox + 
                              oy*oy + 
                              oz*oz)
                
                q = self.quatRot([ow/f,ox/f,oy/f,oz/f])

                f = math.sqrt(q[0]*q[0] + 
                              q[1]*q[1] + 
                              q[2]*q[2] + 
                              q[3]*q[3])

                self.specimen_position = np.array(((q[3],
                                               q[0],
                                               q[1],
                                               q[2]),(x,
                                               y,
                                               z,
                                               1)),dtype=orient_dtype)
                
                self._SpecimenPosition(self.specimen_position, self.specimen_model_tree, self.specimen_pose_chan)

            else:
                self.specimen_model_file = None

            
        except Exception as e:

            self.get_logger().info(f"{e}", once=True)

    def join(self, background_channel=None):

        if self.specimen_model_file is None: return

        if background_channel is None : background_channel=self.specimen_pose_channel

        transact = self.recdb.start_transaction()

        model_channel=snde.recdb_path_join(self.eoat_model_tree,"meshed")

        # print(f"{Colors.MAGENTA}Positioner channel: {self.positioner_channel}{Colors.RESET}")

        model_channel = str(model_channel)
        # print("Model Channel: %s" % model_channel)
        background_channel = str(background_channel)
        # print("Background Channel: %s" % background_channel)

        self.probe_position = np.array((( 
                                    self.arm_q[3],
                                    self.arm_q[0],
                                    self.arm_q[1],
                                    self.arm_q[2]),
                                    (self.t.transform.translation.x,
                                    self.t.transform.translation.y,
                                    self.t.transform.translation.z,
                                    1)),
                                  dtype=orient_dtype)
        
        vsop = snde.VectorOfStringOrientationPairs([snde.StringOrientationPair('/graphics/specimen/meshed',
                                                                               snde.snde_orientation3(self.specimen_position)),
                                                                               snde.StringOrientationPair('/graphics/probe/meshed',
                                                                               snde.snde_orientation3(self.probe_position))])
        rec = snde.create_assembly_recording(transact, self.positioner_chan, vsop)

        rec.mark_data_and_metadata_ready()

        transact.end_transaction()




    def _SpecimenPosition(self, orientation, tree, chan):
        self.get_logger().info("Specimen Position Set to: %s" % orientation, once=True)
        transact = self.recdb.start_transaction()
        specimen_pose_ref = snde.create_pose_channel_ndarray_ref(transact, chan, snde.recdb_path_join(tree, "meshed"))
        specimen_pose_ref.allocate_storage([1])
        specimen_pose_ref.data[0] = orientation
        specimen_pose_ref.rec.mark_metadata_done()
        specimen_pose_ref.rec.mark_data_ready()
        transact.end_transaction()

    def _NewEOATPositionInTransaction(self, orientation):
        transact = self.recdb.start_transaction()
        eoat_pose_ref = snde.create_pose_channel_ndarray_ref(transact,
                                                             self.eoat_pose_chan,
                                                             snde.recdb_path_join(self.eoat_model_tree, "meshed"))
        eoat_pose_ref.allocate_storage([1])
        # eoat_pose_rec = snde.pose_channel_recording.from_ndarray_recording(eoat_pose_ref.rec)
        eoat_pose_ref.data[0] = orientation
        eoat_pose_ref.rec.mark_metadata_done()
        eoat_pose_ref.rec.mark_data_ready()
        try:

            if self.specimen_model_file is not None:
                vsop = snde.VectorOfStringOrientationPairs(
                    [snde.StringOrientationPair('/graphics/specimen/meshed',
                        snde.snde_orientation3(self.specimen_position)),
                        snde.StringOrientationPair('/graphics/probe/meshed',
                        snde.snde_orientation3(self.probe_position))])
                rec = snde.create_assembly_recording(transact, self.positioner_chan, vsop)
                rec.mark_data_and_metadata_ready()
        except Exception as e:
            self.get_logger().info(f"{Colors.RED}{e}{Colors.RESET}", once=True)

        finally:
            transact.end_transaction()


    def Stop(self):
        self.get_logger().info('Canceling goal...')
        cancel_future = self._action_client.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)
        self.timer.cancel() 

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def CartesianMove(self, msg, vel=0.1, acc=0.1, ik_frame="probe_end", goal_timeout=15.0,
                      result_timeout=300.0, min_fraction=1.0):
        """One continuous Cartesian move through a list of poses.

        min_fraction is the smallest acceptable fraction of the requested path.
        The default 1.0 means the move fails outright rather than executing a
        short one: for a rectangular raster row a short pass does not degrade the
        c-scan, it fabricates the end of it, because every later trace is then at
        the wrong grid index.

        Lower it for a surface-following toolpath, where the segments are ragged
        and demanding 100% of each one fails often. There a short pass costs only
        coverage -- the traces carry their own poses, so nothing is misplaced.
        The result carries `.result.fraction`, so the caller can report exactly
        what it got either way.
        """
        goal_msg = RasterScan.Goal()

        goal_msg.velocity = vel
        goal_msg.acceleration = acc
        goal_msg.ik_frame = ik_frame
        goal_msg.min_fraction = float(min_fraction)

        if isinstance(msg, PoseArray):
            goal_msg.pose_array = msg

        elif isinstance(msg, list):
            goal_msg.pose_array.poses = msg
            goal_msg.pose_array.header.frame_id = "world"

        else:

            self.get_logger().error(f'Wrong type: {msg.__class__}')
            return False

        self.get_logger().info(
            f'execute_raster: {len(goal_msg.pose_array.poses)} poses, vel={vel}, acc={acc}, '
            f'min_fraction={min_fraction}')

        # send_goal_bounded, not send_goal -- see the note above JointMove().
        # result_timeout is the long one here: mtc_hub_node accepts immediately
        # and then plans and sweeps the whole row inside the execute callback.
        self._raster_scan_client.wait_for_server()
        return self._raster_scan_client.send_goal_bounded(
            goal_msg, goal_timeout=goal_timeout, result_timeout=result_timeout)

    def FreeMove(self, msg, vel=0.1, acc=0.1, ik_frame="probe_end", goal_timeout=15.0, result_timeout=120.0):

        goal_msg = PipelineMove.Goal()

        goal_msg.velocity = vel
        goal_msg.acceleration = acc
        goal_msg.ik_frame = ik_frame

        if isinstance(msg, TransformStamped):

            # The frame matters, and leaving it empty is not harmless. Without it
            # the goal carries a PoseStamped whose header.frame_id is "", MoveIt
            # cannot resolve what the pose is relative to, and PipelineHandler
            # hangs inside planning instead of failing -- mtc_hub_node logs
            # "Pose frame = ''" and never logs anything again. Because that
            # handler runs on a detached thread while the node is spun by a
            # single-threaded executor, the hang takes the WHOLE node down: it
            # stops answering execute_raster and execute_joint_move too, and the
            # only recovery is restarting mtc_node. The Pose branch below always
            # set the frame; this one never did.
            goal_msg.target_pose.header.stamp = rclpy.time.Time().to_msg()
            goal_msg.target_pose.header.frame_id = (msg.header.frame_id
                                                    or self.from_frame_rel)

            goal_msg.target_pose.pose.position.x = msg.transform.translation.x
            goal_msg.target_pose.pose.position.y = msg.transform.translation.y
            goal_msg.target_pose.pose.position.z = msg.transform.translation.z

            goal_msg.target_pose.pose.orientation.x = msg.transform.rotation.x
            goal_msg.target_pose.pose.orientation.y = msg.transform.rotation.y
            goal_msg.target_pose.pose.orientation.z = msg.transform.rotation.z
            goal_msg.target_pose.pose.orientation.w = msg.transform.rotation.w

        elif isinstance(msg, Pose):

            # .to_msg(), like the branch above. rclpy.time.Time() is an rclpy
            # object, not a builtin_interfaces.msg.Time; rosidl's Python setter
            # does not type-check it, so the wrong type is stored happily and the
            # C converter ABORTS THE WHOLE PROCESS when the goal is serialised:
            #   builtin_interfaces__msg__time__convert_from_py: Assertion
            #   `strncmp("builtin_interfaces.msg._time.Time", ...) == 0' failed
            # No traceback, no exception -- SIGABRT. Nothing in this tree passed
            # a bare Pose to FreeMove until the reconstruction sweep did, which
            # is why it survived; the TransformStamped branch was written
            # correctly when the frame_id fix went in and this one was missed.
            # to_frame_rel, NOT from_frame_rel, and this is the same fault
            # SolveIK() has a paragraph about: an unstamped Pose is in the
            # PLANNING frame, not the tool's own. For motion('ros', recdb,
            # 'probe_end', 'world') from_frame_rel is 'probe_end', so a bare
            # Pose in world coordinates was handed to MoveIt as a pose relative
            # to the very link being moved. mtc_hub_node logs it plainly --
            #   Creating Pipeline Task -> Goal frame: 'probe_end' ...
            #   [ompl] Unable to sample any valid states for goal tree
            # -- and the goal is simply ABORTED, with a target that is
            # individually IK-solvable and collision-free when asked about
            # properly. The TransformStamped branch above never had this because
            # ros.t carries frame_id='world'; only callers passing a bare Pose
            # were affected, which until the reconstruction sweep was nobody.
            goal_msg.target_pose.header.stamp = rclpy.time.Time().to_msg()
            goal_msg.target_pose.header.frame_id = self.to_frame_rel
            goal_msg.target_pose.pose = msg

        else:

            self.get_logger().error(f'Wrong type: {msg.__class__}')
            return False

        # send_goal_bounded, not send_goal -- see the note above JointMove().
        self._pipeline_move_client.wait_for_server()
        return self._pipeline_move_client.send_goal_bounded(
            goal_msg, goal_timeout=goal_timeout, result_timeout=result_timeout)

    # Every action call in this module goes through send_goal_bounded rather
    # than rclpy's send_goal. The stock one waits on an event with no timeout,
    # so a goal response lost to ros2/rclpy#1123 wedges the caller permanently
    # -- a scan that stops partway with no error is worse than one that fails.
    #
    # Splitting the two timeouts matters because mtc_hub_node's handlers all
    # return ACCEPT_AND_EXECUTE straight away and do the planning inside the
    # execute callback. So goal_timeout only has to cover the accept round trip
    # (15 s is generous), while result_timeout has to cover planning -- which
    # retries, up to Planning Attempts x Planning Time -- plus the motion.
    #
    # Note the retry resends rather than waits: the server accepted and started
    # the first goal, so a resend overlaps it instead of following it. That is
    # still the right trade for a lost response, which is rare, and the arm ends
    # at the commanded target either way -- but it is why the timeouts want to
    # be comfortably longer than the real worst case, not tight.
    def JointMove(self, msg, vel=0.1, acc=0.1, goal_timeout=15.0, result_timeout=120.0,
                  joint_names=None, allow_contact_links=None, allow_contact_objects=None):
        """Joint-space move through MoveIt: a named SRDF state, or joint values.

        `msg` is either a string naming an SRDF group state ("over_table") or a
        sequence of joint values. Both go to the same action, which plans with
        MoveIt's joint interpolation planner -- so the arm ends in exactly the
        configuration asked for, and every interpolated step on the way there is
        checked against the planning scene.

        allow_contact_links / allow_contact_objects excuse expected contact; see
        MoveJoints() and JointMove.action. Leave them alone for standoff work.
        """
        goal_msg = JointMove.Goal()
        goal_msg.velocity = vel
        goal_msg.acceleration = acc
        goal_msg.allow_contact_links = list(allow_contact_links or [])
        goal_msg.allow_contact_objects = list(allow_contact_objects or [])

        if isinstance(msg, str):
            goal_msg.target_state = msg
        elif msg is not None and not isinstance(msg, (bytes, dict)):
            try:
                positions = [float(v) for v in msg]
            except TypeError:
                self.get_logger().error(f'Wrong type: {msg.__class__}')
                return False
            goal_msg.target_joints = positions
            goal_msg.target_joint_names = list(joint_names or self.UR_JOINTS)
        else:
            self.get_logger().error(f'Wrong type: {msg.__class__}')
            return False

        self._joint_move_client.wait_for_server()
        return self._joint_move_client.send_goal_bounded(
            goal_msg, goal_timeout=goal_timeout, result_timeout=result_timeout)


    UR_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

    def SolveIK(self, msg, seed=None, group="ur_manipulator", ik_frame="probe_end",
                avoid_collisions=True, timeout=2.0):
        """Joint values for a pose, optionally in the same branch as `seed`.

        Returns a list in UR_JOINTS order, or None if there is no solution.

        `seed` is what makes this useful. An arm has several configurations that
        put the tool at the same pose, and they are not interchangeable: one can
        continue along a path and another runs out of joint travel partway. Pass
        the joint values of a configuration you want to stay near and the solver
        returns the solution in that branch.
        """
        # An unstamped pose is in the PLANNING frame, not the tool's own. This
        # used to fall back to from_frame_rel, which for motion('ros', recdb,
        # 'probe_end', 'world') is 'probe_end' -- so /compute_ik was asked for a
        # pose relative to the very link it was solving for, and returned no
        # solution for every target. It reads as an unreachable workspace: a
        # toolpath over ground the raster scan covers daily came back 0 of 10
        # solvable. to_frame_rel is the world frame this module was constructed
        # with, which is what a caller handing over a bare Pose means.
        if isinstance(msg, TransformStamped):
            pose = Pose()
            pose.position.x = msg.transform.translation.x
            pose.position.y = msg.transform.translation.y
            pose.position.z = msg.transform.translation.z
            pose.orientation = msg.transform.rotation
            frame = msg.header.frame_id or self.to_frame_rel
        elif isinstance(msg, PoseStamped):
            pose = msg.pose
            frame = msg.header.frame_id or self.to_frame_rel
        elif isinstance(msg, Pose):
            pose = msg
            frame = self.to_frame_rel
        else:
            raise TypeError("SolveIK needs a Pose/PoseStamped/TransformStamped, not %r"
                            % (msg.__class__,))

        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.ik_link_name = ik_frame
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout))*1e9)
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        if seed is not None:
            rs = RobotState()
            js = JointState()
            js.name = list(self.UR_JOINTS)
            js.position = [float(v) for v in seed]
            rs.joint_state = js
            req.ik_request.robot_state = rs

        if not self._compute_ik_client.wait_for_service(timeout_sec=5.0):
            return None
        future = self._compute_ik_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not event.wait(max(timeout*4, 10.0)):
            return None
        res = future.result()
        if res is None or res.error_code.val != 1:
            return None
        found = dict(zip(res.solution.joint_state.name,
                         res.solution.joint_state.position))
        try:
            return [found[j] for j in self.UR_JOINTS]
        except KeyError:
            return None

    def CheckStateValidity(self, positions, joint_names=None, group="ur_manipulator",
                           allow_contact_links=None, allow_contact_objects=None,
                           timeout=5.0):
        """Is this joint configuration collision-free? (None = could not tell.)

        Returns True, False, or None -- and the three are genuinely different.
        None means the service did not answer, which is NOT permission to move:
        the caller has to decide whether an unverifiable target is acceptable,
        and the answer next to real fixturing is no.

        allow_contact_links / allow_contact_objects express "contact is expected
        here". The service reports the colliding pairs, so rather than turning
        collision checking off, this looks at what is actually touching and
        excuses only the contacts the caller declared. With objects named, only
        those pairs pass; with links alone, any contact involving one of those
        links passes -- which is blunt, and is what remains available while the
        specimen is not modelled and so has no name to pair against.
        """
        allow_links = set(allow_contact_links or [])
        allow_objects = set(allow_contact_objects or [])

        req = GetStateValidity.Request()
        req.group_name = group
        rs = RobotState()
        js = JointState()
        js.name = list(joint_names or self.UR_JOINTS)
        js.position = [float(v) for v in positions]
        rs.joint_state = js
        # The service checks the state as given against the CURRENT scene, and a
        # partial joint state is completed from the monitored state -- so naming
        # only the arm joints is correct and does not silently zero the rest.
        rs.is_diff = True
        req.robot_state = rs

        if not self._state_validity_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning(
                "/check_state_validity did not appear within %.1f s; "
                "cannot verify the joint target" % timeout)
            return None
        future = self._state_validity_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not event.wait(max(timeout*2, 10.0)):
            self.get_logger().warning("/check_state_validity did not answer")
            return None
        res = future.result()
        if res is None:
            return None
        if res.valid:
            return True

        if not allow_links and not allow_objects:
            for c in res.contacts:
                self.get_logger().error(
                    "joint target in collision: %s <-> %s" % (c.contact_body_1, c.contact_body_2))
            return False

        # Contact was declared expected. Check that every reported contact is one
        # of the declared ones -- an undeclared contact elsewhere on the arm is
        # still a fault, and this is the difference between "the probe is allowed
        # to touch the part" and "collision checking is off".
        if not res.contacts:
            # Invalid with no contacts reported means something other than
            # collision (joint limits, a path constraint). Not ours to excuse.
            self.get_logger().error(
                "joint target reported invalid with no contacts -- joint limits "
                "or a constraint, not something allow_contact can excuse")
            return False

        unexpected = []
        for c in res.contacts:
            bodies = (c.contact_body_1, c.contact_body_2)
            if allow_objects:
                ok = ((bodies[0] in allow_links and bodies[1] in allow_objects) or
                      (bodies[1] in allow_links and bodies[0] in allow_objects))
            else:
                ok = bodies[0] in allow_links or bodies[1] in allow_links
            if ok:
                self.get_logger().warning(
                    "expected contact: %s <-> %s" % bodies)
            else:
                unexpected.append(bodies)
        for bodies in unexpected:
            self.get_logger().error("unexpected contact: %s <-> %s" % bodies)
        return not unexpected

    def MoveJoints(self, positions, secs=5.0, joint_names=None,
                   goal_timeout=15.0, result_timeout=120.0,
                   plan=True, validate=True,
                   allow_contact_links=None, allow_contact_objects=None):
        """Drive the arm to an explicit joint configuration.

        The configuration is the point. A pose goal lets the planner pick
        whichever branch it likes, and the ones it picks routinely cannot follow
        a raster row to its far end -- which is the failure this exists to solve.
        So the target is always the exact configuration asked for; the only
        question is what checks it on the way.

        `plan=True` (default) sends it through MoveIt's execute_joint_move, which
        interpolates in joint space and checks every step against the planning
        scene. Same motion, checked. That is what makes this usable next to real
        fixturing rather than only against Gazebo.

        If no such server is answering -- an older mtc_node, or a cell running
        without it -- this falls back to /check_state_validity on the target plus
        a direct trajectory-controller command. That is weaker: it checks the
        endpoint, not the path. It says so in the log when it does it.

        A PLANNING FAILURE IS NOT A REASON TO FALL BACK, and does not. MoveIt
        refusing the move means something is in the way; commanding it anyway
        would make the check decorative. Pass plan=False if you have decided to
        move regardless, which still validates the endpoint unless you also pass
        validate=False.

        allow_contact_links / allow_contact_objects declare expected contact --
        contact scanning demands contact, so the probe touching the specimen
        cannot be an error. They are honoured by both paths.
        """
        names = list(joint_names or self.UR_JOINTS)
        positions = [float(v) for v in positions]

        if plan:
            if self._joint_move_client.wait_for_server(timeout_sec=5.0):
                result = self.JointMove(positions, joint_names=names,
                                        goal_timeout=goal_timeout,
                                        result_timeout=result_timeout,
                                        allow_contact_links=allow_contact_links,
                                        allow_contact_objects=allow_contact_objects)
                if getattr(getattr(result, "result", None), "success", False):
                    return result
                self.get_logger().error(
                    "MoveIt refused the joint move. NOT falling back to a direct "
                    "command -- something is in the way, or the target is out of "
                    "limits. Pass plan=False to override deliberately.")
                return result
            self.get_logger().warning(
                "no execute_joint_move server; falling back to a direct trajectory "
                "command with an endpoint-only validity check. The PATH is not "
                "checked on this route.")

        if validate:
            valid = self.CheckStateValidity(
                positions, joint_names=names,
                allow_contact_links=allow_contact_links,
                allow_contact_objects=allow_contact_objects)
            if valid is not True:
                self.get_logger().error(
                    "refusing to command joint target: validity is %s"
                    % ("unknown" if valid is None else "invalid",))
                return None

        return self._command_joints(positions, secs=secs, joint_names=names,
                                    goal_timeout=goal_timeout,
                                    result_timeout=result_timeout)

    def _command_joints(self, positions, secs=5.0, joint_names=None,
                        goal_timeout=15.0, result_timeout=120.0):
        """Straight to the trajectory controller. No planner, no checks.

        Separated out so that "command the arm" and "decide whether commanding
        the arm is safe" are not the same function. Use MoveJoints().
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names or self.UR_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(secs)
        point.time_from_start.nanosec = int((secs - int(secs))*1e9)
        goal.trajectory.points.append(point)

        client = self._resolve_joint_traj_client()
        return client.send_goal_bounded(
            goal, goal_timeout=goal_timeout, result_timeout=result_timeout)

    def _resolve_joint_traj_client(self, timeout=5.0):
        """Whichever trajectory controller is actually running here."""
        if self._joint_traj_client is not None:
            return self._joint_traj_client
        for name, client in self._joint_traj_clients:
            if client.wait_for_server(timeout_sec=timeout):
                self.get_logger().info("MoveJoints using %s" % name)
                self._joint_traj_client = client
                return client
        raise RuntimeError(
            "no trajectory controller found -- tried %s"
            % ", ".join(n for n, _ in self._joint_traj_clients))

    def ApproachInConfiguration(self, start, end, secs=6.0,
                                plan=True, allow_contact_links=None,
                                allow_contact_objects=None, **kwargs):
        """Put the arm at `start` in a configuration that can also reach `end`.

        This is the fix for silently truncated raster rows. Moving to the start
        pose the ordinary way lets the planner choose any configuration that
        reaches it, and the ones it picks routinely cannot follow the row to the
        far end -- computeCartesianPath then truncates partway and, before it was
        made to fail, executed the short path as if nothing had happened. Every
        pose on the row is individually reachable; it is the configuration the
        row is entered in that decides how far it gets.

        Solving the END first and seeding the START from it picks a configuration
        that demonstrably reaches both, so the straight line between them stays
        in one branch. Measured on a 60 waypoint, 0.30 m row that had been
        completing 65-88%: 100.0%.

        The move itself goes through MoveIt by default (see MoveJoints), so the
        chosen configuration is reached exactly and the path to it is collision
        checked. Choosing the configuration and checking the way to it are
        separate concerns, and only the first one needs the planner kept out of
        it.

        Returns the joint configuration used, or None if IK failed or the move
        was refused -- in which case the caller should fall back to an ordinary
        Move and will probably get a short row, or stop.
        """
        q_end = self.SolveIK(end, **kwargs)
        if q_end is None:
            return None
        q_start = self.SolveIK(start, seed=q_end, **kwargs)
        if q_start is None:
            return None
        result = self.MoveJoints(q_start, secs=secs, plan=plan,
                                 allow_contact_links=allow_contact_links,
                                 allow_contact_objects=allow_contact_objects)
        if result is None:
            return None
        # The planned path reports whether it executed. The direct fallback
        # returns the controller's own result, which has no .result.success --
        # treat anything that came back at all as commanded, as before.
        inner = getattr(result, "result", None)
        if inner is not None and hasattr(inner, "success") and not inner.success:
            return None
        return q_start

    def Move(self, msg, vel=0.1, acc=0.1, ik_frame="probe_end"):

        goal_msg = DgPython.Goal()
        
        goal_msg.velocity = vel
        goal_msg.acceleration = acc
        goal_msg.ik_frame = ik_frame


        if isinstance(msg, Pose):
            goal_msg.destination = msg
        
        elif isinstance(msg, PoseArray):
            # goal_msg.waypoints = msg
            goal_msg.pose_array = msg

        elif isinstance(msg, TransformStamped):
            t = Pose()
            p = Point()

            t.position.x = msg.transform.translation.x
            t.position.y = msg.transform.translation.y
            t.position.z = msg.transform.translation.z

            t.orientation.x = msg.transform.rotation.x
            t.orientation.y = msg.transform.rotation.y
            t.orientation.z = msg.transform.rotation.z
            t.orientation.w = msg.transform.rotation.w

            self.desired_t = t
            
            goal_msg.destination = self.desired_t
        elif isinstance(msg, list):
            goal_msg.pose_array.poses = msg
            goal_msg.pose_array.header.frame_id = "world"
            self.laser_scan_array_pub.publish(goal_msg.pose_array)

        elif isinstance(msg, str):

            goal_msg.state = msg

        else:
            return 'Invalid variable type: ', msg.__class__

        self._action_client.wait_for_server()
        return self._action_client.send_goal_bounded(goal_msg)

#     def Move(self, msg, vel=0.1, acc=0.1, ik_frame="probe_end"):
# 
#         goal_msg = RasterScan.Goal()
#         
#         goal_msg.velocity = vel
#         goal_msg.acceleration = acc
#         goal_msg.ik_frame = ik_frame
# 
# 
#         if isinstance(msg, Pose):
#             print(f"Pose: {goal_msg}")
#             goal_msg.pose_array.poses.append(msg)
#         
#         elif isinstance(msg, PoseArray):
#             print(f"PoseArray: {goal_msg}")
#             # goal_msg.waypoints = msg
#             goal_msg.pose_array = msg
# 
#         elif isinstance(msg, TransformStamped):
#             print(f"TransformStamped: {goal_msg}")
#             t = Pose()
#             p = Point()
# 
#             t.position.x = msg.transform.translation.x
#             t.position.y = msg.transform.translation.y
#             t.position.z = msg.transform.translation.z
# 
#             t.orientation.x = msg.transform.rotation.x
#             t.orientation.y = msg.transform.rotation.y
#             t.orientation.z = msg.transform.rotation.z
#             t.orientation.w = msg.transform.rotation.w
# 
#             self.desired_t = t
#             
#             goal_msg.pose_array.poses.append(self.desired_t)
#         elif isinstance(msg, list):
#             print(f"list: {goal_msg}")
#             goal_msg.pose_array.poses = msg
#             goal_msg.pose_array.header.frame_id = "world"
#             self.laser_scan_array_pub.publish(goal_msg.pose_array)
# 
#         elif isinstance(msg, str):
# 
#             goal_msg = JointMove.Goal()
#         
#             goal_msg.velocity = vel
#             goal_msg.acceleration = acc
# 
#             goal_msg.target_state = msg
#             self._joint_move_client.wait_for_server()
#             return self._joint_move_client.send_goal(goal_msg)
# 
#         else:
#             return 'Invalid variable type: ', msg.__class__
# 
#         self._raster_scan_client.wait_for_server()
#         return self._raster_scan_client.send_goal(goal_msg)
    

#    def pose_array_callback(self, msg):
#        
#        self.get_logger().info("Received pose array")
#        
#        self.get_logger().info("First pose in PoseArray:")
#        
#        self.get_logger().info("Position: (%.3f, %.3f, %.3f)", msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z)
#        
#        self.get_logger().info("Orientation: (%.3f, %.3f, %.3f, %.3f)", msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w)
#        
#        for pose in msg.poses:
#            
#            self.desired_pose = pose
#
#            goal_msg = DgPython.Goal()
#            goal_msg.destination = self.desired_pose
#
#            self._action_client.wait_for_server()
#            self._action_client.send_goal(goal_msg)

    def _GetParam(self, name):
        return self._paramdict[name][3](self._params[name])

    def _SetParam(self, name, value):
        tmpparams = copy.deepcopy(self._params)
        tmpparams[name] = self._paramdict[name][2](value)
        self.WaitPauseAcquisition()
        self._task.stop()
        try:
            self._task.timing.cfg_samp_clk_timing(tmpparams['SampleRate'], sample_mode=tmpparams['AcquisitionType'], samps_per_chan=tmpparams['SampsPerChan'])
        except Exception as e:
            raise(e)
        else:
            self._params = tmpparams
        finally:
            self.WaitUnPauseAcquisition()        
        return self._GetParam(name)

    def WaitStopAcquisition(self):
        if self._thread is not None:
            self._quit = True
            while self._thread.is_alive():
                time.sleep(0.01)
                pass
        return

    def WaitPauseAcquisition(self):
        if self._thread is not None and self._thread.is_alive():
            self._pause = True
            while self._paused == False:
                time.sleep(0.01)
                pass
        return

    def WaitUnPauseAcquisition(self):
        if self._thread is not None and self._thread.is_alive() and self._paused == True:
            self._pause = False
            while self._paused == True:
                time.sleep(0.01)
                pass
        return

    def RestartAcquisition(self):
        if self._thread is not None:
            self._quit = True
            while self._thread.is_alive():
                time.sleep(0.01)
                pass  
        
        self._quit = False
        self._pause = False
        self._paused = False
        self._thread = Thread(target=self.AcquisitionThread, daemon=True)
        self._thread.start()
 

    
    def AcquisitionThread(self):
        InitCompatibleThread(self, "_thread")

        transform = TransformStamped()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        r = self.create_rate(200)

        while not self._quit:
            if self._pause == True:
                self._paused = True
                while self._pause == True:
                    time.sleep(0.5)
                    pass
                self._paused = False

            try:
                self.t = self._tf_buffer.lookup_transform(
                        self.to_frame_rel,
                        self.from_frame_rel,
                        rclpy.time.Time())

                # Publish to the context-free holder first: this thread keeps
                # running while a move holds the module context, and the holder
                # is the only way anything outside can see the pose meanwhile.
                self.live_pose.update(self.t, time.monotonic())

                self.probe_end_pub.publish(self.t)

                self.azure_t = self._tf_buffer.lookup_transform(
                        self.to_frame_rel,
                        'azure_rgb',
                        rclpy.time.Time())
                
                self.keyence_t = self._tf_buffer.lookup_transform(
                        'probe_end',
                        'lj_v7080_optical_frame',
                        rclpy.time.Time())
                # e = transforms3d.euler.quat2euler(self.t.transform.rotation.w,
                #                               self.t.transform.rotation.x,
                #                               self.t.transform.rotation.y,
                #                               self.t.transform.rotation.z)
                # e_tmp = e
                # e[1] = e_tmp[2] * -1
                # e[2] = e_tmp[1]
            
                d = math.sqrt(self.t.transform.rotation.w*self.t.transform.rotation.w + 
                              self.t.transform.rotation.x*self.t.transform.rotation.x + 
                              self.t.transform.rotation.y*self.t.transform.rotation.y + 
                              self.t.transform.rotation.z*self.t.transform.rotation.z)

                self.arm_q = self.quatRot([self.t.transform.rotation.w/d,
                                  self.t.transform.rotation.x/d,
                                  self.t.transform.rotation.y/d,
                                  self.t.transform.rotation.z/d])


                if self.eoat_mesh is not None:
                    self._NewEOATPositionInTransaction(np.array(((
                                                                  [self.arm_q[3]/d,
                                                                  self.arm_q[0]/d,
                                                                  self.arm_q[1]/d,
                                                                  self.arm_q[2]/d]),
                                                                  (self.t.transform.translation.x,
                                                                  self.t.transform.translation.y,
                                                                  self.t.transform.translation.z,
                                                                  1)),dtype=orient_dtype))
                try:
                    if self.positioner_chan:
                        # self.get_logger().info(f'Poistioner channel {self.counter} found', throttle_duration_sec=3)
                        self.join()
                except Exception as e:
                    self.get_logger().info(f"{e}", once=True)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}', once=True)

            try:

                self.azure_t = self._tf_buffer.lookup_transform(
                        self.to_frame_rel,
                        'azure_rgb',
                        rclpy.time.Time())
                
            except Exception as e:

                self.get_logger().info(f"Azure trasnform error: {e}", once=True)

            r.sleep()

    
    def quatRot(self, quat):

                q = []

                w = 0.7071
                x = 0.7071

                q.append(quat[0]*x+quat[1]*w)
                q.append(quat[2]*w+quat[3]*x)
                q.append(quat[3]*w-quat[2]*x)
                q.append(quat[0]*w-quat[1]*x)

                return q

    # clientCreation(StartReconstruction, "start_reconstruction", TariReconstruction)

    def clientCreation(self, action, serviceType, serviceName):
        if action:
            cli = ActionClient(self, serviceType, serviceName)
        else:
            cli = self.create_client(serviceType, serviceName)
            # while not cli.wait_for_service(timeout_sec=1.0):
            #     self.get_logger().info(f'{serviceName} service not available, waiting again...')

        ret = client(cli, serviceType, serviceName)        
        
        return ret
       
    def domePoseArray(self, full_dome, x, y, rot=False, angle=12, length = 0.3, width = 0.15, x_pos = 0.655, y_pos = -0.1, z_pos = 0.946, x_offset=1, y_offset=1):
        recon_pose_array = PoseArray()
        recon_pose_array.header.frame_id = "world"
        x_unit = length/x
        y_unit = width/y
        if angle != 0:
            x_angle = math.pi/angle
            y_angle = math.pi/angle
        else:
            x_angle = 0
            y_angle = 0
        for e in range(y):
            for i in range(x):
                pose = Pose()
                pose.position.x = x_pos + x_unit * i
                pose.position.y = y_pos + y_unit * e
                pose.position.z = z_pos
                if rot:
                    if full_dome:
                        q = transforms3d.euler.euler2quat(-y_angle*math.sin((e-((y-1)/2))*math.pi/(y-1)), x_angle*math.sin((i-((x-x_offset)/2))*math.pi/((x-1))), 0, 'sxyz')
                    else:
                        q = transforms3d.euler.euler2quat(-y_angle*math.sin((e-((y-1)/2))*math.pi/(y-1)), x_angle*math.sin((i-(x-1))*math.pi/((x-1)*2)), 0, 'sxyz')
                    pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
                else:
                    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                recon_pose_array.poses.append(pose)
        rclpy.logging.get_logger("recon_pose_array").info(f"Pose array has {len(recon_pose_array.poses)} poses")
        self.dome_pose_pub.publish(recon_pose_array)
        return recon_pose_array
     
    def addPose(self, pose):
        
        if isinstance(pose, Pose):
            self.get_logger().info("Got Pose")
            self.pose_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.get_logger().info("Got Pose Array")
            self.pose_array.poses = pose.poses

        elif isinstance(pose, TransformStamped):
            self.get_logger().info("Got Transform")
            t = Pose()

            t.position.x = pose.transform.translation.x
            t.position.y = pose.transform.translation.y
            t.position.z = pose.transform.translation.z

            t.orientation.x = pose.transform.rotation.x
            t.orientation.y = pose.transform.rotation.y
            t.orientation.z = pose.transform.rotation.z
            t.orientation.w = pose.transform.rotation.w

            self.pose_array.poses.append(t)

        elif isinstance(pose, list):
            self.get_logger().info("Got List")
            self.pose_array.poses = pose
        
        elif isinstance(pose, PointStamped):
            self.get_logger().info("Got Point")
            t = Pose()

            t.position.x = pose.point.x
            t.position.y = pose.point.y
            t.position.z = pose.point.z

            self.pose_array.poses.append(t)

        else:

            self.get_logger("Add Pose").error("Wrong type")
            return

        # print(f"Pose array is now:\n\n{self.pose_array}\n")


        return self.laser_scan_array_pub.publish(self.pose_array)
    
    def addCross(self, pose):

        if isinstance(pose, Pose):
            self.coarse_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.coarse_array.poses = pose.poses

        else:
            rclpy.logging.get_logger("Add Pose").error("Wrong type")
            return
        
        return self.coarse_scan_pub.publish(self.coarse_array)
    
    
    def addCoordinate(self, pose):

        if isinstance(pose, Pose):
            self.coordinate_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.coordinate_array.poses = pose.poses

        else:
            rclpy.logging.get_logger("Add Pose").error("Wrong type")
            return
        
        return self.angled_corners_pub.publish(self.coordinate_array)
    
    def addDetection(self, pose):

        if isinstance(pose, Pose):
            self.yolo_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.yolo_array.poses = pose.poses

        else:
            rclpy.logging.get_logger("Add Pose").error("Wrong type")
            return
        
        return self.object_detection_pub.publish(self.yolo_array)
    
    def pubPoseArray(self, pose_array):
        return self.laser_scan_array_pub.publish(pose_array)

    def SpinnerThread(self):
        InitCompatibleThread(self, "_spinner")

        # rclpy.spin() is single threaded: one callback at a time for the whole
        # node. spin_once() on a multi threaded executor dispatches to the pool
        # and returns, so TF, images and action responses proceed in parallel
        # instead of taking turns.
        executor = DgpyMultiThreadedExecutor(self, num_threads=4)
        executor.add_node(self)
        try:
            while not self._quit:
                executor.spin_once(timeout_sec=0.1)
        finally:
            executor.remove_node(self)
            executor.shutdown()

