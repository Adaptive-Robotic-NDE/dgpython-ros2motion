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
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
from dgpython_ros2_actioninterface.action import DgPython
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, WrenchStamped
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Quaternion, TransformStamped, Point
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import Image
from ur_msgs.msg import IOStates
from threading import RLock
from typing import Any
from rclpy.task import Future
# from tari_interfaces.action import PoseArrayHandler
# from tari_interfaces.srv import TariNoether
# from industrial_reconstruction_msgs.srv import StartReconstruction, StopReconstruction
# from industrial_reconstruction_msgs.msg import NormalFilterParams, RGBDImageParams, TSDFVolumeParams

ur = pint.get_application_registry()
orient_dtype = [('offset', '<f4', (4,)), ('quat', '<f4', (4,))]


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

    def sendRequest(self, interface, options):
        req = self.interfaces[interface].Request()
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

    def __init__(self, module_name, recdb, from_frame_rel, to_frame_rel, eoat_mesh=None, **kwargs):
        super().__init__('dataguzzler_frame_publisher')
        self.get_logger().info("Dataguzzler Frame Publisher Module Starting")
        self.pose_pub = self.create_publisher(PoseArray, 'pose_array', 10)
        self.coordinate_pub = self.create_publisher(PoseArray, 'coordinate_array', 10)
        self.center_pub = self.create_publisher(PoseArray, 'center_array', 10)
        self.yolo_pub = self.create_publisher(PoseArray, 'yolo_array', 10)
        self.recon_pose_pub = self.create_publisher(PoseArray, 'recon_pose_array', 10)
        self.gpio_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.gpio_callback, 10)
        self.detect_sub = self.create_subscription(DetectionArray, '/yolo/detections', self.detection_callback, 10)
        self.tracking_sub = self.create_subscription(DetectionArray, '/yolo/tracking', self.tracking_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_to_rgb/image_raw', self.depth_image_callback, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.force_callback, 10)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        self.bridge = CvBridge()
        self.gpio_sub
        self.counter = 0
        self.detections = DetectionArray()
        self.tracked_detections = DetectionArray()
        self.svd_msg = IOStates()
        self.force_msg = WrenchStamped()
        self.click_msg = PointStamped()
        self.click_msg.header.frame_id = to_frame_rel
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = to_frame_rel
        self.center_array = PoseArray()
        self.center_array.header.frame_id = to_frame_rel
        self.coordinate_array = PoseArray()
        self.coordinate_array.header.frame_id = to_frame_rel
        self.yolo_array = PoseArray()
        self.yolo_array.header.frame_id = to_frame_rel
        self.module_name = module_name
        self._params = {}
        self.from_frame_rel = from_frame_rel
        self.to_frame_rel = to_frame_rel
        self.t = TransformStamped()
        self.desired_t = None
        self.specimen_pose_channel = []
        self.specimen_model_file = []
        self.specimen_model_tree = []
        self.specimen_data_channel = []
        self.specimen_data_chan = []
        self.specimen_pose_chan = []
        self.positioner_channel = []
        self.specimen_position = []
        self.positioner_chan = []
        self.probe_position = None
        self.recdb = recdb
        self.graphman = snde.graphics_storage_manager("/graphics/", self.recdb.lowlevel_alloc, self.recdb.alignment_requirements,self.recdb.lockmgr, 1e-8, 2000000000)

        if eoat_mesh is not None:
            self.eoat_mesh = eoat_mesh
            self.eoat_model_tree = '/graphics/probe/'
            self.eoat_pose_channel = '/probe_pose'
            transact = self.recdb.start_transaction()
            self.graphicsgroup = self.recdb.define_channel('/graphics/', 'main', self.recdb.raw(), False, self.graphman)
            self.eoat_recording = snde.x3d_load_geometry(self.recdb, self.graphman, self.eoat_mesh, 0, 'main', self.recdb.raw(), self.eoat_model_tree, None, ['reindex_vertices', 'reindex_tex_vertices'])
            self.eoat_pose_chan = self.recdb.define_channel(self.eoat_pose_channel, "main", self.recdb.raw())
            self._NewEOATPositionInTransaction(snde.snde_invalid_orientation3())
            transact.end_transaction()

            
        else:
            self.eoat_mesh = None


        self._action_client = ActionClient(self, DgPython, 'dgpython')
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

    def gpio_callback(self, msg):
        self.svd_msg = msg
        return
    
    def tracking_callback(self, msg):
        self.tracked_detection = msg
        return
    
    def force_callback(self,msg):
        self.force_msg = msg
    
    def detection_callback(self, msg):
        self.detected = msg
        return
    
    def click_callback(self, msg):
        self.click_msg = msg
        return
    
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        # center_x = self.depth_image.shape[1] // 2
        # center_y = self.depth_image.shape[0] // 2
        # self.depth_value = self.depth_image[center_y, center_x]
        return

    def specimenMesh(self,x=0,y=0,z=0,ox=0,oy=0,oz=0,ow=1,specimen_mesh=None):
        if specimen_mesh is not None:
            self.counter += 1
            self.specimen_model_file.append(specimen_mesh)
            self.specimen_model_tree.append('/graphics/specimen_' + str(self.counter) + '/')
            self.specimen_pose_channel.append('/specimen_' + str(self.counter) + '_pose')
            try:
                transact = self.recdb.start_transaction()
                self.specimen_data_channel.append("/specimen_" + str(self.counter) + "_data")
                self.specimen_data_chan.append(self.recdb.define_channel(self.specimen_data_channel[self.counter-1],"main",self.recdb.raw()))
                self.specimen_recording = snde.x3d_load_geometry(self.recdb, self.graphman, self.specimen_model_file[self.counter-1], 0, 'main', self.recdb.raw(), self.specimen_model_tree[self.counter-1], None, ['reindex_vertices', 'reindex_tex_vertices'] + snde.project_onto_parameterization_processing_tags)
                self.specimen_pose_chan.append(self.recdb.define_channel(self.specimen_pose_channel[self.counter-1], "main", self.recdb.raw()))
                self.positioner_channel.append("/probe_and_specimen_" + str(self.counter))
                self.positioner_chan.append(self.recdb.define_channel(self.positioner_channel[self.counter-1],"main",self.recdb.raw()))            
                f = math.sqrt(ow*ow + 
                              ox*ox + 
                              oy*oy + 
                              oz*oz)
                
                q = self.quatRot([ow/f,ox/f,oy/f,oz/f])

                try:
                    self.specimen_position[self.counter-1] = np.array(((x,
                                                   y,
                                                   z,
                                                   0),(q[0],q[1],q[2],q[3])),dtype=orient_dtype)
                except:
                    self.specimen_position.append(np.array(((x,
                                                   y,
                                                   z,
                                                   0),(q[0],q[1],q[2],q[3])),dtype=orient_dtype))
                self._SpecimenPosition(self.specimen_position[self.counter-1], self.specimen_model_tree[self.counter-1], self.specimen_pose_chan[self.counter-1])
            finally:
                transact.end_transaction()
                # self.join(specimenNum=self.counter)

        else:
            self.specimen_mesh = None

    def join(self, background_channel=None, specimenNum=1):

        if self.counter == 0 : return print(f"{Colors.RED}No specimens available{Colors.RESET}")

        elif specimenNum-1 > self.counter : return print(f"{Colors.RED}Specimen index out of range{Colors.RESET}")

        if background_channel is None : background_channel=self.specimen_pose_channel[specimenNum-1]

        model_channel=snde.recdb_path_join(self.eoat_model_tree,"meshed")

        # print(f"{Colors.MAGENTA}Positioner channel: {self.positioner_channel}{Colors.RESET}")

        model_channel = str(model_channel)
        # print("Model Channel: %s" % model_channel)
        background_channel = str(background_channel)
        # print("Background Channel: %s" % background_channel)

        self.probe_position = np.array(((self.t.transform.translation.x,
                                    self.t.transform.translation.y,
                                    self.t.transform.translation.z,
                                    0),( 
                                    self.arm_q[0],
                                    self.arm_q[1],
                                    self.arm_q[2],
                                    self.arm_q[3])),
                                  dtype=orient_dtype)
        vsop = snde.VectorOfStringOrientationPairs([snde.StringOrientationPair('/graphics/specimen_'+ str(specimenNum) +'/meshed',snde.snde_orientation3(self.specimen_position[specimenNum-1])), snde.StringOrientationPair('/graphics/probe/meshed',snde.snde_orientation3(self.probe_position))])
        rec = snde.create_assembly_recording(self.recdb, self.positioner_chan[specimenNum-1], self.recdb.raw(), vsop)
        rec.mark_data_and_metadata_ready()



    def _SpecimenPosition(self, orientation, tree, chan):
        specimen_pose_ref = snde.create_pose_channel_ndarray_ref(self.recdb, chan, self.recdb.raw(), snde.recdb_path_join(tree, "meshed"))
        specimen_pose_ref.allocate_storage([1])
        specimen_pose_ref.data()[0] = orientation
        specimen_pose_ref.rec.mark_metadata_done()
        specimen_pose_ref.rec.mark_data_ready()

    def _NewEOATPositionInTransaction(self, orientation):
        eoat_pose_ref = snde.create_pose_channel_ndarray_ref(self.recdb, self.eoat_pose_chan, self.recdb.raw(), snde.recdb_path_join(self.eoat_model_tree, "meshed"))
        eoat_pose_ref.allocate_storage([1])
        # eoat_pose_rec = snde.pose_channel_recording.from_ndarray_recording(eoat_pose_ref.rec)
        eoat_pose_ref.data()[0] = orientation
        eoat_pose_ref.rec.mark_metadata_done()
        eoat_pose_ref.rec.mark_data_ready()
        try:

            self.get_logger().info(f"{Colors.MAGENTA}There are {self.counter}(s) specimen{Colors.RESET}", once=True)
            if self.counter:
                for i in range(self.counter):
                    vsop = snde.VectorOfStringOrientationPairs([snde.StringOrientationPair('/graphics/specimen_'+ str(i+1) +'/meshed',snde.snde_orientation3(self.specimen_position[i])), snde.StringOrientationPair('/graphics/probe/meshed',snde.snde_orientation3(self.probe_position))])
                    rec = snde.create_assembly_recording(self.recdb, self.positioner_chan[i], self.recdb.raw(), vsop)
                    rec.mark_data_and_metadata_ready()
        except Exception as e:
            self.get_logger().info(f"{Colors.RED}{e}{Colors.RESET}", once=True)

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


    def Move(self, msg, vel=0.1, acc=0.1):

        goal_msg = DgPython.Goal()
        
        goal_msg.velocity = vel
        goal_msg.acceleration = acc

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
            self.pose_pub.publish(goal_msg.pose_array)

        elif isinstance(msg, str):

            goal_msg.state = msg

        else:
            return 'Invalid variable type: ', msg.__class__

        self._action_client.wait_for_server()
        return self._action_client.send_goal(goal_msg)
    

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

        r = self.create_rate(30)

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


                transact = self.recdb.start_transaction()
                if self.eoat_mesh is not None:
                    self._NewEOATPositionInTransaction(np.array(((self.t.transform.translation.x,
                                                                  self.t.transform.translation.y,
                                                                  self.t.transform.translation.z,
                                                                  0),(
                                                                  self.arm_q[0],
                                                                  self.arm_q[1],
                                                                  self.arm_q[2],
                                                                  self.arm_q[3])),dtype=orient_dtype))
                try:
                    if self.positioner_chan[self.counter-1]:
                        # self.get_logger().info(f'Poistioner channel {self.counter} found', throttle_duration_sec=3)
                        if self.counter:
                            for i in range(self.counter):
                                self.join(specimenNum=i+1)
                except Exception as e:
                    self.get_logger().info(f"{e}", once=True)
                transact.end_transaction()
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
       
    def reconPoseArray(self, full_dome, x, y, rot=False, angle=12, length = 0.3, width = 0.15, x_pos = 0.655, y_pos = -0.1, z_pos = 0.946, x_offset=1, y_offset=1):
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
        self.recon_pose_pub.publish(recon_pose_array)
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


        return self.pose_pub.publish(self.pose_array)
    
    def addCross(self, pose):

        if isinstance(pose, Pose):
            self.center_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.center_array.poses = pose.poses

        else:
            rclpy.logging.get_logger("Add Pose").error("Wrong type")
            return
        
        return self.center_pub.publish(self.center_array)
    
    
    def addCoordinate(self, pose):

        if isinstance(pose, Pose):
            self.coordinate_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.coordinate_array.poses = pose.poses

        else:
            rclpy.logging.get_logger("Add Pose").error("Wrong type")
            return
        
        return self.coordinate_pub.publish(self.coordinate_array)
    
    def addDetection(self, pose):

        if isinstance(pose, Pose):
            self.yolo_array.poses.append(pose)
        
        elif isinstance(pose, PoseArray):
            self.yolo_array.poses = pose.poses

        else:
            rclpy.logging.get_logger("Add Pose").error("Wrong type")
            return
        
        return self.yolo_pub.publish(self.yolo_array)
    
    def pubPoseArray(self, pose_array):
        return self.pose_pub.publish(pose_array)

    def SpinnerThread(self):
        InitCompatibleThread(self, "_spinner")
    
        r = self.create_rate(30)

        while not self._quit:
            rclpy.spin(self)
            r.sleep()
            #time.sleep(0.5)

