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
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray, Pose, Quaternion, TransformStamped
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

    def sendGoal(self, interface, options):
        goal_msg = self.interfaces[interface].Goal()
        for key in options:
            setattr(goal_msg, key, options[key])
            print(f"Setting {key} to {options[key]}")
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def sendRequest(self, interface, options):
        req = self.interfaces[interface].Request()
        for key in options:
            setattr(req, key, options[key])
            print(f"Setting {key} to {options[key]}")
        future = self.client.call_async(req)
        while not future.done():
            time.sleep(0.01)
        return future.result()




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
        self.recon_pose_pub = self.create_publisher(PoseArray, 'recon_pose_array', 10)
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = to_frame_rel
        self.module_name = module_name
        self._params = {}
        self.from_frame_rel = from_frame_rel
        self.to_frame_rel = to_frame_rel
        self.t = Pose()
        self.desired_t = None
        self.recdb = recdb

        if eoat_mesh is not None:
            self.eoat_model_file = eoat_mesh
            self.graphman = snde.graphics_storage_manager("/graphics/", self.recdb.lowlevel_alloc, self.recdb.alignment_requirements,self.recdb.lockmgr, 1e-8, 2000000000)
            self.eoat_model_tree = '/graphics/probe/'
            self.eoat_pose_channel = '/probe_pose'
            transact = self.recdb.start_transaction()
            self.graphicsgroup = self.recdb.define_channel('/graphics/', 'main', self.recdb.raw(), False, self.graphman)
            self.eoat_recording = snde.x3d_load_geometry(self.recdb, self.graphman, self.eoat_model_file, 0, 'main', self.recdb.raw(), self.eoat_model_tree, None, ['reindex_vertices', 'reindex_tex_vertices'])
            self.eoat_pose_chan = self.recdb.define_channel(self.eoat_pose_channel, "main", self.recdb.raw())
            self._NewEOATPositionInTransaction(snde.snde_invalid_orientation3())
            transact.end_transaction()

            
        else:
            self.eoat_mesh = None


        self._action_client = PatchRclpyIssue1123(self, DgPython, 'dgpython')
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
    def specimenMesh(self,specimen_mesh=None):
        if specimen_mesh is not None:
            self.specimen_model_file = specimen_mesh
            self.specimen_model_tree = '/graphics/specimen/'
            self.specimen_pose_channel = '/specimen_pose'
            try:
                transact = self.recdb.start_transaction()
                self.specimen_recording = snde.x3d_load_geometry(self.recdb, self.graphman, self.specimen_model_file, 0, 'main', self.recdb.raw(), self.specimen_model_tree, None, ['reindex_vertices', 'reindex_tex_vertices'])
                self.specimen_pose_chan = self.recdb.define_channel(self.specimen_pose_channel, "main", self.recdb.raw())
                self._NewEOATPositionInTransaction(snde.snde_invalid_orientation3())
            finally:
                transact.end_transaction()

        else:
            self.specimen_mesh = None
    def _NewSpecimenPositionInTransaction(self, orientation):
        specimen_pose_ref = snde.create_pose_channel_ndarray_ref(self.recdb, self.specimen_pose_chan, self.recdb.raw(), snde.recdb_path_join(self.specimen_model_tree, "meshed"))
        specimen_pose_ref.allocate_storage([1])
        specimen_pose_rec = snde.pose_channel_recording.from_ndarray_recording(specimen_pose_ref.rec)
        specimen_pose_ref.data()[0] = orientation
        specimen_pose_ref.rec.mark_metadata_done()
        specimen_pose_ref.rec.mark_data_ready()

    def _NewEOATPositionInTransaction(self, orientation):
        eoat_pose_ref = snde.create_pose_channel_ndarray_ref(self.recdb, self.eoat_pose_chan, self.recdb.raw(), snde.recdb_path_join(self.eoat_model_tree, "meshed"))
        eoat_pose_ref.allocate_storage([1])
        eoat_pose_rec = snde.pose_channel_recording.from_ndarray_recording(eoat_pose_ref.rec)
        eoat_pose_ref.data()[0] = orientation
        eoat_pose_ref.rec.mark_metadata_done()
        eoat_pose_ref.rec.mark_data_ready()

    def Move(self, msg, vel, acc):

        goal_msg = DgPython.Goal()
        
        goal_msg.velocity = vel
        goal_msg.acceleration = acc

        if isinstance(msg, type(Pose())):
            goal_msg.destination = msg
        
        elif isinstance(msg, type(PoseArray())):
            # goal_msg.waypoints = msg
            goal_msg.pose_array = msg

        elif isinstance(msg, type(TransformStamped())):
            t = Pose()

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
                transform = self._tf_buffer.lookup_transform(
                        self.to_frame_rel,
                        self.from_frame_rel,
                        rclpy.time.Time())
                self.t.position = transform.transform.translation
                self.t.orientation = transform.transform.rotation
                transact = self.recdb.start_transaction()
                self._NewEOATPositionInTransaction(np.array(((transform.transform.translation.x,
                                                              transform.transform.translation.y,
                                                              transform.transform.translation.z,
                                                              0),(
                                                              transform.transform.rotation.x, 
                                                              transform.transform.rotation.y, 
                                                              transform.transform.rotation.z, 
                                                              transform.transform.rotation.w)),dtype=orient_dtype))
                transact.end_transaction()
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')

            r.sleep()

    
    # clientCreation(StartReconstruction, "start_reconstruction", TariReconstruction)

    def clientCreation(self, action, serviceType, serviceName):
        if action:
            cli = ActionClient(self, serviceType, serviceName)
        else:
            cli = self.create_client(serviceType, serviceName)
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{serviceName} service not available, waiting again...')

        ret = client(cli, serviceType, serviceName)        
        
        return ret
       
    def reconPoseArray(self, full_dome, slice=9, angle=12, length = 0.3, width = 0.15, x_pos = 0.655, y_pos = -0.1, z_pos = 0.946, x_offset=1, y_offset=1):
        recon_pose_array = PoseArray()
        recon_pose_array.header.frame_id = "world"
        x_unit = length/slice
        y_unit = width/slice
        if angle != 0:
            x_angle = math.pi/angle
            y_angle = math.pi/angle
        else:
            x_angle = 0
            y_angle = 0
        for e in range(slice):
            for i in range(slice):
                pose = Pose()
                pose.position.x = x_pos + x_unit * i
                pose.position.y = y_pos + y_unit * e
                pose.position.z = z_pos
                if full_dome:
                    q = transforms3d.euler.euler2quat(-y_angle*math.sin((e-((slice-1)/2))*math.pi/(slice-1)), x_angle*math.sin((i-((slice-x_offset)/2))*math.pi/((slice-1))), 0, 'sxyz')
                else:
                    q = transforms3d.euler.euler2quat(-y_angle*math.sin((e-((slice-1)/2))*math.pi/(slice-1)), x_angle*math.sin((i-(slice-1))*math.pi/((slice-1)*2)), 0, 'sxyz')
                pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
                recon_pose_array.poses.append(pose)
        rclpy.logging.get_logger("recon_pose_array").info(f"Pose array has {len(recon_pose_array.poses)} poses")
        self.recon_pose_pub.publish(recon_pose_array)
        return recon_pose_array
     
    def addPose(self, pose):
        self.pose_array.poses.append(pose)
        return self.pose_pub.publish(self.pose_array)

    def SpinnerThread(self):
        InitCompatibleThread(self, "_spinner")
    
        r = self.create_rate(30)

        while not self._quit:
            rclpy.spin(self)
            r.sleep()
            #time.sleep(0.5)

