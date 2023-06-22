import sys
import nidaqmx
import spatialnde2 as snde
import numpy as np
import time
import pint
import copy
import typing
from threading import Thread
import traceback
from dataguzzler_python.dgpy import Module as dgpy_Module
from dataguzzler_python.dgpy import InitCompatibleThread
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dgpython_ros2_actioninterface.action import DgPython
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from threading import RLock
from typing import Any
from rclpy.task import Future

ur = pint.get_application_registry()


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

    def __init__(self, module_name, from_frame_rel, to_frame_rel, **kwargs):
        super().__init__('dataguzzler_frame_publisher')
        self.module_name = module_name
        self._params = {}
        self.from_frame_rel = from_frame_rel
        self.to_frame_rel = to_frame_rel
        self.t = None
        self.desired_t = None
        self._action_client = PatchRclpyIssue1123(self, DgPython, 'dgpython') 
        #self.broadcaster = self.create_publisher(Pose, "/dataguzzler_frame_publisher", 100)

        for param in self._paramdict:
            self._params[param] = self._paramdict[param][1]
        
        for param in kwargs:
            self._params[param] = self._paramdict[param][2](kwargs[param])

        # For now, we'll run our own thread to query ROS -- not sure how threading works in the create_subscription method
        self.RestartAcquisition()

    def Move(self, msg):
        t = Pose()

        t.position.x = msg.transform.translation.x
        t.position.y = msg.transform.translation.y
        t.position.z = msg.transform.translation.z

        t.orientation.x = msg.transform.rotation.x
        t.orientation.y = msg.transform.rotation.y
        t.orientation.z = msg.transform.rotation.z
        t.orientation.w = msg.transform.rotation.w

        self.desired_t = t

        goal_msg = DgPython.Goal()
        goal_msg.destination = self.desired_t

        self._action_client.wait_for_server()

        return self._action_client.send_goal(goal_msg)

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
            while self._thread.isAlive():
                time.sleep(0.01)
                pass
        return

    def WaitPauseAcquisition(self):
        if self._thread is not None and self._thread.isAlive():
            self._pause = True
            while self._paused == False:
                time.sleep(0.01)
                pass
        return

    def WaitUnPauseAcquisition(self):
        if self._thread is not None and self._thread.isAlive() and self._paused == True:
            self._pause = False
            while self._paused == True:
                time.sleep(0.01)
                pass
        return

    def RestartAcquisition(self):
        if self._thread is not None:
            self._quit = True
            while self._thread.isAlive():
                time.sleep(0.01)
                pass  
        
        self._quit = False
        self._pause = False
        self._paused = False
        self._thread = Thread(target=self.AcquisitionThread, daemon=True)
        self._thread.start()
        self._spinner = Thread(target=self.SpinnerThread, daemon=True)
        self._spinner.start()
    
    def AcquisitionThread(self):
        InitCompatibleThread(self, "_thread")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        while not self._quit:
            if self._pause == True:
                self._paused = True
                while self._pause == True:
                    time.sleep(0.01)
                    pass
                self._paused = False

            try:
                self.t = self._tf_buffer.lookup_transform(
                        self.to_frame_rel,
                        self.from_frame_rel,
                        rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')

    def SpinnerThread(self):
        InitCompatibleThread(self, "_spinner")

        while not self._quit:
            rclpy.spin(self)
            #time.sleep(0.5)

