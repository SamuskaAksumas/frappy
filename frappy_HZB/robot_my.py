from frappy.datatypes import  EnumType, FloatRange, StringType, ArrayOf,StatusType
import urx
from frappy.core import Command, Parameter, Readable, HasIO,StructOf, IDLE, BUSY,  IntRange, Drivable

from frappy.errors import    ImpossibleError, IsBusyError

from frappy.lib.enum import Enum


from frappy.modules import Attached

import re
import uuid



class robot(Readable):

    def __init__(self, name, logger, cfgdict, srv, ip):
        super().__init__(name, logger, cfgdict, srv)
        self.robot = urx.Robot(ip, use_rt=True, urFirm=5.9)

    def read_status(self):
        if self.robot.
        

    def activate_rt_monitor(self):
        self.robot.get_realtime_monitor()

    
    def get_joint_temperature(self):
        return self.robot.get_joint_temperature()
    
    def get_joint_voltage(self):
        return self.robot.get_joint_voltage()
    
    def get_main_voltage(self):
        return self.robot.get_main_voltage()
    
    def get_robot_voltage(self):
        return self.robot.get_robot_voltage()
    
    def get_robot_current(self):
        return self.robot.get_robot_current()
    

    @Command
    def get_data(self):
        self.activate_rt_monitor()
        return [self.get_joint_temperature,
                self.get_joint_voltage,
                self.get_main_voltage,
                self.get_robot_current,
                self.get_robot_voltage]
    
    






