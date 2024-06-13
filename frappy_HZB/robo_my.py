from frappy.datatypes import BoolType, EnumType, FloatRange, StringType,  ArrayOf

from frappy.core import StatusType ,Command, Parameter,  HasIO, StringIO,StructOf,  IDLE, BUSY, ERROR, Drivable

from frappy.errors import IsErrorError, ReadFailedError, InternalError,   ImpossibleError, IsBusyError

from urx import URRobot

from frappy.lib.enum import Enum

from frappy.modules import Attached

import re




ROBOT_MODE_ENUM = {
    'NO_CONTROLLER'  :0,
    'DISCONNECTED'   :1,
    'CONFIRM_SAFETY' :2,
    'BOOTING'        :3,
    'POWER_OFF'      :4,
    'POWER_ON'       :5,
    'IDLE'           :6,
    'BACKDRIVE'      :7,
    'RUNNING'        :8          
}

SAFETYSTATUS = {
    'NORMAL' :0,
    'REDUCED' :1,
    'PROTECTIVE_STOP' :2,
    'RECOVERY' :3,
    'SAFEGUARD_STOP' :4,
    'SYSTEM_EMERGENCY_STOP' :5,
    'ROBOT_EMERGENCY_STOP' :6,
    'VIOLATION' :7,
    'FAULT' :8,
    'AUTOMATIC_MODE_SAFEGUARD_STOP' :9,
    'SYSTEM_THREE_POSITION_ENABLING_STOP' :10,
    'UNKNOWN':11

} 


RESET_PROG = 'reset.urp'

class RobotIO(StringIO):
    pass
    
    default_settings = {'port': 29999} # for dashboard client?
    pollinterval = 1
    wait_before = 0.05








class UR_Robot(HasIO,Drivable, URRobot):

    def __init__(self, robot_ip, use_rt=True):
        
        self.bot = URRobot(robot_ip)
       
    
    attached_sample =  Attached(mandatory=True)
    
    attached_storage = Attached(mandatory=True)
    
    Status = Enum(
        Drivable.Status,
        Drivable.Status,
        DISABLED = StatusType.DISABLED,
        PREPARING = StatusType.PREPARING,
        PAUSED = 305,
        UNKNOWN = StatusType.UNKNOWN,
        STOPPED = 402,
        STANDBY = StatusType.STANDBY,
        LOCAL_CONTROL = 403,
        LOCKED = 404        
                
        )  #: status codes

    status = Parameter(datatype=StatusType(Status))  # override Readable.status

    
    
    coords = Parameter("Coordinate Position",
                       datatype=ArrayOf(FloatRange()),
                       default = 0,
                       readonly = True)
    
    robot_ip = Parameter("IP of the robot",
                       datatype=StringType(),
                       default = '0.0.0.0',
                       readonly = True)

    joint_temperature = Parameter("Temperatures in Robot (Joints?)",
                       datatype=ArrayOf(FloatRange()),
                       default = 'none',
                       readonly = True)
    
    joint_position = Parameter("Joint angels",
                      datatype=ArrayOf(FloatRange()),
                      default = "none",                
                      readonly = True)
    
    model = Parameter("Model name of the robot",
                      datatype=StringType(),
                      default = "none",                
                      readonly = True,
                      group = "Robot Info")
    
    serial = Parameter("Serial number of connected robot",
                       datatype=StringType(),
                       default = "none",
                       readonly = True,
                       group = "Robot Info")
    
    ur_version = Parameter("Version number of the UR software installed on the robot",
                           datatype=StringType(),
                           default = "none",
                           readonly = True,
                           group = "Robot Info",
                           visibility = 'expert')
    
    robotmode = Parameter("Current mode of robot",
                          datatype=EnumType("Robot Mode",ROBOT_MODE_ENUM),
                          default = "DISCONNECTED",
                          readonly = True,
                          group = "Status Info")
    
    powerstate = Parameter("Powerstate of robot",
                           datatype=EnumType("Pstate",POWER_OFF= None,POWER_ON = None ),
                           default = "POWER_OFF" ,
                           readonly = False,
                           group = "Status Info")
    
    safetystatus = Parameter("Safetystatus: Specifying if a given Safeguard Stop was caused by the permanent safeguard I/O stop,a configurable I/O automatic mode safeguard stop or a configurable I/O three position enabling device stop.",
                             datatype=EnumType(SAFETYSTATUS),
                             default = "NORMAL",
                             readonly = True,
                             group = 'Status Info')

    is_in_remote_control = Parameter("Control status of robot arm",
                                     datatype=BoolType,
                                     readonly = True,
                                     default = False,
                                     group = 'Status Info')
    
    stop_State = Parameter("Robot state when stop was pressed",
                           datatype=StructOf(stopped = BoolType(),
                                             interrupted_prog = StringType(),
                                             ),
                           visibility = 'expert',
                           default = {'stopped':False,'interrupted_prog':'none'}
                           )
    
    pause_State = Parameter("Robot state when pause was pressed",
                           datatype=StructOf(paused = BoolType(),
                                             interrupted_prog = StringType(),
                                             ),
                           visibility = 'expert',
                           default = {'paused':False,'interrupted_prog':'none'})
    
    
    def doPoll(self):
        self.read_status()
        self.read_coords()


    def read_coords(self):
        return self.bot.getl()
    

    def read_joint_position(self):
        return self.bot.getj()
    

    def read_model(self):
        return str(self.communicate('get robot model'))


    def read_serial(self):
        return str(self.communicate('get serial number'))
    

    def read_ur_version(self):
        return str(self.communicate('version'))
    

    def read_robotmode(self):
        robo_mode =  str(self.communicate('robotmode')).removeprefix('Robotmode: ')
    
        if robo_mode in ROBOT_MODE_ENUM:
            return robo_mode

        raise ReadFailedError("Unknown robot mode:" + robo_mode)
    

    def read_powerstate(self):
        self.read_robotmode()
        if self.robotmode.value > 4:
            return 'POWER_ON' 
        else:
            return 'POWER_OFF'


    def write_powerstate(self,powerstate):
        p_str = powerstate.name
        
        self.communicate(POWER_STATE.get(p_str,None))
        
        if powerstate == 'POWER_ON':
            self.communicate('brake release')
        
        
        self.powerstate = self.read_powerstate()
        
        return powerstate.name
    

    def read_status(self):
    
        if self.bot.is_running:
            if self.bot.is_program_running:
                return BUSY, 'Robot is running a program'
            else:
                return IDLE, 'Robot on without any prorgam running'
        else:
            return STOPPED, 'Robot not running at all'


    def _program_running(self): 
        if self.bot.is_program_running():
            return True
        else:
            return False
        

    @Command(group ='control')
    def stop(self):
        """Stop execution of program"""
        
        self.bot.stop()
    
    @Command(group ='control')
    def play(self):
        """Start/continue execution of program"""
        
        if self.status == BUSY:
            raise ImpossibleError('Robot currently busy')
        if self.status == STOPPED:
            raise ImpossibleError('Robot stopped')
        if self.status == IDLE:
            self.bot.down()
            self.bot.up()       #do something
            
            
    
  
PAUSED           = UR_Robot.Status.PAUSED
STOPPED          = UR_Robot.Status.STOPPED
UNKNOWN          = UR_Robot.Status.UNKNOWN
PREPARING        = UR_Robot.Status.PREPARING
DISABLED         = UR_Robot.Status.DISABLED
STANDBY          = UR_Robot.Status.STANDBY 
LOCAL_CONTROL    = UR_Robot.Status.LOCAL_CONTROL 
LOCKED           = UR_Robot.Status.LOCKED
ERROR            = UR_Robot.Status.ERROR

ROBOT_MODE_STATUS = {
    'NO_CONTROLLER' :(ERROR,'NO_CONTROLLER'),
    'DISCONNECTED' :(DISABLED,'DISCONNECTED'),
    'CONFIRM_SAFETY' :(DISABLED,'CONFIRM_SAFETY'),
    'BOOTING' :(PREPARING,'BOOTING'),
    'POWER_OFF' :(DISABLED,'POWER_OFF'),
    'POWER_ON' :(STANDBY,'POWER_ON'),
    'IDLE' :(IDLE,'IDLE'),
    'BACKDRIVE' :(PREPARING,'BACKDRIVE'),
    'RUNNING' :(IDLE,'IDLE'),
}



POWER_STATE = {
    'POWER_ON'  : 'power on',
    'POWER_OFF' : 'power off'
}


if __name__ == '__main__':
    print(UR_Robot('198.162.113.215').read_coords())