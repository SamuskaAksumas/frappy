from frappy.datatypes import BoolType, EnumType, FloatRange, StringType,  ArrayOf, IntRange

from frappy.core import StatusType ,Command, Parameter,  HasIO, StringIO,StructOf,  IDLE, BUSY, ERROR, Drivable

from frappy.errors import IsErrorError, ReadFailedError, InternalError,   ImpossibleError, IsBusyError

from urx import URRobot

from urx.gripper_davinci import Gripper

from frappy.lib.enum import Enum

from frappy.modules import Attached

import asyncio

import numpy as np

from scipy.spatial.transform import Rotation as R

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








class Robot(HasIO,Drivable):

        

    # Der wert der robot IP kann in der cfg Datei innerhalb des robot modules gesetzt werden (z.b.: robot_ip='192.168.1.2')
    robot_ip = Parameter("IP Address of the robot arm",
    	datatype=StringType(),
    	readonly = True,
    	export = False) # export= false --> der parameter ist nur intern und wird nicht mit der SEC node gepublished 
    
    attached_sample =  Attached(mandatory=True)
    
    attached_storage = Attached(mandatory=True)
    
    Status = Enum(
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

    
    value = Parameter("Currently loaded program",
                       datatype=StringType(),
                       default = '<unknown>.urp',
                       readonly = True)

    target = Parameter("Program that is to be executed",
                       datatype=StringType(),
                       default = 'none',
                       readonly = False)
    
    loaded_prog = Parameter("Program that is currently loaded",
                            datatype= StringType(),
                            default = "<unknown>.urp",
                            readonly = True,
                            visibility = 'expert')

    coords = Parameter("Coordinate Position",
                       datatype=ArrayOf(FloatRange(),minlen=6,maxlen=6), # Hier mussder datentyp genau bestimmt sein am besten mit minlen == maxlen  
                       default = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # der default value muss dem datentyp ensprechen 
                       readonly = True)
    
    #TODO anpassen minlen maxlen und default value anpassen
    joint_temperature = Parameter("Temperatures in Robot (Joints?)",
                       datatype=ArrayOf(FloatRange(),minlen=6,maxlen=6), #noch unbekannt
                       default = [0,0,0,0,0,0],
                       readonly = True)
    
    joint_positions = Parameter("Joint angels",
                      datatype=ArrayOf(FloatRange(),minlen=6,maxlen=6),
                      default = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],                
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
                                     datatype=BoolType(),
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
    
    obj_grabbed = Parameter("Boolean if object has been detected in gripper",
                           datatype = IntRange(0,4),
                           visibility = 'expert', #issue:closing raises KeyError: True when grabbing (closing gripper)
                           default = 0,
                           readonly = True)


    def initModule(self):
        try:
            self.bot = URRobot(self.robot_ip,use_rt=True,urFirm=5.9)
            self.gripper = Gripper(self.robot_ip, 63352)
        except TimeoutError:
            self.bot = None    

        return super().initModule() 


    def doPoll(self):
        self.read_status()
        self.read_coords()
        self.read_joint_positions()
        self.read_robotmode()
        self.read_is_in_remote_control()
        self.read_joint_temperature()


    def read_joint_temperature(self):
        if self.bot == None:
            return [0,0,0,0,0,0]

        return self.bot.get_joint_temperature()



    def read_coords(self):
        if self.bot == None:
            return [0,0,0,0,0,0]
        
        return self.bot.getl()
    

    def read_joint_positions(self):
        if self.bot == None:
            return [0,0,0,0,0,0]
        
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
    

    def read_is_in_remote_control(self):
        remote_control =  str(self.communicate('is in remote control'))
        
        if remote_control == 'true':
            return True
        return False
    

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
    
        if self.bot is None:
            return DISABLED, "not connected"
        if self.bot.is_running:
            if self.bot.is_program_running():
                return BUSY, 'Robot is running a program'
            else:
                return IDLE, 'Robot on without any prorgam running'
        else:
            return STOPPED, 'Robot not running at all'
    
    def write_target(self,target):
        # Is Robot in remote control Mode?        
        
        if self.safetystatus > SAFETYSTATUS['REDUCED']:
            raise IsErrorError('Robots is locked due to a safety related problem (' + str(self.safetystatus.name) + ") Please refer to instructions on the controller tablet or try 'clear_error' command.")
            
        
        if not self.is_in_remote_control:
            raise ImpossibleError('Robot arm is in local control mode, please switch to remote control mode on the Robot controller tablet')
        
        # Is the Robot in a stpped state?
        if self.stop_State['stopped']:
            raise IsErrorError('cannot run program when execution was Stopped, please reset and check for consistency')
        
        if self.pause_State['paused']:
            raise IsBusyError("continue loaded program before executing "+ target)
        
        if self.status[0] == BUSY or self.status[0] == PREPARING:
            raise IsBusyError('Robot is already executing another program')
        
        if self.status[0] >= 400 and self.status[0] != STOPPED:
            raise IsErrorError("Robot is in an error state. program '"+target+ "' cannot be exectuted")
        
        load_reply = str(self.communicate(f'load {target}'))
              
        
        if re.match(r'Loading program: .*%s' % target,load_reply):
            self._run_loaded_program()
            self.value = target
            return target
           
        elif re.match(r'File not found: .*%s' % target,load_reply):
            raise InternalError('Program not found: '+target)
        
        elif re.match(r'Error while loading program: .*%s' % target,load_reply):
            raise InternalError('write_target ERROR while loading program: '+ target)
            
        else:
            self.status = ERROR, 'unknown answer: '+ load_reply 
            raise InternalError('unknown answer: '+load_reply) 
        
    
    def _run_loaded_program(self):
        play_reply  = str(self.communicate('play'))
        

        
        if play_reply == 'Starting program':
            # Reset paused state
            self.pause_State = {'paused'  : False, 'interrupted_prog' : self.loaded_prog}
            self.stop_State  = {'stopped' : False, 'interrupted_prog' : self.loaded_prog}
            self.status = BUSY, "Starting program"
        else:
            raise InternalError("Failed to execute: play")
    

    @Command(group ='control')
    def init(self):
        """Stop execution of program"""
        
        self.write_target('init')
        

    def convert_local2global(origin_point,local_point, degrees=False):
        '''
        This funtions converts points in a local coordinate system (x,y,z)
        into a point given in the global coordinatesystem (x,y,z,xr,yr,zr)
        Input Angles have to be given in radians.
        Input:
        origin_point: Origin of local coordinatesystem as: x,y,z,xr,yr,zr
        local_point: local point given as: x,y,z
        optional:
        degrees: Output angles in degree (default: False, means in radians)
        '''

        # Function to clean rounding errors
        def apply_threshold(matrix, threshold=1e-12):
            matrix[np.abs(matrix) < threshold] = 0
            return matrix

        # Create Rotation Matrizes for each Rotationaxis
        R_x =np.array([
        [1, 0, 0],
        [0, np.cos(origin_point[3]), -np.sin(origin_point[3])],
        [0, np.sin(origin_point[3]), np.cos(origin_point[3])]
        ])

        R_y = np.array([
        [np.cos(origin_point[4]), 0, np.sin(origin_point[4])],
        [0, 1, 0],
        [-np.sin(origin_point[4]), 0, np.cos(origin_point[4])]
        ])

        R_z = np.array([
        [np.cos(origin_point[5]), -np.sin(origin_point[5]), 0],
        [np.sin(origin_point[5]), np.cos(origin_point[5]), 0],
        [0, 0, 1]
        ])

        # Clean rotations from rounding errors
        R_x = apply_threshold(R_x)
        R_y = apply_threshold(R_y)
        R_z = apply_threshold(R_z)

        # Combined Rotation
        R_combined = R_z @ R_y @ R_x

        # vector for translations
        d = np.array([origin_point[0], origin_point[1], origin_point[2]])

        # Homogene transformationmatrix
        T = np.eye(4)
        T[:3, :3] = R_combined
        T[:3, 3] = d

         # Point in homogene coordinates
        point_hom = np.append(local_point, 1)

        # Transformed point in global coordinatesystem
        point_global_hom = T @ point_hom
        global_position = point_global_hom[:3]

        # get global rotations into euler angles
        rotation = R.from_matrix(R_combined)
        global_orientation = rotation.as_euler('xyz', degrees=degrees)
        
        # return global position in x,y,z,xr,yr,zr
        return np.concatenate((global_position, global_orientation))


    @Command(group ='control')
    def stop(self):
        """Stop execution of program"""
        
        self.bot.stop()
    
    @Command(group ='control')
    def play(self):
        """Start/continue execution of program"""
        
        # if self.status == BUSY:
        #     raise ImpossibleError('Robot currently busy')
        # if self.status == STOPPED:
        #     raise ImpossibleError('Robot stopped')
        # if self.status == IDLE:
        self.bot.down()
        self.bot.up()       #do something

    @Command(group ='control')
    def down(self):
        """Start/continue execution of program"""
        
        # if self.status == BUSY:
        #     raise ImpossibleError('Robot currently busy')
        # if self.status == STOPPED:
        #     raise ImpossibleError('Robot stopped')
        # if self.status == IDLE:
        #self.bot.down()
        self.bot.movel((0, 0, -0.05, 0, 0, 0), 0.5, 0.5, relative=True)
    
    @Command(group ='control')
    def up(self):
        """Start/continue execution of program"""
        
        # if self.status == BUSY:
        #     raise ImpossibleError('Robot currently busy')
        # if self.status == STOPPED:
        #     raise ImpossibleError('Robot stopped')
        # if self.status == IDLE:
        #self.bot.up()       #do something
        self.bot.movel((0, 0, 0.05, 0, 0, 0), 0.5, 0.5, relative=True)

    @Command(group ='control')
    def move_in_local_coord(self):
        """Start/continue execution of program"""
        a=0.4
        v=0.4
        local_coord = self.bot.getl()
        print(local_coord)
        #x
        tpose = self.local_to_global([0.05,0,0,0,0,0],local_coord)
        print(tpose)
        self.bot.movel(tpose,a,v)
        tpose = self.local_to_global([0,0,0,0,0,0],local_coord)
        print(tpose)
        self.bot.movel(tpose,a,v)
        #y
        tpose = self.local_to_global([0,0.05,0,0,0,0],local_coord)
        self.bot.movel(tpose,a,v)
        tpose = self.local_to_global([0,0,0,0,0,0],local_coord)
        self.bot.movel(tpose,a,v)
        #z
        self.bot.movel(tpose,a,v)
        tpose = self.local_to_global([0,0,0.05,0,0,0],local_coord)
        self.bot.movel(tpose,a,v)
        tpose = self.local_to_global([0,0,0,0,0,0],local_coord)
        self.bot.movel(tpose,a,v)


    @Command(group ='control')
    def home(self):
        """Start/continue execution of program"""
        
        self.bot.movel([-0.136,-0.267,-0.091,0.001,-3.166,-0.040], 0.1, 0.1)       #do something

    @Command(group ='control') # connect and disconnect has to be included, otherwise error pops up because socket connection cannot be seperated
    def open_gripper(self):
        """Start/continue execution of program"""
        async def run():
            await self.gripper.connect()
            await self.gripper.move_and_wait_for_pos(1, 100, 100)
            self.obj_grabbed =  await self.gripper._get_var('OBJ') #muss noch überprüft werden welche zahl grabbed bedeutet
            await self.gripper.disconnect()
            # print(await self.gripper.get_current_position())
            # print('detected:', await self.gripper._get_var('OBJ'))
        asyncio.run(run())

    @Command(group ='control')
    def close_gripper(self):
        """Start/continue execution of program"""
        async def run():
            await self.gripper.connect()
            self.obj_grabbed =  await self.gripper._get_var('OBJ') #muss noch überprüft werden welche zahl grabbed bedeutet
            await self.gripper.move_and_wait_for_pos(255, 100, 100)
            await self.gripper.disconnect()
        asyncio.run(run())

    @Command(group ='control')
    def activate_and_calibrate_gripper(self):
        """Start/continue execution of program"""
        async def run():
            await self.gripper.connect()
            await self.gripper.activate()
            await self.gripper.disconnect()
        asyncio.run(run())
            
    
  
PAUSED           = Robot.Status.PAUSED
STOPPED          = Robot.Status.STOPPED
UNKNOWN          = Robot.Status.UNKNOWN
PREPARING        = Robot.Status.PREPARING
DISABLED         = Robot.Status.DISABLED
STANDBY          = Robot.Status.STANDBY 
LOCAL_CONTROL    = Robot.Status.LOCAL_CONTROL 
LOCKED           = Robot.Status.LOCKED
ERROR            = Robot.Status.ERROR

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

