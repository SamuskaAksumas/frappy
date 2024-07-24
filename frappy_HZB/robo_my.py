from frappy.datatypes import BoolType, EnumType, FloatRange, StringType,  ArrayOf

from frappy.core import StatusType ,Command, Parameter,  HasIO, StringIO,StructOf,  IDLE, BUSY, ERROR, Drivable

from frappy.errors import IsErrorError, ReadFailedError, InternalError,   ImpossibleError, IsBusyError

from urx import URRobot,Gripper

from frappy.lib.enum import Enum

from frappy.modules import Attached





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
                       datatype=ArrayOf(FloatRange(),minlen=3,maxlen=3), #noch unbekannt
                       default = [0,0,0],
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
    
    speed = Parameter("Gripper Speed",
                           datatype=FloatRange(),
                           default = 50)
    
    pos = Parameter("Gripper Position",
                           datatype=FloatRange(),
                           default = 50)
    
    force = Parameter("Gripper Force",
                           datatype=FloatRange(),
                           default = 10)
    
    
    
    def initModule(self):
        try:
            self.bot = URRobot(self.robot_ip)
        except TimeoutError:
            self.bot = None

        try:
            self.gripper = Gripper(self.robot_ip)
        except:
            self.gripper = None

        return super().initModule() 


    def doPoll(self):
        self.read_status()
        self.read_coords()
        self.read_joint_positions()
        self.read_robotmode()
        self.read_is_in_remote_control()



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
        self.bot.down()
    
    @Command(group ='control')
    def up(self):
        """Start/continue execution of program"""
        
        # if self.status == BUSY:
        #     raise ImpossibleError('Robot currently busy')
        # if self.status == STOPPED:
        #     raise ImpossibleError('Robot stopped')
        # if self.status == IDLE:
        self.bot.up()       #do something

    @Command(group ='control')
    def home(self):
        """Start/continue execution of program"""
        
        self.bot.movel([-0.136,-0.267,-0.091,0.001,-3.166,-0.040], 0.1, 0.1)       #do something
    
    @Command(group ='control')
    def activate_gripper(self):
        """Stop execution of program"""
        self.gripper.activate()
    
    @Command(group ='control')
    def connect_gripper(self):
        """Stop execution of program"""
        self.gripper.connect
    
    @Command(group ='control')
    def disconnect_gripper(self):
        """Stop execution of program"""
        self.gripper.disconnect()
    
    @Command(group ='control')
    async def go_to_pos_gripper(self):#, pos, spd, frc):
        """Stop execution of program"""
        pos = self.pos
        spd = self.speed
        frc = self.force
        await print('pos, speed, force',pos, spd, frc)
        await self.gripper.move_and_wait_for_pos(pos,spd, frc)


            
            
    
  
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

