
from frappy.datatypes import  EnumType, FloatRange, StringType, ArrayOf,StatusType

from frappy.core import Command, Parameter, Readable, HasIO,StructOf, IDLE, BUSY,  IntRange, Drivable

from frappy.errors import    ImpossibleError, IsBusyError

from frappy.lib.enum import Enum

from time import sleep


from frappy.modules import Attached

import re
import uuid


nsamples = 12 #check and make depend of mag slots



class Magazin:

    def __init__(self,nSamples):
        self.Mag = [''] * nSamples
          
        
    def insertSample(self,sampleID, samplePos):                   
        if self.Mag[samplePos-1]:
            raise Exception("Sample Pos already occupied")
        else:
            self.Mag[samplePos-1] = sampleID
                
            
    def removeSample(self,samplePos):            
        if self.Mag[samplePos-1] == '':
            raise Exception("No sample at Pos "+ str(samplePos-1))
        else: 
            sample = self.Mag[samplePos-1]
            self.Mag[samplePos-1] = '' 
            return sample       
    
    def get_sample(self,samplePos):
        return self.Mag[samplePos-1]     

    def mag2Arrayof(self): #needed?
        storage_arr = []
        for sample in  self.Mag:
            if sample:
                storage_arr.append(SampleToMeasure)
            
        return storage_arr   




class SampleToMeasure(HasIO,Drivable):
    
    Status = Enum(Drivable.Status,
                  DISABLED = StatusType.DISABLED,
                  PREPARING = StatusType.PREPARING,
                  HOLDING_SAMPLE = 101, 
                  MOUNTING=301,
                  UNMOUNTING = 302,
                  UNLOADING = 304,
                  MEASURING = 303,
                  PAUSED = 305,
                  UNKNOWN = 401,
                  STOPPED = 402,
                  LOCAL_CONTROL = 403,
                  LOCKED = 404 
                  )  #: status codes

    status = Parameter(datatype=StatusType(Status))  # override Readable.status

    attached_robot = Attached(mandatory = True)
    attached_storage = Attached(mandatory = True)
    

    
    value = Parameter("Active Sample currently in measurement position (blank == no sample)",
                      datatype=StringType,
                      readonly = True,
                      default = '') 

    target =  Parameter("Target Sample to be put in measurement position",
                      datatype=StringType,
                      default = '')
    
    
    def read_value(self):
        return self.value
       
    def read_target(self):
        return self.target
    
    def write_target(self,target): #next functions already integrated
           
        ### Mount to measurement position:
        if target != '':

            if self.value == target:
                raise ImpossibleError('Target and Value are the same. Target already mounted.')
            
            if self.value == '':
                self._mount(target)

            if self.value != '':
                self._unmount(self.value)
                self._mount(target)


        ### Unmount from measurement position:
        
        if target == '':

            if self.value == '':
                raise Exception("Value already empty.")
            
            if self.value != '':
                self._unmount(target)
            
        return target
    
    
    def read_status(self):   #noch nicht bearbeitet, wie wird dieser part gemacht? wie lesen wir aus ob der roboter am arbeiten ist prog_is_running?
        robo_stat = self.attached_robot.read_status()
        return robo_stat
    
           
    def _get_current_sample(self):
        return self.attached_storage.mag.get_sample(self.value)
    
    def _check_value_consistency(self):
        if not self._get_current_sample():
            self.status = ERROR, "inconsistent storage! expected sample object at pos:" + str(self.value)
            raise ImpossibleError('no sample stored at Pos: ' +str(self.value)+'! please, check sample objects stored in storage' )
        
    def _mount(self,target):
        """Mount Sample to Measurement Position"""  #not finished yet
        assert(target != 0)
        
        # check if sample is present in Storage
        if not self.attached_storage.mag.get_sample(target):
            raise ImpossibleError('no sample stored at pos: ' +str(target)+'! please, check sample objects stored in storage' )
        
        
        # check if robot is currently holding a Sample from Storage
        if self._holding_sample():
            raise ImpossibleError('Gripper is already holding sample' + str(self.value))       
     
        # Run Robot script to mount actual Sample        
        prog_name = 'messpos'+ str(target) + '.urp'
        
        assert(re.match(r'messpos\d+\.urp',prog_name) )
        
        self.attached_robot.write_target(prog_name)
 
        self.status = MOUNTING , "Mounting Sample: " + str(target)
        
        self.target = target
        
        # Robot successfully mounting the sample
        self.value = self.target
        
        self.read_status()
     

    def _unmount(self,target):
        """Unmount Sample to Measurement Position"""    #not finished yet
        
        assert(target == 0)
        
        # check if sample is present in Storage
        self._check_value_consistency()
        
        # check if robot is ready to mount sample
        if not self._holding_sample():
            raise ImpossibleError('Gripper is currently not holding a sample, cannot unmount')   
        
        
        # Run Robot script to unmount Sample        
        prog_name = 'messposin'+ str(self.value) + '.urp'
        
        assert(re.match(r'messposin\d+\.urp',prog_name) )
        
        self.attached_robot.write_target(prog_name)

        self.status = UNMOUNTING , "Unmounting Sample: " + str(self.value)
        
        self.target = target
        # Robot successfully unmounted the sample
        self.value = 0
        
        self.read_status()
               
    
    @Command
    def stop(self):
        """Stop execution of program"""
        self.attached_robot.stop()

    
    
class Storage(HasIO,Readable):
    
    Status = Enum(
        Drivable.Status,
        DISABLED = StatusType.DISABLED,
        PREPARING = StatusType.PREPARING,
        LOADING=303,
        UNLOADING = 304,
        PAUSED = 305,
        STOPPED = 402,
        LOCAL_CONTROL = 403,
        LOCKED = 404 
        )  #: status codes

    status = Parameter(datatype=StatusType(Status))  # override Readable.status
    
    attached_sample =  Attached(mandatory=True)
    
    attached_robot = Attached(mandatory = True)
    
    storage_size = Parameter("number of slots in storage",
                            datatype=IntRange(),
                            readonly = True,
                            default = 1,
                            visibility ="expert")
       
    
    value = Parameter("Sample IDs in storage",
                    datatype=ArrayOf(StringType(),0,nsamples),
                    readonly = True)
    
    
    
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.mag = Magazin(nsamples)
        
    def read_value(self):      
        return self.mag.mag2Arrayof()
    

    def read_status(self):   #noch nicht bearbeitet
        robo_stat = self.attached_robot.status
        
        # Robot Arm is Busy        
        if robo_stat[0] == BUSY:
            if re.match(r'in\d+\.urp',self.attached_robot.value):
                return  LOADING, "Loading sample"
            if re.match(r'out\d+\.urp',self.attached_robot.value):
                return UNLOADING , "Unloading sample"
            
            # Robot Running and No sample in Gripper
            return BUSY , "Robot is in use by another module"
        
        #TODO
        #if self.attached_sample._holding_sample():
        #    return BUSY , "Robot is in use by another module"
        
        
        return robo_stat

    
    @Command()
    def stop(self,group = 'control'):
        """Stop execution of program"""
        self.attached_robot.stop()
        return            
         
    
    @Command() #noch nicht bearbeitet
    def load(self,sample_pos,sample_id):
        """load sample into storage"""
     
        
        # check if robot is ready to load sample
        if self.attached_sample._holding_sample():            
            raise ImpossibleError('Gripper is already holding sample: ' + str(self.attached_sample.value))
        
                     
        # check if Sample position is already occupied
        if self.mag.get_sample(sample_pos) != None:
            raise ImpossibleError( "Sample pos "+ str(sample_pos) +" is already occupied")
            
        
        # Run Robot script to insert actual Sample        
        prog_name = 'in'+ str(sample_pos)+ '.urp'
        assert(re.match(r'in\d+\.urp',prog_name))
        
        self.attached_robot.write_target(prog_name)
        
        self.attached_robot.read_status()

        self.read_status()
        
        # Insert new Sample in Storage Array (it is assumed that the robot programm executed successfully)
        new_Sample = sample_id 
        try:
            self.mag.insertSample(new_Sample)
        except:
            raise ImpossibleError( "Sample pos "+ str(sample_pos) +" is already occupied")
            
        
        self.last_pos = sample_pos
        
        return
    
        
    @Command(IntRange(minval=1,maxval=nsamples),result=None)    #noch nicht bearbeitet
    def unload(self,sample_pos):
        """unload sample from storage"""
        
        # check if robot is ready to load sample
        if self.attached_sample._holding_sample() == True:
            raise ImpossibleError('Gripper is already holding sample' + str(self.attached_sample.value)+" try unloading via 'sample' module")
            

        # check if Sample position is already occupied
        if self.mag.get_sample(sample_pos) == None:
            raise ImpossibleError( "No sample present at pos: "+ str(sample_pos) )
        
        # Run Robot script to unload actual Sample        
        prog_name = 'out'+ str(sample_pos) +'.urp'
        assert(re.match(r'out\d+\.urp',prog_name))
        
        self.attached_robot.write_target(prog_name)
        
        self.attached_robot.read_status()

        self.read_status()

        try:
            self.mag.removeSample(sample_pos)
        except:
            raise ImpossibleError( "No sample at array pos " + str(sample_pos))
        
        self.last_pos = sample_pos
        
        return
            

MOUNTING         = SampleToMeasure.Status.MOUNTING
UNMOUNTING       = SampleToMeasure.Status.UNMOUNTING
MEASURING        = SampleToMeasure.Status.MEASURING
HOLDING_SAMPLE   = SampleToMeasure.Status.HOLDING_SAMPLE
PAUSED_SAMPLE    = SampleToMeasure.Status.PAUSED
STOPPED_SAMPLE   = SampleToMeasure.Status.STOPPED


LOADING          = Storage.Status.LOADING
UNLOADING        = Storage.Status.UNLOADING
PAUSED_STORAGE   = Storage.Status.PAUSED
STOPPED_STORAGE  = Storage.Status.STOPPED
LOCAL_CONTROL    = Storage.Status.LOCAL_CONTROL 
LOCKED           = Storage.Status.LOCKED
ERROR            = Storage.Status.ERROR

PREPARING  = Storage.Status.PREPARING
DISABLED   = Storage.Status.DISABLED