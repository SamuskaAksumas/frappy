nsamples = 12

Node('sample_changer_myspot.HZB',  # a globally unique identification
     'Sample Changer for the myspot research station\n\nThis is the  SECoP (Sample Environment Communication Protocol) sample changer SEC-Node for the mysport research station.',  # describes the node
      'tcp://10770', #not finished yet
      implementor = 'Peter Wegnmann')  # you might choose any port number > 1024

Mod('io',  # the name of the module
    'frappy_HZB.robo_my.py',  # the class used for communication
    'TCP communication to robot Dashboard Server Interface',  # a description
    uri='tcp://192.168.113.215:29999')  # the serial connection
    #absatz und komma geändert
    
Mod('robot',
    'frappy_HZB.robo_my.py', #not finished yet
    'Module for controlling the Robotarm. It provides diagnostic information on the tool center point, joint information and general status of the robot',
    io='io',
    host='192.168.113.215',
    attached_sample = 'sample_to_measure',
    attached_storage = 'storage',
    group = 'UR_Robot',
    
    
    model = "none",
    serial = "none",
    ur_version = "none",
    
   
    pollinterval = 0.1,
    stop_State = {'stopped' : False,'interrupted_prog' : 'none'},
    pause_State = {'paused' : False,'interrupted_prog' : 'none'}

)

Mod('storage',
    'frappy_HZB.probenwechsler_my.Storage',
    'Samplestorage with slots for holding samples',
    io ='io',
    attached_sample = 'sample_to_measure',
    attached_robot = 'robot',
    group = 'sample_changer',
    storage_size = nsamples,
    pollinterval = 1
  
)

Mod('sample_to_measure',
    'frappy_HZB.probenwechsler_my.SampleToMeasure',
    'Sample to be measured on measuring spot',
    io ='io',
    attached_robot = 'robot',
    attached_storage = 'storage',
    group = 'sample_changer',
    pollinterval = 1
    )



