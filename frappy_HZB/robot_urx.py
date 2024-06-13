import urx
import time
import logging

r = urx.URRobot("192.168.113.215", use_rt=True)#, urFirm=5.9)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    while 1:

        r.up()
        print('Program is running:',r.is_program_running)
        print('Robot is running:', r.is_running)
        print('Force:', r.get_force)
        r.down()
        print("##########")
    
        time.sleep(3)


    r.close()