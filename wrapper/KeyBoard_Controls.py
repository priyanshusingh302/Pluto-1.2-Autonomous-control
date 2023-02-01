from pluto import Pluto
import time
import sys, select, os
from threading import Thread, Event
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios


'''
The code is a Python script for controlling pluto drone.It uses a infinite loop to continuously listen for user inputs 
from the keyboard,which are stored in the variable "key". Depending on the input key value, the script sets various 
parameters for the drone such as roll, pitch, yaw, and throttle, and sends these parameters to the drone using the method 
"pluto.setRC()".The script also provides functions for different drone actions such as arming ("pluto.ARM()"), disarming 
("pluto.DISARM()"), taking off ("pluto.TakeOff()"), landing ("pluto.Land()"), turning on/off altitude hold 
("pluto.AltitudeHold_ON()"/"pluto.AltitudeHold_OFF()"), and turning on/off head-free mode 
("pluto.HeadFree_ON()"/"pluto.HeadFree_OFF()").If the input key is "b", the script breaks the loop and disconnects 
from the drone using the method "pluto.disconnect()". If the key value is not recognized, the script sets all control 
parameters to neutral and continues the loop.
'''

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



pluto = Pluto()

pluto.connect()


HIGH = 1700
LOW = 1300
Neutral = 1500
pluto.ping()

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    while True:
        key = getKey()
        roll=Neutral
        pitch=Neutral
        yaw=Neutral
        throttle=Neutral
        if len(key) > 0:
            print(key)

            if key == "m":
                pluto.ARM()
            elif key == "n":
                pluto.DISARM()

            elif key == "w":
                pitch=HIGH
            elif key == "s":
                pitch=LOW
            
            elif key == "d":
                roll=HIGH
            elif key == "a":
                roll=LOW
            
            elif key == "4":
                yaw=LOW
            elif key == "6":
                yaw=HIGH
            
            elif key=="8":
                throttle=HIGH
            elif key=="2":
                throttle=LOW

            elif key == "c":
                pluto.Land()
            elif key == "x":
                pluto.TakeOff()

            elif key=="y":
                pluto.AltitudeHold_OFF()
            elif key=="u":
                pluto.AltitudeHold_ON()

            elif key=="p":
                pluto.HeadFree_OFF()
            elif key=="o":
                pluto.HeadFree_ON()

            elif key == "b":
                print("[DISCONNECTED]")
                break

            else:
                pluto.setRC({"roll":Neutral,"pitch":Neutral,"yaw":Neutral,"throttle":Neutral})
                continue
    
        pluto.setRC({"roll":roll,"pitch":pitch,"yaw":yaw,"throttle":throttle})

    pluto.disconnect()
