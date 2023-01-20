from pluto import Pluto
import time
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios


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

# pluto.setHostPort("192.168.10.7",23)

pluto.connect()

# # pluto.setRC({"pitch":1000,"roll":2000})

# # pluto.setThrottle(1500)

# pluto.ARM()

# # pluto.DISARM()
# time.sleep(1)

# pluto.TakeOff()

# print(f'Altitiude: {pluto.getAltitude()}')
# print(f'VarioMeter: {pluto.getVariometer()}')
# print(f'Acc: {pluto.getAcc()}')
# print(f'Gyro: {pluto.getGyro()}')
# print(f'Mag: {pluto.getMag()}')
# print(f'Roll: {pluto.getRoll()}')
# print(f'Pitch: {pluto.getPitch()}')
# print(f'Yaw: {pluto.getYaw()}')
# # print(pluto.getVariometer())
# # pluto.disconnect()
# pluto.disconnect()

HIGH = 1700
LOW = 1300
Neutral = 1500
throt=900
pluto.ping()

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)


    while True:
        # print(f'Roll:{pluto.getRoll()}   Pitch:{pluto.getPitch()}   Yaw:{pluto.getYaw()}')
        # print(f'Height:{pluto.getAltitude()}')
        # print(f'acc: {pluto.getAcc()}')
        key = getKey()
        # pluto.ping()
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
