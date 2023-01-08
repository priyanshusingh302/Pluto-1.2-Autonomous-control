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

# # pluto.setHostPort("127.0.1.1",23)

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

HIGH = 2100
LOW = 900
Neutral = 1500

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)


    while True:

        pluto.ARM()
        key = getKey()
        if len(key) > 0:
            if key == "m":
                pass
            elif key == "n":
                pluto.DISARM()

            elif key == "w":
                pluto.setPitch(HIGH)
            elif key == "s":
                pluto.setPitch(LOW)
            

            elif key == "d":
                pluto.setRoll(HIGH)
            elif key == "a":
                pluto.setRoll(LOW)
            

            elif key == "4":
                pluto.setYaw(HIGH)
            elif key == "6":
                pluto.setYaw(LOW)
            
            elif key=="8":
                pluto.setThrottle(HIGH)
            elif key=="2":
                pluto.setThrottle(LOW)

            elif key == "c":
                pluto.Land()

            elif key == "x":
                pluto.TakeOff()

            elif key=="y":
                pluto.AltitudeHold_OFF()
            elif key=="u":
                pluto.AltitudeHold_ON()

            elif key == "b":
                break
            print(key)
                
        # else:
        #     pluto.setPitch(Neutral)
        #     pluto.setRoll(Neutral)
        #     pluto.setThrottle(Neutral)
        #     pluto.setYaw(Neutral)

    pluto.disconnect()
