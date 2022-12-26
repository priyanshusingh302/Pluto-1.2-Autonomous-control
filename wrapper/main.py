from pluto import Pluto


pluto = Pluto()

pluto.setHostPort("192.168.56.1",23)

pluto.connect()

# pluto.setRC({"pitch":1000,"roll":2000})

# pluto.setThrottle(1500)

# pluto.ARM()

# pluto.DISARM()


pluto.TakeOff()

print(f'Altitiude: {pluto.getAltitude()}')
print(f'VarioMeter: {pluto.getVariometer()}')
print(f'Acc: {pluto.getAcc()}')
print(f'Gyro: {pluto.getGyro()}')
print(f'Mag: {pluto.getMag()}')
print(f'Roll: {pluto.getRoll()}')
print(f'Pitch: {pluto.getPitch()}')
print(f'Yaw: {pluto.getYaw()}')
# print(pluto.getVariometer())
# pluto.disconnect()
pluto.temp_disconnect()

