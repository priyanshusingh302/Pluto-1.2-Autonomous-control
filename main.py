from pluto import Pluto


pluto = Pluto()

pluto.setHostPort("192.168.56.1",23)

pluto.connect()

pluto.setRC({"pitch":1000,"roll":2000})

# pluto.setThrottle(1500)

# pluto.ARM()

# pluto.DISARM()


pluto.TakeOff()

print(pluto.getAltitude())

pluto.TakeOff()

print(pluto.getAltitude())

# print(pluto.getVariometer())

pluto.temp_disconnect()
