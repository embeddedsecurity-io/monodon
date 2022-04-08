from pyftdi.i2c import I2cController

i2c = I2cController()
i2c.configure('ftdi://ftdi:2232:TG110505/2')

with open('firmware.bin', 'wb') as f:
    for i in range(0, (128*1000) // 16):
        slave = i2c.get_port(0x21)
        data = slave.read(readlen=16, start=True)
        f.write(data)
        print("Downloaded 16 byte")