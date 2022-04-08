import pyftdi.serialext
import struct
port = pyftdi.serialext.serial4url('ftdi://ftdi:2232:TG110505/1', baudrate=9600)

with open('firmware.bin', 'wb') as f:
    for i in range(0, (128*1000) // 16):
        len = port.write(struct.pack('<IIII', 0x2c,0x00, i*16 , 16))
        data = port.read(16)
        f.write(data)