from smbus import SMBus
import time
 
addr = 0x08 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
time.sleep(5) #wait here to avoid 121 IO Error

landed = False

def StringToBytes(val):
    retVal = []
    for c in val:
        retVal.append(ord(c))
    return retVal

while True:
    while not landed:
        comm_Arduino = bus.read_i2c_block_data(addr, 0x00, 1)
        time.sleep(1)
        print (comm_Arduino[0])
        if comm_Arduino[0] == 1:
            landed = True
    msg = "A1B2C3D4E5F6"
    BytesToSend = StringToBytes(msg)
    bus.write_i2c_block_data(addr, 0x00, BytesToSend)
