# I2C with the Arduino as the master device and the OpenMV Cam as the slave.
#
# Please wire up your OpenMV Cam to your Arduino like this:
#
# OpenMV Cam Master I2C Data  (P5) - Teensy SDA (18)
# OpenMV Cam Master I2C Clock (P4) - Teensy SCL (19)
# OpenMV Cam Ground                - Arduino Ground

import pyb, ustruct, sensor, image, time, machine, pyb

# Use "ustruct" to build data packets to send.
# "<" puts the data in the struct in little endian order.
# "%ds" puts a string in the data stream. E.g. "13s" for "Hello World!\n" (13 chars).
# See https://docs.python.org/3/library/struct.html

# READ ME!!!
#
# Please understand that when your OpenMV Cam is not the I2C master it may miss responding to
# sending data as a I2C slave no matter if you call "i2c.send()" in an interupt callback or in the
# main loop below. When this happens the Arduino will get a NAK and have to try reading from the
# OpenMV Cam again. Note that both the Arduino and OpenMV Cam I2C drivers are not good at getting
# unstuck after encountering any I2C errors. On the OpenMV Cam and Arduino you can recover by
# de-initing and then re-initing the I2C peripherals.

# The hardware I2C bus for your OpenMV Cam is always I2C bus 2.
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit() # Fully reset I2C device...
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
print("Waiting for Teensy...")

# Note that for sync up to work correctly the OpenMV Cam must be running this script before the
# Arduino starts to poll the OpenMV Cam for data. Otherwise the I2C byte framing gets messed up,
# and etc. So, keep the Arduino in reset until the OpenMV Cam is "Waiting for Arduino...".

data = bytearray(11)  # create a buffer

previousTask = False
done = False

#Clock Timer
clock = time.clock()

#Initialize Timer
start = pyb.millis()

#Initialize Camera Settings
numImg = 0

sensor.reset() #Clear the sensor
sensor.set_pixformat(sensor.RGB565) #Color image (RGB)
sensor.set_framesize(sensor.QVGA) #Framesize
sensor.skip_frames(time = 1000)

img = sensor.snapshot()

while(not done):
    try:
        bus.recv(data, timeout=10000)       # receive 20 bytes, writing them into data
        fullTask = data.decode("utf-8")
        print(fullTask)

        for task in fullTask:
            if task == 'C':
                if not previousTask:
                    numImg = numImg+1 #Adds one to the images
                    sensor.skip_frames(time = 1000)
                    img = sensor.snapshot() #Take picture
                    timeseconds = round(pyb.elapsed_millis(start)/1000)
                    timeminutes = round(pyb.elapsed_millis(start)/60000)
                    img.draw_string(0,0, "Time Since Launch: " + str(timeminutes) + " Minutes " + str(timeseconds) + " Seconds ", 986)
                else:
                    previousTask = False
                    numImg = numImg+1 #Adds one to the images
                    sensor.skip_frames(time = 1000)
                    img = sensor.snapshot() #Take picture
                    timeseconds = round(pyb.elapsed_millis(start)/1000)
                    timeminutes = round(pyb.elapsed_millis(start)/60000)
                    img.draw_string(0,0, "Time Since Launch: " + str(timeminutes) + " Minutes " + str(timeseconds) + " Seconds ", 986)
                    img.save("Jun"+str(numImg)+".jpg") #Save to SD Card
                    print("Save")
            if task == 'D':
                previousTask = True
                sensor.set_pixformat(sensor.GRAYSCALE) #Color image (RGB)
                sensor.skip_frames(time = 1000)
            if task == 'E':
                previousTask = True
                sensor.set_pixformat(sensor.RGB565) #Color image (RGB)
                sensor.skip_frames(time = 1000)
            if task == 'F':
                img = img.replace(vflip=1,hmirror=1) #Rotate image 180
                img.save("Jun"+str(numImg)+".jpg") #Save to SD Card
                print("Save")
                sensor.skip_frames(time = 1000)
            if task == 'G':
                originalImg = img.copy()
                img = img.to_bitmap() # Apply filter
                img.save("Jun"+str(numImg)+".jpg") #Save to SD Card
                print("Save")
                sensor.skip_frames(time = 1000)
            if task == 'H':
                img = originalImg
                img.save("Jun"+str(numImg)+".jpg") #Save to SD Card
                print("Save")
                sensor.skip_frames(time = 1000)
        print("Done")
        done = True

    except OSError as err:
        pass # Don't care about errors - so pass.
        # Note that there are 3 possible errors. A timeout error, a general purpose error, or
        # a busy error. The error codes are 116, 5, 16 respectively for "err.arg[0]".#     Wire.requestFrom(0x12, temp);
