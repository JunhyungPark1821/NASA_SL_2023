import sensor, image, time, machine, pyb

#Initialize Camera Settings
#numImg = 1

#Clock Timer
clock = time.clock()

#Initialize Timer
start = pyb.millis()


#For loop to take multiple images
for numImg in range (5):
    print("Image Number:" + str(numImg))
    sensor.reset() #Clear the sensor
    sensor.set_pixformat(sensor.GRAYSCALE) #Color image (RGB)
    sensor.set_framesize(sensor.QVGA) #Framesize
    sensor.skip_frames(time = 1000) #Time in-between images
    img = sensor.snapshot().replace(vflip=numImg%2,hmirror=numImg%2) #takes image and flips them alternating
    timeseconds = round(pyb.elapsed_millis(start)/1000)
    timeminutes = round(pyb.elapsed_millis(start)/60000)
    imgtime = img.draw_string(0,0, "Time Since Launch: " + str(timeminutes) + " Minutes " + str(timeseconds) + " Seconds ", 986)
    imgtime.save("TimestampAttempt"+str(numImg)+".jpg") #Save to SD Card
    numImg = numImg+1 #Adds one to the images
    print("Saved")
machine.reset()

#FOR REFERENCE
#clock.tick() == the times since the command was last called
#print(clock.fps()) == prints the frames per second
#COLORS:
    #200 - Gray
    #500 - Blue
    #888 - Blue
    #1000 - Green
    #2000 - Bright Green
#INITIALIZE DATE AND TIME:
    #rtc = pyb.RTC()
    #rtc.datetime((2022, 3, 17, 7, 30, 0, 0, 0))
#FILTERS:
    #COLOR - RGB565
    #GRAYSCALE - GRAYSCALE

