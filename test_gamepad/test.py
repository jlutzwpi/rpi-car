import time
import threading
import io
import picamera
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from evdev import InputDevice,categorize,ecodes,KeyEvent
from Motor import *
from servo import *
from Buzzer import *
from Command import COMMAND as cmd
from Led import *
from Thread import *
from threading import Timer
from threading import Thread


gamepad = InputDevice('/dev/input/event0')
car = Motor()
car.setMotorModel(0,0,0,0)
servos = Servo()
curPosServ0 = 90
curPosServ1 = 90
servos.setServoPwm('0', curPosServ0)
servos.setServoPwm('1', curPosServ1)
led = Led()
ledCount = 0
ledStart = False
Led_Mode=Thread(target=led.ledMode,args=('4',))
buzzer = Buzzer()

# Initialize the camera
camera = PiCamera(resolution=(320,240),framerate=32)
# Generates a 3D RGB array and stores it in rawCapture
raw_capture = PiRGBArray(camera, size=(320, 240))
# Wait a certain number of seconds to allow the camera time to warmup
time.sleep(0.1)

def streamVid():
    # Capture frames continuously from the camera
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True): 
        # Grab the raw NumPy array representing the image
        image = frame.array
        # Display the frame using OpenCV
        cv2.imshow("Frame", image)
        # Wait for keyPress for 1 millisecond
        key = cv2.waitKey(1) & 0xFF
        # Clear the stream in preparation for the next frame
        raw_capture.truncate(0)
        # If the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        
#start a thread to stream video
try:
    vid=Thread(target=streamVid)
    vid.start()
    print('Video started!')
except:
    print('Video error with cv!')

#control the car with the game pad
for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.scancode == 305:
                print('Right')
                car.setMotorModel(2500,2500,-2000,-2000)
            elif keyevent.scancode == 304:
                print ('Forward')
                car.setMotorModel(2500,2500,2500,2500)
            elif keyevent.scancode == 307:
                print ('Left')
                car.setMotorModel(-2000,-2000,2500,2500)
            elif keyevent.scancode == 306:
                print ('Backward')
                car.setMotorModel(-2500,-2500,-2500,-2500)
            elif keyevent.scancode == 309: #R1 button
                print ('Buzzer')
                buzzer.run('1')
                time.sleep(0.5)
                buzzer.run('0')
            elif keyevent.scancode == 310: #L2 button
                print ('Quit!')
                led.ledMode('0')
                stop_thread(Led_Mode)
                stop_thread(vid)
                break
            elif keyevent.scancode == 311: #R2 button
                ledCount = ledCount+1
                print ('LED lights! LED count: ' + str(ledCount))
                #do led light stuff here
                if ledCount == 1:
                    #check if we've already cycled through, can't start multiple threads
                    if ledStart == False:
                        try:
                            Led_Mode.start()
                            ledStart = True
                        except (KeyboardInterrupt, SystemExit):
                            stop_thread(Led_Mode)
                            stop_thread(vid)
                            break
                    else:
                        led.ledMode('0')
                    
                elif ledCount == 2:
                    #do something
                    led.ledMode('0')
                    ledCount=0
                    stop_thread(Led_Mode)
        if keyevent.keystate == KeyEvent.key_up:
            #turn off motor
            car.setMotorModel(0,0,0,0)
    #left part of controller is up,down,left,right for servos to control the camera
    elif event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        #control the servos, move 5 degrees at a time (controller returns -1 to 1
        move5Deg = 5 * absevent.event.value
        #set serve range
        servoRange = range(30,150)
        #only attempt to move if button was pressed
        if move5Deg !=0:
            if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0X':
                #move servo 0 by 5 degrees
                curPosServ0 = curPosServ0 + move5Deg
                if curPosServ0 in servoRange:
                    servos.setServoPwm('0', curPosServ0)
                    time.sleep(0.01)
                else:
                    print ('Servo0 out of range: ' + str(curPosServ0))
                    curPosServ0 = curPosServ0 - move5Deg
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0Y':
                #move servo 1 by 5 degree, have to subtract here, up is - number
                curPosServ1 = curPosServ1 - move5Deg
                if curPosServ1 in servoRange:
                    servos.setServoPwm('1', curPosServ1)
                    time.sleep(0.01)
                else:
                    print ('Servo1 out of range: ' + str(curPosServ1))
                    curPosServ1 = curPosServ1 - move5Deg
                    
cv2.destroyAllWindows()
        