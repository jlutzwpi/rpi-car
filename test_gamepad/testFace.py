import time
import threading
import io
import picamera
import cv2
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
import pickle
import os.path
from os import path
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
from test_email import *
from datetime import datetime

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
screenshotCount = 0
ledStart = False
Led_Mode=Thread(target=led.ledMode,args=('4',))
buzzer = Buzzer()

#set up for face recognition
#Determine faces from encodings.pickle file model created from train_model.py
encodingsP = "encodings.pickle"
#use this xml file
cascade = "haarcascade_frontalface_default.xml"
# load the known faces and embeddings along with OpenCV's Haar
# cascade for face detection
data = pickle.loads(open(encodingsP, "rb").read())
detector = cv2.CascadeClassifier(cascade)

# Initialize the camera
vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
fps = FPS().start()
curFrame = vs.read() #initialize curFrame

emailAddr="jlutz@alum.wpi.edu"
msgBody="Justin was dumb enough to give his robot the ability to email people when his face is detected!  Silly Justin."
# get the Gmail API service
service = gmail_authenticate()

def streamVid():
    # loop over frames from the video file stream
    currentname = "unknown"
    while True:
        # grab the frame from the threaded video stream and resize it
        # to 500px (to speedup processing)
        frame = vs.read()
        frame = imutils.resize(frame, width=500)
        
        # convert the input frame from (1) BGR to grayscale (for face
        # detection) and (2) from BGR to RGB (for face recognition)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # detect faces in the grayscale frame
        rects = detector.detectMultiScale(gray, scaleFactor=1.1, 
            minNeighbors=5, minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE)

        # OpenCV returns bounding box coordinates in (x, y, w, h) order
        # but we need them in (top, right, bottom, left) order, so we
        # need to do a bit of reordering
        boxes = [(y, x + w, y + h, x) for (x, y, w, h) in rects]

        # compute the facial embeddings for each face bounding box
        encodings = face_recognition.face_encodings(rgb, boxes)
        names = []

        # loop over the facial embeddings
        for encoding in encodings:
            # attempt to match each face in the input image to our known
            # encodings
            matches = face_recognition.compare_faces(data["encodings"],
                encoding)
            name = "Unknown" #if face is not recognized, then print Unknown

            # check to see if we have found a match
            if True in matches:
                # find the indexes of all matched faces then initialize a
                # dictionary to count the total number of times each face
                # was matched
                matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                counts = {}

                # loop over the matched indexes and maintain a count for
                # each recognized face face
                for i in matchedIdxs:
                    name = data["names"][i]
                    counts[name] = counts.get(name, 0) + 1

                # determine the recognized face with the largest number
                # of votes (note: in the event of an unlikely tie Python
                # will select first entry in the dictionary)
                name = max(counts, key=counts.get)
                
                #If someone in your dataset is identified, print their name on the screen
                if currentname != name:
                    currentname = name
                    print(currentname)
                    
            
            # update the list of name
            names.append(name)

        # loop over the recognized faces
        for ((top, right, bottom, left), name) in zip(boxes, names):
            # draw the predicted face name on the image - color is in BGR
            cv2.rectangle(frame, (left, top), (right, bottom),
                (0, 255, 225), 2)
            y = top - 15 if top - 15 > 15 else top + 15
            cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                .8, (0, 255, 255), 2)
            if not path.exists("jlutz.jpg"):
                print ('Email sent!')
                cv2.imwrite("jlutz.jpg", frame)
                send_message(service, emailAddr, "Justin's Robo Cam from the Donovans!", msgBody, ["jlutz.jpg",])

        # display the image to our screen
        cv2.imshow("Robot Car Is Running!", frame)
        global curFrame
        curFrame = frame
        key = cv2.waitKey(1) & 0xFF

        # quit when 'q' key is pressed
        if key == ord("q"):
            break

        # update the FPS counter
        fps.update()
        
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
            elif keyevent.scancode == 308: #L1 button
                timenow = datetime.now()
                timeStr = timenow.strftime("%m-%d-%Y-%H%M%S")
                screen_path = 'screenshots/screenshot_' + timeStr + '.jpg'
                print ('Capture screenshot! - path: ' + screen_path)
                cv2.imwrite(screen_path, curFrame)
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

fps.stop()
cv2.destroyAllWindows()
vs.stop()       