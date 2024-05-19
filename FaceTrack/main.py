# This script will detect faces via your webcam.
# Tested with OpenCV3

import cv2
import time
import board
from adafruit_motorkit import MotorKit

fireRange = 100
isMotor = False
mSpeed = .8


kit = MotorKit(i2c = board.I2C())


# distance from camera to object(face) measured
# centimeter
Known_distance = 25

# width of face in the real world or Object Plane
# centimeter
Known_width = 5.63

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


midPoint = (-1,-1)

# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX

# face detector object
face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")


def moveXMotor(t):
    if(t == 0):
        return False
    #kit.motor1.throttle = t
    #kit.motor2.throttle = t
    if((t < fireRange and t > -fireRange)):
        return True
    return False

def moveYMotor(t):
    #kit.motor3.throttle = t
    if ((t < fireRange and t > -fireRange)):
        return True
    return False

def moveZMotor():
    isMotor = True
    kit.motor1.throttle = mSpeed
    kit.motor2.throttle = mSpeed
    kit.motor3.throttle = mSpeed
    kit.motor4.throttle = mSpeed
    

def noneZMotor(isMotor):
    kit.motor1.throttle = None
    kit.motor2.throttle = None
    kit.motor3.throttle = None
    kit.motor4.throttle = None
    if(isMotor == True):
        time.sleep(1)
        isMotor = False

# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length


# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length) / face_width_in_frame

    # return the distance
    return distance


def face_data(image):
    face_width = 0  # making face width zero

    # converting color image to gray scale image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # detecting face in the image
    faces = face_detector.detectMultiScale(gray_image, 1.3, 5)

    # this is used to detect the face which
    # is closest to the web-cam on the first position
    faces = sorted(faces, key=lambda x: x[2] * x[3],
                   reverse=True)

    # only the first detected face is used
    faces = faces[:1]

    # len(faces) is the number of
    # faces showing in a frame
    if len(faces) == 1:
        # this is removing from tuple format
        face = faces[0]
    elif len(faces) == 0:
        noneZMotor(isMotor)

        # looping through the faces detect in the image
    # getting coordinates x, y , width and height
    print((midPoint[0]+fireRange,midPoint[1]+fireRange))
    cv2.rectangle(image, (midPoint[0]+fireRange,midPoint[1]+fireRange), (midPoint[0]-fireRange,midPoint[1]-fireRange), RED, 2)
    for (x, y, h, w) in faces:
        # draw the rectangle on the face
        cv2.rectangle(image, (x, y), (x + w, y + h), GREEN, 2)
        

        if (midPoint != (-1, -1)):
            cv2.line(frame, (midPoint), (int(x + w / 2), midPoint[1]), (255, 0, 0), 1)
            cv2.line(frame, (int(x + w / 2), midPoint[1]), (int(x + w / 2), int(y + h / 2)), (0, 0, 255), 1)
            cv2.putText(frame, "X offset: {}".format(int(x + w / 2) - midPoint[0]), (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(frame, "Y offset: {}".format(-1 * (int(y + h / 2) - midPoint[1])), (5, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(frame, "Square size: {}".format(w * h), (5, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 0, 0), 1, cv2.LINE_AA)

        # getting face width in the pixels
        face_width = w
        if(moveXMotor(int(x + w / 2) - midPoint[0]) and (moveYMotor(-1 * (int(y + h / 2) - midPoint[1])))):
            moveZMotor()
            
            


        print((int(x + w / 2) - midPoint[0]))
        print((-1 * (int(y + h / 2) - midPoint[1])))

    # return the face width in pixel
    return face_width


# reading reference_image from directory
ref_image = cv2.imread("Ref_image.jpg")

# find the face width(pixels) in the reference_image
ref_image_face_width = face_data(ref_image)

Focal_length_found = Focal_Length_Finder(
    Known_distance, Known_width, ref_image_face_width)

print(Focal_length_found)

# initialize the camera object so that we
cap = cv2.VideoCapture(0)

# looping through frame, incoming from
# camera/video
while True:

    _, frame = cap.read()
    frame = cv2.resize(frame, None, None, fx=2, fy=2)

    face_width_in_frame = face_data(frame)

    if face_width_in_frame != 0:

        Distance = Distance_finder(
            Focal_length_found, Known_width, face_width_in_frame)


        cv2.putText(
            frame, f"Distance: {round(Distance, 2)} IN", (50, 60),
            fonts, 0.3, GREEN, 1)

    # show the frame on the screen
    cv2.imshow("frame", frame)

    midPoint = int(cv2.getWindowImageRect('frame')[2] / 2), int(cv2.getWindowImageRect('frame')[3] / 2)

    # quit the program if you press 'q' on keyboard
    if cv2.waitKey(1) == ord("q"):
        kit.motor1.throttle = None
        kit.motor2.throttle = None
        kit.motor3.throttle = None
        kit.motor4.throttle = None
        break



# closing the camera
cap.release()

# closing the windows that are opened
cv2.destroyAllWindows()

