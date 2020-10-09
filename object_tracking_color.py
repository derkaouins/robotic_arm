#todo verify length between refRight and objects
import servoMotor as sm
import time
import cv2
import numpy as np
import imutils
import argparse
import math
from imutils import contours
from colorlabeler import ColorLabeler
from imutils import perspective
from scipy.spatial import distance as dist

#funcation: calculation of middle point of line
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
    
    
def sort_contours(cnts, method="bottom-to-top"):
    # initialize the reverse flag and sort index
    reverse = False
    i = 0
 
    # handle if we need to sort in reverse
    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True
 
    # handle if we are sorting against the y-coordinate rather than
    # the x-coordinate of the bounding box
    if method == "top-to-bottom" or method == "bottom-to-top":
        i = 1
 
    # construct the list of bounding boxes and sort them from top to
    # bottom
    boundingBoxes = [cv2.boundingRect(c) for c in cnts]
    (cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
        key=lambda b:b[1][i], reverse=reverse))
 
    # return the list of sorted contours and bounding boxes
    return cnts


#parsing the command arguments (width)
ap = argparse.ArgumentParser()
ap.add_argument("-w", "--width", type=float, required=True, help="width of the left-most object in the image (in cm)")
ap.add_argument("-l1", "--length1", type=float, required=True,  help="the length between left-most object and the buttom-most object (in cm)")
ap.add_argument("-l2", "--length2", type=float, required=True,  help="the length between right-most object and the buttom-most object (in cm)")
args = vars(ap.parse_args())

#device = cv2.VideoCapture('http://192.168.43.1:8080/video')  #'http://192.168.43.1:8080/video'

#ret, frame = device.read()

colors = ((0, 0, 255), (240, 0, 159), (0, 165, 255), (255, 255, 0), (255, 0, 255))

#ColorLabeler Object
cl = None

#between 1 and 3
nbrClasses = 2
supervised = False

if supervised:
    places = {0:(2.5,4), 1:(2.5, 9),2:(12.5,4)}
else:
    if nbrClasses == 1:
        places = {0:(2.5, 8, 12)}
    elif nbrClasses == 2:
        places = {0:(2.5, 8, 12), 1:(12.5, 8, 12)}
    elif nbrClasses == 3:
        places = {0:(2.5, 8, 12), 1:(2.5, 5, 9),2:(12.5, 8, 12)}
    else:
        places = {0:(2.5, 8, 12), 1:(2.5, 5, 9),2:(12.5, 8, 12),2:(12.5, 5, 9)}

d0 = args["width"]    #length between refLeft and refRight
d1 = args["length1"]    #length refLeft and center
d2 = args["length2"]    #length refRight and center

angle0 = 0      
angle1 = 0
angle2 = 0

#the angles 
angle0 = math.acos((((d1**2)+(d2**2)-(d0**2))/(2*d1*d2)))
angle1 = math.acos((((d0**2)+(d2**2)-(d1**2))/(2*d0*d2)))
angle2 = math.acos((((d0**2)+(d1**2)-(d2**2))/(2*d0*d1)))

#triangle height
H = 0.5 * math.sqrt((d0 + d1 + d2) * (-d0 + d1 + d2) * (d0 - d1 + d2) * (d0 + d1 - d2))/d0

# h line x base
H_base = d1 * math.cos(angle2)

#colors classes detection..
#contoursFirst = None


#sm.gripOpen()

sm.setup()
hasMoved = True
    
while True :
    #make the robot in the start position
    if hasMoved:
        hasMoved = False
        sm.ServoUpdate(2, 5.8)
        sm.ServoUpdate(3, 5.8)
        time.sleep(sm.sleepTime)
    
    device = cv2.VideoCapture('http://192.168.43.1:8080/video')
    ret, frame = device.read()
    #cv2.imshow("original", frame)
    
    resized = imutils.resize(frame, width=600)

    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)[1]

    mask = np.zeros(thresh.shape, np.uint8) 
    
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)


    result = cv2.bitwise_and(resized, resized, mask=thresh)
    
    (cnts, _) = contours.sort_contours(cnts)
    
        
    #only accepted contours
    tmp = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area > 500 and area < 20000:
            tmp.append(c)

    cnts = tmp
    
    
    #not enough objects
    if len(cnts) <= 2:
        print("completed successfully..")
        exit(0)
    
    
        
    #configure the left reference object
    c = cnts[0]
    rect = cv2.minAreaRect(c)
    rect = cv2.boxPoints(rect)
    rect = np.array(rect, dtype="int")
    box = perspective.order_points(rect)
    
    cX = np.average(box[:, 0])
    cY = np.average(box[:, 1])
    
    refLeft = (cX, cY)
    del cnts[0]
    cv2.drawContours(result, [box.astype("int")], -1, colors[1], 2)
    
        
    #configure the right reference object
    c = cnts[-1]
    rect = cv2.minAreaRect(c)
    rect = cv2.boxPoints(rect)
    rect = np.array(rect, dtype="int")
    box = perspective.order_points(rect)
    
    cX = np.average(box[:, 0])
    cY = np.average(box[:, 1])
    
    refRight = (cX, cY)
    del cnts[-1]
    cv2.drawContours(result, [box.astype("int")], -1, colors[1], 2)
    
    #pixels per cm
    D = dist.euclidean((refLeft[0], refLeft[1]), (refRight[0], refRight[1]))
    refRatio = D / d0
    
    #distance and line between refLeft and refRight
    cv2.line(result, (int(refLeft[0]), int(refLeft[1])), (int(refRight[0]), int(refRight[1])), colors[0], 2)
    d0 = dist.euclidean((refLeft[0], refLeft[1]), (refRight[0], refRight[1])) / refRatio
    (mX, mY) = midpoint((refLeft[0], refLeft[1]), (refRight[0], refRight[1]))
    cv2.putText(result, "{:.1f}cm".format(d0), (int(mX), int(mY - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    
    
    #sorting the remaining contours from buttom 2 top
    if len(cnts) > 1:
        cnts = sort_contours(cnts)  
    
    
    #every contour with its mean color
    i = 0
    rows, cols = (len(cnts), 2) 
    tmp = [[0]*cols]*rows
    for c in cnts:
        mask = np.zeros(resized.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [c[0]], -1, 255, -1)
        mean = cv2.mean(resized, mask=mask)[:3]
        tmp[i] = [c,mean]
        i += 1
    
    cnts = tmp
    
    
    
    #if it is the first time
    if cl == None:
        cl = ColorLabeler(cnts,nbrClasses)
        continue
        
    
    
    #colors classification
    #cnts = cl.getCenters(cnts)
    
    #every contour with its label
    i = 0
    labels = cl.getLabel(cnts,supervised = supervised)
    rows, cols = (len(cnts), 3) 
    tmp = [[0]*cols]*rows
    for c in cnts:
        tmp[i] = [c[0],c[1],labels[i]]
        i += 1
    
    cnts = tmp
    
    
    
    
    
    #taking the object to the right place
    c = cnts[0]
        
    rect = cv2.minAreaRect(c[0])
    rect = cv2.boxPoints(rect)
    rect = np.array(rect, dtype="int")
    box = perspective.order_points(rect)
        
    cX = np.average(box[:, 0])
    cY = np.average(box[:, 1])
        
    cv2.drawContours(result, c[0], -1, colors[1], 1)
    cv2.circle(result, (cX, cY), 3, (255, 255, 255), -1)

    cv2.drawContours(result,[rect],0,colors[0],2)
        
    cv2.drawContours(result, [box.astype("int")], -1, colors[0], 2)
        
    color = c[2]
        
    cv2.line(result, (int(refLeft[0]), int(refLeft[1])), (int(cX), int(cY)), colors[0], 2)
    cv2.line(result, (int(refRight[0]), int(refRight[1])), (int(cX), int(cY)), colors[0], 2)
        
    D1 = dist.euclidean((refLeft[0], refLeft[1]), (cX, cY)) / refRatio
    D2 = dist.euclidean((refRight[0], refRight[1]), (cX, cY)) / refRatio
        
    #triangle height
    h = 0.5 * math.sqrt((d0 + D1 + D2) * (-d0 + D1 + D2) * (d0 - D1 + D2) * (d0 + D1 - D2))/d0
        
    #refRight refLeft object angle
    angle = math.acos((((d0**2)+(D1**2)-(D2**2))/(2*d0*D1)))
        
    # h line x base
    h_base = D1 * math.cos(angle)
        
    #object is below or above the line between refLeft and refRight
    m = (refLeft[1]-refRight[1])/(refLeft[0]-refRight[0])
    b = (refLeft[0]*refRight[1] - refRight[0]*refLeft[1])/(refLeft[0]-refRight[0])
    isBelow = m*cX+b < cY
        
    #angleTmp = angle2 -  math.acos((((d0**2)+(D1**2)-(D2**2))/(2*d0*D1)))
    if isBelow:
        distanceFinal = math.sqrt((H_base-h_base)**2 + (H - h)**2)
    else:
        distanceFinal = math.sqrt((H_base-h_base)**2 + (H + h)**2)
    
    angleFinal = math.acos(((d1**2)+(distanceFinal**2)-(D1**2))/(2*d1*distanceFinal))
        
    (mX, mY) = midpoint((refLeft[0], refLeft[1]), (cX, cY))
    cv2.putText(result, "{:.1f}cm".format(D1), (int(mX), int(mY - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    (mX, mY) = midpoint((refRight[0], refRight[1]), (cX, cY))
    cv2.putText(result, "{:.1f}cm".format(D2), (int(mX), int(mY - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        
    cv2.putText(result, "{} {}".format(str(i),color), (int(cX) - 20, int(cY) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
    
    cv2.imshow("result",result)
    
    print ("distance final = ", distanceFinal)
    print ("angle final = ", angleFinal)
    
      
    sm.updateAngle(angleFinal)
    time.sleep(sm.sleepTime)
    
    if not sm.servoUpdatePerCm(distanceFinal):
        print ("object is too far or too close..")
        device.release()
        continue
    hasMoved = True
        
    time.sleep(2*sm.sleepTime)
    
    sm.gripClose()
    time.sleep(sm.sleepTime)    
    
    sm.ServoUpdate(2, 5)
    time.sleep(sm.sleepTime)
    
    sm.ServoUpdate(1, places[c[2]][0])
    sm.ServoUpdate(2, places[c[2]][1])
    sm.ServoUpdate(3, places[c[2]][2])
    time.sleep(sm.sleepTime)
    
    sm.gripOpen()
    time.sleep(sm.sleepTime)
    
    #sm.ServoUpdate(2,12.5)
    #time.sleep(sm.sleepTime)
    #sm.ServoUpdate(3, 5)
    #time.sleep(sm.sleepTime)
    

    device.release()

    if cv2.waitKey(100) == 27:
        break


device.release()
cv2.destroyAllWindows()


