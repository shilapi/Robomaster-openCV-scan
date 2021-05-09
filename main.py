import numpy as np
import cv2 as cv
import time
import robomaster as robot

face_cascade = cv.CascadeClassifier('haarcascades/haarcascade_frontalface_alt2.xml')
eye_cascade = cv.CascadeClassifier('haarcascades/haarcascade_eye.xml')
cam_front = cv.VideoCapture('E:/素材/东方绿舟/650d/day2/MVI_0156.MOV')
# cam_back = cv.VideoCapture(0)
# print(video_capt.read())


def face_sync(facemodel, eyemodel, sourceimg, frame):
    sourceimg.set(cv.CAP_PROP_POS_FRAMES, frame)
    checkexist, framenow = sourceimg.read()  # read video frame by frame
    # print(frame)
    img = cv.resize(framenow, (800, 450))  # resize frame (depends on device)
    gray_capt = cv.cvtColor(img, cv.COLOR_BGR2GRAY)  # turn frame to grey
    faces_dected = facemodel.detectMultiScale(gray_capt, scaleFactor=1.2, minNeighbors=5)
    face_list = []
    for(x, y, w, h) in faces_dected:
        img = cv.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_gray = gray_capt[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        # return position of face
        pos_tempx = (x * 2 + w) / 2
        pos_tempy = (y * 2 + h) / 2
        pos_temp = pos_tempx, pos_tempy
        face_list.append(pos_temp)
    return face_list


def posdect(x, y):
    stop = False
    xdeg = (float(x) - 400) // 7
    if y <= 100:
        stop = True
    return xdeg, stop


# def robot_move (posx,ifstop) :


def position(robot_position):
    x, y, z = robot_position
    return x, y


frameNow = 0
ep_robot = robot.robot()
ep_robot.initialize(conn_type="rndis")
ep_chassis = ep_robot.chassis
ep_sensor = ep_robot.sensor
# 机器信息订阅
# ep_chassis.sub_position(freq=10, callback=positionBack)
# ep_sensor.sub_distance(freq=5, callback=distanceBack)
# main
while frameNow < int(cam_front.get(cv.CAP_PROP_FRAME_COUNT)):
    # dect start
    framerateBefore = time.time()
    facelist = face_sync(face_cascade, eye_cascade, cam_front, frameNow)
    print(facelist)
    print(frameNow)
    facex, facey = 0, 0
    cam_front.set(cv.CAP_PROP_POS_FRAMES, frameNow)
    check, frameshow = cam_front.read()
    picshow = cv.resize(frameshow, (800, 450))
    picshow = cv.cvtColor(picshow, cv.COLOR_BGR2GRAY)
    if len(facelist) != 0:
        for faces in facelist:
            facex, facey = faces
            picshow = cv.circle(picshow, (facex, facey), 50, (255, 0, 0), 2)
            cv.imshow('img', picshow)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    framerateAfter = time.time()
    framerate = 1/(framerateAfter-framerateBefore)
    print(framerate)
    frameNow = frameNow+1
    # dect_end
    # robot_start
    # robotx, roboty = position(robot_position = positionBack)
    z_val, stop_val = posdect(facex, facey)
    if stop_val:
        ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=1)
    else:
        ep_chassis.drive_speed(x=3.5, y=0, z=z_val, timeout=0.5)
