import cv2
from cvzone.HandTrackingModule import HandDetector
import mouse
import threading
import numpy as np
import time
import tkinter as tk
from tkinter import ttk



frameR = 100
cam_w, cam_h = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, cam_w)
cap.set(4, cam_h)
detector = HandDetector(detectionCon=0.9, maxHands=1)

l_delay = 0
r_delay = 0
double_delay = 0
is_running = threading.Event()

def l_clk_delay():
    global l_delay
    time.sleep(1)
    l_delay = 0

def r_clk_delay():
    global r_delay
    time.sleep(1)
    r_delay = 0

def double_clk_delay():
    global double_delay
    time.sleep(2)
    double_delay = 0

def start_camera():
    is_running.set()
    threading.Thread(target=camera_loop).start()

def stop_camera():
    is_running.clear()

def camera_loop():
    global l_delay, r_delay, double_delay
    while is_running.is_set():
        success, img = cap.read()
        if success:
            img = cv2.flip(img, 1)
            hands, img = detector.findHands(img, flipType=False)
            cv2.rectangle(img, (frameR, frameR), (cam_w - frameR, cam_h - frameR), (255, 0, 255), 2)
            if hands:
                lmlist = hands[0]['lmList']
                ind_x, ind_y = lmlist[8][0], lmlist[8][1]
                mid_x, mid_y = lmlist[12][0], lmlist[12][1]
                cv2.circle(img, (ind_x, ind_y), 5, (0, 255, 255), 2)
                cv2.circle(img, (mid_x, mid_y), 5, (0, 255, 255), 2)
                fingers = detector.fingersUp(hands[0])

                # Mouse movement
                if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 1:
                    conv_x = int(np.interp(ind_x, (frameR, cam_w - frameR), (0, 1536)))
                    conv_y = int(np.interp(ind_y, (frameR, cam_h - frameR), (0, 864)))
                    mouse.move(conv_x, conv_y)
                    print(conv_x, conv_y)

                # Mouse Button Clicks
                if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 1:
                    if abs(ind_x - mid_x) < 25:
                        # Left Click
                        if fingers[4] == 0 and l_delay == 0:
                            mouse.click(button="left")
                            l_delay = 1
                            threading.Thread(target=l_clk_delay).start()
                        # Right Click
                        if fingers[4] == 1 and r_delay == 0:
                            mouse.click(button="right")
                            r_delay = 1
                            threading.Thread(target=r_clk_delay).start()
                # Mouse Scrolling
                if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 0 and fingers[4] == 0:
                    if abs(ind_x - mid_x) < 25:
                        mouse.wheel(delta=-1)
                if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 0 and fingers[4] == 1:
                    if abs(ind_x - mid_x) < 25:
                        mouse.wheel(delta=1)

                # Double Mouse Click
                if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 0 and fingers[4] == 0 and double_delay == 0:
                    double_delay = 1
                    mouse.double_click(button="left")
                    threading.Thread(target=double_clk_delay).start()

            cv2.imshow("Camera Feed", img)
            cv2.waitKey(1)

# Create GUI
root = tk.Tk()
root.title("Hand Gesture Control Mouse")

# Start Button
start_button = ttk.Button(root, text="START", command=start_camera)
start_button.pack()

# Stop & Exit Button
stop_exit_button = ttk.Button(root, text="STOP & EXIT", command=stop_camera)
stop_exit_button.pack()

# Run the GUI loop
root.mainloop()
