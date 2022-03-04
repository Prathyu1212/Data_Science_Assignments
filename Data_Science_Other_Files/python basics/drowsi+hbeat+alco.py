from scipy.spatial import distance
from imutils import face_utils
import RPi.GPIO as GPIO
import imutils
import dlib
import cv2
from time import sleep
import threading
import time
import serial
stat_led=21
data_led=20
buzzer=16
mot1_fwd=26
mot1_rev=19
sw=13
ws=6
acc=5
bcc=12
alcohol=8
hbeat=24
hbeat_flag=0
hbeat_count=0
ser=serial.Serial(port='/dev/ttyS0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
global sleep_flag
sleep_flag=0

def main():
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(buzzer,GPIO.OUT)
        GPIO.setup(mot1_fwd,GPIO.OUT)
        GPIO.setup(mot1_rev,GPIO.OUT)
        GPIO.setup(stat_led,GPIO.OUT)
        GPIO.setup(data_led,GPIO.OUT)
        GPIO.setup(sw,GPIO.IN)
        GPIO.setup(ws,GPIO.IN)
        GPIO.setup(acc,GPIO.IN)
        GPIO.setup(bcc,GPIO.IN)
        GPIO.output(buzzer,GPIO.LOW)
        GPIO.output(mot1_fwd,False)
        GPIO.output(mot1_rev,False)
        global engine_flag
        global sw_flag
        sw_flag=0
        engine_flag=0
        threading.Thread(target=buffering).start()
        threading.Thread(target=detect).start()
        send_flag=1
        while True:
            sw_status=GPIO.input(sw)
            alco_status=GPIO.input(alcohol)
            if sw_status==0:
                sw_flag=1
                time.sleep(0.5)
                sw_status=GPIO.input(sw)
                if sw_status==1 and sw_flag==1 and alco_status==1 and engine_flag==0:
                    sw_flag=0
                    engine_flag=1
                    GPIO.output(data_led,True)
                    GPIO.output(mot1_fwd,True)
                    GPIO.output(mot1_rev,False)
                    time.sleep(0.5)
                    GPIO.output(data_led,False)
                    checking()
                if sw_status==1 and sw_flag==1 and engine_flag==1:
                    sw_flag=0
                    engine_flag=0
                    GPIO.output(mot1_fwd,False)
                    GPIO.output(mot1_rev,False)
                sw_flag=0
            res=heart_beat_measure()
            if res>60 and send_flag==1:
                    send_flag=0
                    ser.write('B')
            if res<60 and send_flag==0:
                    send_flag=1
                    
def eye_aspect_ratio(eye):
	A = distance.euclidean(eye[1], eye[5])
	B = distance.euclidean(eye[2], eye[4])
	C = distance.euclidean(eye[0], eye[3])
	ear = (A + B) / (2.0 * C)
	return ear
def checking():
        print('checking start')
        thresh = 0.25
        frame_check = 5
        detect = dlib.get_frontal_face_detector()
        predict = dlib.shape_predictor("/home/pi/shape_predictor_68_face_landmarks.dat")# Dat file is the crux of the code

        (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
        (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
        cap=cv2.VideoCapture(0)
        flag=0
        global engine_flag
        global sleep_flag
        while engine_flag==1:
                sw_status=GPIO.input(sw)
                acc_status=GPIO.input(acc)
                if sw_status==0:
                        sw_flag=1
                        GPIO.output(data_led,True)
                        time.sleep(0.5)
                        GPIO.output(data_led,False)
                        sw_status=GPIO.input(sw)
                        if sw_status==1 and sw_flag==1:
                                sw_flag=0
                                engine_flag=0
                                print('engine stoped')
                                GPIO.output(mot1_fwd,False)
                                GPIO.output(mot1_rev,False)
                ret,frame=cap.read()
                frame = imutils.resize(frame, width=450)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                subjects = detect(gray, 0)
                for subject in subjects:
                        shape=predict(gray,subject)
                        shape=face_utils.shape_to_np(shape)
                        leftEye = shape[lStart:lEnd]
                        rightEye = shape[rStart:rEnd]
                        leftEAR = eye_aspect_ratio(leftEye)
                        rightEAR = eye_aspect_ratio(rightEye)
                        ear = (leftEAR + rightEAR) / 2.0
                        if ear < thresh:
                            flag += 1
                            print (flag)
                            if flag >= frame_check:
                                print('sleepd')
                                engine_flag=0
                                sleep_flag=1
                                GPIO.output(buzzer,GPIO.HIGH)
                                for i in range (0,10):
                                        GPIO.output(mot1_fwd,True)
                                        time.sleep(0.2)
                                        GPIO.output(mot1_fwd,False)
                                        time.sleep(0.2)
                                #GPIO.output(mot1_fwd,False)
                                #GPIO.output(mot1_rev,False)
                        else:
                            print(flag)
                            flag = 0
                            GPIO.output(buzzer,GPIO.LOW)

        print('exited')
        time_flag=0
        cnt=0
        ws_flag=0
        ws_check=0
        while sleep_flag==1:
                while time_flag==0:
                        time.sleep(0.1)
                        cnt=cnt+1
                        ws_status=GPIO.input(ws)
                        if ws_status==0:
                                ws_flag=1
                                time.sleep(0.5)
                                ws_status=GPIO.input(ws)
                                if ws_status==1 and ws_flag==1:
                                        ws_flag=0
                                        GPIO.output(data_led,True)
                                        GPIO.output(buzzer,GPIO.LOW)
                                        engine_flag=0
                                        print('alarm stoped')
                                        ws_check=1
                                        sleep_flag=0
                                        time.sleep(0.5)
                                        GPIO.output(data_led,False)
                                        break
                        if cnt>150:
                                time_flag=1
                if ws_check==0:
                        print('vehicle automatically controlled')
                        GPIO.output(buzzer,GPIO.LOW)
                        sleep_flag=0
                        
                print('hai')
                                            
def buffering():
       
        while(1):
                GPIO.output(stat_led,True)
                time.sleep(0.2)
                GPIO.output(stat_led,False)
                time.sleep(0.2)

def detect():
        global engine_flag
        count=0
        while(1):
                acc_status=GPIO.input(acc)
                bcc_status=GPIO.input(bcc)
                if engine_flag==1 and acc_status==1:
                        count=count+1
                        print('hai')
                        time.sleep(0.5)                             
                if bcc_status==1:
                        count=count+1
                        print('hello')
                        time.sleep(0.5)
                if count>5:
                        print('accident detected')
                        GPIO.output(buzzer,True)
                        GPIO.output(data_led,True)
                        GPIO.output(mot1_fwd,False)
                        GPIO.output(mot1_rev,False)
                        engine_flag=0
                        ser.write('A'.encode())
                        time.sleep(0.5)
                        GPIO.output(data_led,False)
                        GPIO.output(buzzer,False)
                        
                        count=0
def heart_beat_measure():
    global hbeat_count
    global hbeat_flag
    count=0
    h_count
    while(count>10)
        hbeat_val=GPIO.input(hbeat)
        if hbeat_val==1:
            hbeat_flag=1
            hbeat_val=GPIO.input(hbeat)
            if hbeat_val==0 and hbeat_flag==1:
                h_count=h_count+1
        count=count+1
        if count==10:
            count=0
            hbeat_count=h_count*6
            h_count=0
        return hbeat_count
                                                
if __name__=="__main__":
        main()

