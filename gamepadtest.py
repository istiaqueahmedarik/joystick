import time
import pygame
import socketio
import keyboard
# import rospy
# from std_msgs.msg import int64MultiArray,String
from flask_cors import CORS
from flask import Flask,jsonify,request
from flask_socketio import SocketIO, emit
import zlib
sio = socketio.Client()


DEADZONE = 30

sio.connect('http://192.168.1.111:5476')
# sio.connect('http://192.168.1.177:5476')

sio.on('connect', lambda: print('Connected to server'))
sio.on('disconnect', lambda: print('Disconnected from server'))


def emit_with_retry(event, message, namespace, max_retries=300, retry_delay=1):
    print(message)
    attempt = 0
    while attempt < max_retries:
        try:
            sio.emit(event, message, namespace=namespace)
            break  # If emit succeeds, break out of the loop
        except Exception as e:
            print(f"Error: {e}. Retrying...")
            time.sleep(retry_delay)  # Wait before retrying
            attempt += 1
    if attempt == max_retries:
        print("Failed to emit after several retries.")


# sio.on('rover', lambda data: print(data.message))

# waypoints = [] 
def getMotorSpeed(x,y):
    if(x>1400 and x<1600 and (y>1600 or y<1400)):
        return (y,y)
    elif(y>1400 and y<1600):
        return (x,3000-x)
    else:
        l = int((((y-1500)*(x-1500))/500))
        r = (y-l)
        return (l,r)
    

def joystick_to_motor_speed(x, y):
    # if((x<1500+DEADZONE and x>1500-DEADZONE) or (y<1500+DEADZONE and y>1500-DEADZONE)):
    #     left_motor = 1500
    #     right_motor = 1500
    #     return int(left_motor), int(right_motor)
    # Normalize the joystick values
    x_norm = (x - 1500) / 500
    y_norm = (y - 1500) / 500
    
    # Calculate motor speeds
    left_speed_norm = y_norm + x_norm
    right_speed_norm = y_norm - x_norm
    
    # Rescale to motor speed range
    left_speed = 1500 + 500 * left_speed_norm
    right_speed = 1500 + 500 * right_speed_norm
    
    # Ensure the values are within the valid range
    left_speed = max(1000, min(2000, left_speed))
    right_speed = max(1000, min(2000, right_speed))

    return int(left_speed), int(right_speed)

open1=1000
open=1000
cnt1 = 0
cnt2 = 0
flag1 = False
flag2 = False
flag3 = [0]*15
def joystick():
    # rospy.init_node('joystickVal', anonymous=True)
    # pub = rospy.Publisher('joystick', String, queue_size=15)
    # rate = rospy.Rate(10)
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Initialized joystick:", joystick.get_name())

    axes = joystick.get_numaxes()
    buttons = joystick.get_numbuttons()

    print("Number of axes:", axes)
    print("Number of buttons:", buttons)

    while True:
        global open
        global open1
        global cnt1 
        global cnt2
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            emit_with_retry('joystick_data', "LOL", namespace='/')
            break
        
        pygame.event.pump()
        leftY = joystick.get_axis(3)
        leftX = joystick.get_axis(2)
        rightY = joystick.get_axis(1)
        rightX = joystick.get_axis(0)
        gripper_of = joystick.get_button(2)
        gripper_on = joystick.get_button(3)
        base = joystick.get_axis(4)
        lifter_mode = joystick.get_axis(5)
        speed_mode = joystick.get_button(1)
        lifter = joystick.get_axis(6)
        arm = joystick.get_button(0)
        # map from -1 to 1 to 1000 to 2000
        # leftMotor = (leftY+1)*500+1000
        leftY = int((leftY+1)*500+1000)
        leftX = int((leftX+1)*500+1000)
        rightY = int((rightY+1)*500+1000)
        rightX = int((rightX+1)*500+1000)
        arm = int((arm+1)*500+1000)
        base = int((base+1)*500+1000)
        speed_mode = int((speed_mode+1)*500+1000)
        lifter = int((lifter+1)*500+1000)
        lifter_mode = int((lifter_mode+1)*500+1000)
        gripper_of = int((1 - gripper_of) * 500 + 1000)
        gripper_on = int(gripper_on * 500 + 1500)
        gripper = gripper_on
        if(gripper_of!=1500):
            gripper = gripper_of

        l = False
        if(keyboard.is_pressed('l')):
            l = True
        w=False
        if(keyboard.is_pressed('w')):
            w=True
        s=False
        if(keyboard.is_pressed('s')):
            s=True
        a=False
        if(keyboard.is_pressed('a')):
            a=True
        d=False
        if(keyboard.is_pressed('d')):
            d=True
        o=False
        if(keyboard.is_pressed('p')):
            o=True
            cnt1+=1
            cnt1%=3
        if(o):
            if(cnt1==0 and open==1000):
                open=2000
            elif(cnt1==0 and open==2000):
                open=1000

            
        o1=False
        if(keyboard.is_pressed('o')):
            o1=True
            cnt2+=1
            cnt2%=3
        if(o1):
            
            if(cnt2==0 and open1==1000):
                open1=2000
            elif(cnt2==0 and open1==2000):
                open1=1000
        up_down=1500
        left_right=1500
        if(w):
            up_down=1000
        if(s):
            up_down=2000
        if(a):
            left_right=2000
        if(d):
            left_right=1000
        print(open)
        light = int((l+1)*500+1000)
        print(open)
        # print(f"leftY: {leftY}, leftX: {leftX}, rightY: {rightY}, rightX: {rightX}, arm: {arm}, speed_mode: {speed_mode}, arm_mode: {arm_mode}, lifter: {lifter}")
        (leftMotor,rightMotor) = joystick_to_motor_speed(rightX,rightY)
        # s = str(23)+s

        #map all the value from 10-20

        

        s = "["
        if(abs(1500-leftMotor)>DEADZONE):
            flag3[0] = 0
            s += "0"+str(leftMotor)+","
        elif(flag3[0]==0):
            flag3[0] = 1
            s += "0"+str(1500)+","
        
        if(abs(1500-rightMotor))>DEADZONE:
            flag3[1] = 0
            s += "1"+str(rightMotor)+","
        elif(flag3[1]==0):
            flag3[1] = 1
            s += "1"+str(1500)+","
        if(abs(leftX-1500)>5):
            flag3[2] = 0
            s += "2"+str(leftX)+","
        elif(flag3[2]==0):
            flag3[2] = 1
            s += "2"+str(1500)+","
        if(abs(leftY-1500)>5):
            flag3[3] = 0
            s += "3"+str(leftY)+","
        elif(flag3[3]==0):
            flag3[3] = 1
            s += "3"+str(1500)+","
        if(abs(base-1500)>5):
            flag3[4] = 0
            s+= "4"+str(base)+","
        elif(flag3[4]==0):
            flag3[4] = 1
            s+= "4"+str(1500)+","
        if(lifter!=1500):
            flag3[5] = 0
            s+= "5"+str(lifter)+","
        elif(flag3[5]==0):
            flag3[5] = 1
            s+= "5"+str(1500)+","
        if(gripper!=1500):
            flag3[6] = 0
            s+= "6"+str(gripper)+","
        elif(flag3[6]==0):
            flag3[6] = 1
            s+= "6"+str(1500)+","
        if(lifter_mode!=1500):
            flag3[7] = 0
            s+= "7"+str(lifter_mode)+","
        elif(flag3[7]==0):
            flag3[7] = 1
            s+= "7"+str(1500)+","
        if(speed_mode!=1500):
            flag3[8] = 0
            s+= "8"+str(speed_mode)+","
        elif(flag3[8]==0):
            flag3[8] = 1
            s+= "8"+str(1500)+","
        if(arm!=1500):
            flag3[9] = 0
            s+= "9"+str(arm)+","
        elif(flag3[9]==0):
            flag3[9] = 1
            s+= "9"+str(1500)+","
        if(light!=1500):
            flag3[10] = 0
            s+= "A"+str(light)+","
        elif(flag3[10]==0):
            flag3[10] = 1
            s+= "A"+str(1500)+","
        if(up_down!=1500):
            flag3[11] = 0
            s+= "B"+str(up_down)+","
        elif(flag3[11]==0):
            flag3[11] = 1
            s+= "B"+str(1500)+","
        if(left_right!=1500):
            flag3[12] = 0
            s+= "C"+str(left_right)+","
        elif(flag3[12]==0):
            flag3[12] = 1
            s+= "C"+str(1500)+","
        if(open!=1500):
            flag3[13] = 0
            s+= "D"+str(open)+","
        elif(flag3[13]==0):
            flag3[13] = 1
            s+= "D"+str(1500)+","
        if(open1!=1500):
            flag3[14] = 0
            s+= "E"+str(open1)
        elif(flag3[14]==0):
            flag3[14] = 1
            s+= "E"+str(1500)
        s+="]"


        if(s[len(s)-2]==','):
            s = s[:len(s)-2]+']'


        # if(rightMotor!=1500):

        #     s += "1"+str(rightMotor)+","
        # if(leftX!=1500):
        #     s += "2"+str(leftX)+","
        # if(leftY!=1500):
        #     s += "3"+str(leftY)+","
        # if(base!=1500):
        #     s+= "4"+str(base)+","
        # if(lifter!=1500):
        #     s+= "5"+str(lifter)+","
        # if(gripper!=1500):
        #     s+= "6"+str(gripper)+","
        # if(lifter_mode!=1500):
        #     s+= "7"+str(lifter_mode)+","
        # if(speed_mode!=1500):
        #     s+= "8"+str(speed_mode)+","
        # if(arm!=1500):
        #     s+= "9"+str(arm)+","
        # if(light!=1500):
        #     s+= "A"+str(light)+","
        # if(up_down!=1500):
        #     s+= "B"+str(up_down)+","
        # if(left_right!=1500):
        #     s+= "C"+str(left_right)+","
        # if(open!=1500):
        #     s+= "D"+str(open)+","
        # if(open1!=1500):
        #     s+= "E"+str(open1)
        # s+="]"



        # s += "3"+str(leftY)+","
        # s+= "4"+str(base)+","
        # s+= "5"+str(lifter)+","
        # s+= "6"+str(gripper)+","
        # s+= "7"+str(lifter_mode)+","
        # s+= "8"+str(speed_mode)+","
        # s+= "9"+str(arm)+","
        # s+= "A"+str(light)+","
        # s+= "B"+str(up_down)+","
        # s+= "C"+str(left_right)+","
        # s+= "D"+str(open)+","
        # s+= "E"+str(open1)
        # s+="]"
        print(arm)
        emit_with_retry('armMsg', 'noarm' if arm == 2000 else 'arm', namespace='/')
        if arm == 2000:
            emit_with_retry('joystick_data', s, namespace='/')
        else:
            emit_with_retry('joystick_data',"LOL",namespace="/")

        # left_motor,right_motor,leftX,leftY,DL,DR,DU,DD,LT,RT,B0,B1,B2,B3,B4,B5,B6,B7,B8,B9,B10
        time.sleep(0.1)
        # rate.sleep()
        # 15(B) 16(x) 17(y)
        # 11 (DR)
        # 8 (DU)
        # 13

    

if __name__ == "__main__":
        joystick()


# [leftX,leftY,rightX,rightY,B0,B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,DL,DR,DU,DD,LT,RT]