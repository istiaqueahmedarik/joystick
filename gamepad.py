import time
import pygame
import socketio
# import rospy
# from std_msgs.msg import int64MultiArray,String
from flask_cors import CORS
from flask import Flask,jsonify,request
from flask_socketio import SocketIO, emit
sio = socketio.Client()

DEADZONE = 50

# sio.connect('http://192.168.1.111:5476')
sio.connect('http://192.168.1.155:5476')

sio.on('connect', lambda: print('Connected to server'))
sio.on('disconnect', lambda: print('Disconnected from server'))
# sio.on('rover', lambda data: print(data.message))

# waypoints = [] 
def getMotorSpeed(x,y):
    if(x>1400 and x<1000 and (y>1600 or y<1600)):
        return (y,y)
    elif(y>1400 and y<1600):
        return (x,3000-x)
def calculate_motor_speeds(x, y):
    # Normalize joystick inputs to range -1 to 1
    normalized_x = (x - 1500) / 500
    normalized_y = (y - 1500) / 500

    # Compute motor speeds in the normalized range
    left_motor_speed = normalized_y + normalized_x
    right_motor_speed = normalized_y - normalized_x

    # Ensure motor speeds stay within the range -1 to 1
    left_motor_speed = max(min(left_motor_speed, 1), -1)
    right_motor_speed = max(min(right_motor_speed, 1), -1)

    # Determine if the joystick is in a pure forward or backward position
    is_forward_backward = abs(normalized_x) < 0.1

    # Scale down the speeds based on distance from the center for diagonal movement
    if not is_forward_backward:
        scale_factor = max(abs(normalized_x), abs(normalized_y))
        left_motor_speed *= (1 - 0.5 * scale_factor)
        right_motor_speed *= (1 - 0.5 * scale_factor)

    # Map normalized motor speeds to the range 1000 to 2000
    left_motor = (left_motor_speed + 1) * 500 + 1000
    right_motor = (right_motor_speed + 1) * 500 + 1000
    if((left_motor<1500+DEADZONE and left_motor>1500-DEADZONE) or (right_motor<1500+DEADZONE and right_motor>1500-DEADZONE)):
        left_motor = 1500
        right_motor = 1500
    return int(left_motor), int(right_motor)

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

    # Initialize previous values
    prev_values = {
        "left_stick_x": 0,
        "left_stick_y": 0,
        "right_stick_x": 0,
        "right_stick_y": 0,
        "LT": 0,
        "RT": 0,
        "DPad_up": 0,
        "DPad_down": 0,
        "DPad_left": 0,
        "DPad_right": 0,
        "buttons": [0] * buttons  # Initialize to zeros for all buttons
    }

    while True:
        pygame.event.pump()

        # Read current values
        left_stick_x = joystick.get_axis(0)
        left_stick_y = joystick.get_axis(1)
        right_stick_x = joystick.get_axis(3)
        right_stick_y = joystick.get_axis(4)
        LT = joystick.get_axis(2)
        RT = joystick.get_axis(5)
        DPad_up = joystick.get_hat(0)[1]
        DPad_down = -joystick.get_hat(0)[1]
        DPad_left = -joystick.get_hat(0)[0]
        DPad_right = joystick.get_hat(0)[0]
        current_buttons = [joystick.get_button(i) for i in range(buttons)]
    
        # Check for changes and print if there are any
        changes_detected = False

        if (left_stick_x, left_stick_y) != (prev_values["left_stick_x"], prev_values["left_stick_y"]):
            print(f"Left Stick: xval={left_stick_x:.2f}; yval={left_stick_y:.2f}")
            changes_detected = True

        if (right_stick_x, right_stick_y) != (prev_values["right_stick_x"], prev_values["right_stick_y"]):
            print(f"Right Stick: xval={right_stick_x:.2f}; yval={right_stick_y:.2f}")
            changes_detected = True

        if LT != prev_values["LT"]:
            print(f"LT={LT:.2f}")
            changes_detected = True

        if RT != prev_values["RT"]:
            print(f"RT={RT:.2f}")
            changes_detected = True

        if DPad_up != prev_values["DPad_up"]:
            print("DPad Up pressed" if DPad_up == 1 else "DPad Up released")
            changes_detected = True

        if DPad_down != prev_values["DPad_down"]:
            print("DPad Down pressed" if DPad_down == 1 else "DPad Down released")
            changes_detected = True

        if DPad_left != prev_values["DPad_left"]:
            print("DPad Left pressed" if DPad_left == 1 else "DPad Left released")
            changes_detected = True

        if DPad_right != prev_values["DPad_right"]:
            print("DPad Right pressed" if DPad_right == 1 else "DPad Right released")
            changes_detected = True

        for i in range(buttons):
            if current_buttons[i] != prev_values["buttons"][i]:
                print(f"Button {i} {'pressed' if current_buttons[i] == 1 else 'released'}")
                changes_detected = True

        if changes_detected:
            print()  # Add a blank line between different changes

        # Update previous values
        prev_values["left_stick_x"] = left_stick_x
        prev_values["left_stick_y"] = left_stick_y
        prev_values["right_stick_x"] = right_stick_x
        prev_values["right_stick_y"] = right_stick_y
        prev_values["LT"] = LT
        prev_values["RT"] = RT
        prev_values["DPad_up"] = DPad_up
        prev_values["DPad_down"] = DPad_down
        prev_values["DPad_left"] = DPad_left
        prev_values["DPad_right"] = DPad_right
        prev_values["buttons"] = current_buttons
        # msg = String()
        left_stick_x = int((left_stick_x + 1) * 500 + 1000)
        left_stick_y = int((left_stick_y*-1 + 1) * 500 + 1000)
        right_stick_x = int((right_stick_x + 1) * 500 + 1000)
        right_stick_y = int((right_stick_y*-1 + 1) * 500 + 1000)
        (left_motor, right_motor) = calculate_motor_speeds(right_stick_x,right_stick_y)
        print(f"Left Motor: {left_motor}; Right Motor: {right_motor}")
        LT = (((LT + 1) * (1000 - 1500)) / -2) + 1500

        if(LT>1500):
            diff = LT-1500
            LT = 1500 - diff
        elif(LT<1500):
            diff = 1500-LT
            LT = 1500 + diff
        LT = int(LT)
        # map RT from -1 to 1 to 1500-2000
        RT = int((((RT + 1) * (2000 - 1500)) / 2) + 1500)

        if(LT!=1500):
            RT = LT

        DPad_up = int((DPad_up + 1) * 500 + 1000)
        DPad_down = int((DPad_down + 1) * 500 + 1000)
        DPad_left = int((DPad_left + 1) * 500 + 1000)
        DPad_right = int((DPad_right + 1) * 500 + 1000)
        for i in range(11):
            current_buttons[i] = 2000 if current_buttons[i]==1 else 1000
        # msg.data = ','.join([str(left_stick_x), str(left_stick_y), str(right_stick_x), str(right_stick_y), str(current_buttons[0]), str(current_buttons[1]), str(current_buttons[2]), str(current_buttons[3]), str(current_buttons[4]), str(current_buttons[5]), str(current_buttons[6]), str(current_buttons[7]), str(current_buttons[8]), str(current_buttons[9]), str(current_buttons[10]), str(DPad_left), str(DPad_right), str(DPad_up), str(DPad_down), str(LT), str(RT)])
        # print(msg.data)
        # pub.publish(str(left_motor)+","+str(right_motor))
        (left_motor1, right_motor1) = calculate_motor_speeds(left_stick_x,left_stick_y)
        s = str(left_motor)+","+str(right_motor)+","+str(left_motor1)+","+str(right_motor1)+","+str(left_stick_x)+","+str(left_stick_y)+","+str(right_stick_x)+","+str(right_stick_y)+","+str(DPad_up)+","+str(DPad_down)+","+str(DPad_left)+","+str(DPad_right)+","+str(LT)+","+str(RT)+","+str(current_buttons[0])+","+str(current_buttons[1])+","+str(current_buttons[2])+","+str(current_buttons[3])+","+str(current_buttons[4])+","+str(current_buttons[5])+","+str(current_buttons[6])+","+str(current_buttons[7])+","+str(current_buttons[8])+","+str(current_buttons[9])+","+str(current_buttons[10])
        # s = str(23)+s
        sio.emit('joystick_data', s)
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