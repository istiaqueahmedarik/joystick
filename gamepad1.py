import asyncio
import websockets
import pygame
import keyboard
import zlib
import time

def compress_string(string):
    compressed_data = zlib.compress(string.encode(), 9)
    return compressed_data

def decompress_string(compressed_data):
    decompressed_string = zlib.decompress(compressed_data).decode()
    return decompressed_string

DEADZONE = 50

async def emit_with_retry(websocket, event, message, max_retries=30, retry_delay=1):
    attempt = 0
    while attempt < max_retries:
        try:
            await websocket.send(compress_string(message))
            break  # If send succeeds, break out of the loop
        except Exception as e:
            print(f"Error: {e}. Retrying...")
            await asyncio.sleep(retry_delay)  # Wait before retrying
            attempt += 1
    if attempt == max_retries:
        print("Failed to send after several retries.")

def joystick_to_motor_speed(x, y):
    x_norm = (x - 1500) / 500
    y_norm = (y - 1500) / 500

    left_speed_norm = y_norm + x_norm
    right_speed_norm = y_norm - x_norm

    left_speed = 1500 + 500 * left_speed_norm
    right_speed = 1500 + 500 * right_speed_norm

    left_speed = max(1000, min(2000, left_speed))
    right_speed = max(1000, min(2000, right_speed))

    return int(left_speed), int(right_speed)

async def joystick():
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

    async with websockets.connect('ws://192.168.1.100:5476') as websocket:
        while True:
            l = False
            if keyboard.is_pressed('l'):
                l = True
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

            leftY = int((leftY + 1) * 500 + 1000)
            leftX = int((leftX + 1) * 500 + 1000)
            rightY = int((rightY + 1) * 500 + 1000)
            rightX = int((rightX + 1) * 500 + 1000)
            arm = int((arm + 1) * 500 + 1000)
            base = int((base + 1) * 500 + 1000)
            speed_mode = int((speed_mode + 1) * 500 + 1000)
            lifter = int((lifter + 1) * 500 + 1000)
            lifter_mode = int((lifter_mode + 1) * 500 + 1000)
            light = int((l + 1) * 500 + 1000)
            gripper_of = int((1 - gripper_of) * 500 + 1000)
            gripper_on = int(gripper_on * 500 + 1500)
            gripper = gripper_on
            if gripper_of != 1500:
                gripper = gripper_of

            (leftMotor, rightMotor) = joystick_to_motor_speed(rightX, rightY)

            s = "["
            s += str(leftMotor) + ","
            s += str(rightMotor) + ","
            s += str(leftX) + ","
            s += str(leftY) + ","
            s += str(base) + ","
            s += str(lifter) + ","
            s += str(gripper) + ","
            s += str(lifter_mode) + ","
            s += str(speed_mode) + ","
            s += str(arm)
            s += "]"

            print(arm)
            await emit_with_retry(websocket, 'armMsg', 'noarm' if arm == 2000 else 'arm')
            if arm == 2000:
                await emit_with_retry(websocket, 'joystick_data', s)

            time.sleep(0.3)

if __name__ == "__main__":
    asyncio.run(joystick())
