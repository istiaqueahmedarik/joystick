# import cv2
# import numpy as np
# import base64
# from PIL import Image
# from io import BytesIO

# def capture_and_convert_to_base64():
#     # Initialize the webcam
#     cap = cv2.VideoCapture(0)

#     if not cap.isOpened():
#         print("Error: Could not open webcam.")
#         return None

#     # Capture a single frame
#     ret, frame = cap.read()
#     cap.release()

#     if not ret:
#         print("Error: Could not read frame from webcam.")
#         return None

#     # Convert the frame from BGR to RGB (OpenCV uses BGR by default)
#     frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

#     # Convert to PIL Image
#     image = Image.fromarray(frame_rgb)

#     # Save image to BytesIO stream in PNG format
#     buffered = BytesIO()
#     image.save(buffered, format="PNG")

#     # Encode image to base64
#     img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')

#     return img_base64

# # Capture image and get the base64 string
# img_base64 = capture_and_convert_to_base64()
# if img_base64:
#     print("Image as base64:")
#     print(img_base64)


import cv2
import numpy as np
import base64
from PIL import Image
from io import BytesIO
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS



app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*')

def capture_and_convert_to_base64():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return None

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Error: Could not read frame from webcam.")
        return None

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(frame_rgb)

    buffered = BytesIO()
    image.save(buffered, format="PNG")

    img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
    return img_base64



@socketio.on('capture_image')
def handle_capture_image():
    img_base64 = capture_and_convert_to_base64()
    if img_base64:
        emit('image_data', {'image': img_base64})

if __name__ == '__main__':
    socketio.run(app,port=5555,host='0.0.0.0',debug=True,use_reloader=True,log_output=True)
