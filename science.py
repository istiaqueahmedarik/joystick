import subprocess
import time
import os
import signal

def run_command(command):
    return subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

def is_roscore_running():
    try:
        # Attempt to communicate with the ROS master
        output = subprocess.check_output(["rosnode", "list"], stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError:
        return False

def kill_server():
    try:
        # Find the PID(s) of the running server.py process
        pids = subprocess.check_output(["pgrep", "-f", "python sensor.py"], stderr=subprocess.STDOUT).decode().split()
        for pid in pids:
            os.kill(int(pid), signal.SIGTERM)
            print(f"Killed server.py with PID {pid}")
    except subprocess.CalledProcessError:
        # server.py is not running
        pass

def terminate_process(process):
    if process and process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)

# Kill existing server.py process if running
kill_server()

# Start roscore
roscore_process = run_command("roscore")

# Wait until roscore is fully running
print("Waiting for roscore to start...")
while not is_roscore_running():
    time.sleep(1)
print("roscore started.")

# Start the other commands
socket_service = run_command("roslaunch rosbridge_server rosbridge_websocket.launch")
sensor = run_command("python sensor.py")
head = run_command("python gpsHead.py")

# Keep the script running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Terminate processes if the script is stopped
    terminate_process(roscore_process)
    terminate_process(sensor)
    terminate_process(head)