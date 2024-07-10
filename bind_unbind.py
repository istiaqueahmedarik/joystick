import subprocess

def bind_teensy(device):
    bind_command = f"echo {device} > /sys/bus/usb/drivers/usb/bind"
    subprocess.run(bind_command, shell=True)

def unbind_teensy(device):
    unbind_command = f"echo {device} > /sys/bus/usb/drivers/usb/unbind"
    subprocess.run(unbind_command, shell=True)

# Bind the Teensy device
bind_teensy("/dev/ACM0")
print("Teensy device bound successfully.")

# Unbind the Teensy device
unbind_teensy("/dev/ACM0")
print("Teensy device unbound successfully.")

# Bind the Teensy device again
bind_teensy("/dev/ACM0")
print("Teensy device bound again successfully.")