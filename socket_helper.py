import socket
from time import sleep

IP = "192.168.85.42"
PORT = 8889
sock = None

def initialize_socket():
    global sock

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(4)
    sock.bind(("", PORT))
    return sock

# RMTT needs to receive commands to stay alive
def keep_drone_alive():
    while True:
        send_command("command", True)
        sleep(10)

def send_command(command: str, print_command: bool = None):
    try:
        sock.sendto(command.encode("utf-8"), (IP, PORT))
        if print_command:
            print(f"sent command: {command} to {IP, PORT}")
    except Exception as e:
        print(f"send_command() error: {e}")