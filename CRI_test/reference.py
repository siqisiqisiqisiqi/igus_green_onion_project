import socket
import select
import re
import sys
import time
from threading import Thread, Event

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Enter the IP address of the robot (192.168.3.11) here if you're not using a CPRog/iRC simulation
server_address = ('192.168.3.11', 3920)
print("Connecting...")
sock.connect(server_address)
print("Connected")


class AliveDecode(Thread):
    def __init__(self, event):
        super(AliveDecode, self).__init__()

        self.event = event
        # The ALIVEJOG message needs to be sent regularly (at least once a second) to keep the connection alive
        messageAliveJog = "CRISTART 1234 ALIVEJOG 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND"
        encodedAliveJog = messageAliveJog.encode('utf-8')
        self.arrayAliveJog = bytearray(encodedAliveJog)
        self.cartesian_position = None
        # digital input
        self.din = []

    def run(self):
        global sock
        while True:
            sock.sendall(self.arrayAliveJog)
            data = sock.recv(1024).decode()
            # print(f"data is {data}.")

            try:
                result1 = re.findall(r'DIN \d+', data, re.DOTALL)
                result2 = re.findall(
                    r'POSCARTROBOT(?: \d+\.?\d+){3}', data, re.DOTALL)

                a = result1[0].split()
                din = int(a[1])
                self.din = [i + 1 for i,
                            j in enumerate(bin(din)[:1:-1]) if j == "1"]

                b = result2[0].split()
                self.cartesian_position = [float(ele) for ele in b[1:]]
            except:
                pass

            if self.event.is_set():
                break

            time.sleep(0.05)


stop_event = Event()
alive_client = AliveDecode(stop_event)


def start_machine():
    messageConnect = "CRISTART 1234 CMD Connect CRIEND"
    messageReset = "CRISTART 1234 CMD Reset CRIEND"
    messageEnable = "CRISTART 1234 CMD Enable CRIEND"
    encodedConnect = messageConnect.encode('utf-8')
    encodedReset = messageReset.encode('utf-8')
    encodedEnable = messageEnable.encode('utf-8')
    array = bytearray(encodedConnect)
    sock.sendall(array)
    time.sleep(1)
    array = bytearray(encodedReset)
    sock.sendall(array)
    time.sleep(1)
    array = bytearray(encodedEnable)
    sock.sendall(array)
    time.sleep(1)
    print("start the machine")


def reference_machine():
    messageReference = "CRISTART 1234 CMD ReferenceAllJoints CRIEND"
    encodedReference = messageReference.encode('utf-8')
    array = bytearray(encodedReference)
    sock.sendall(array)
    print("start reference the robot.")
    time.sleep(90)
    print("finish the robot referencing.")


def close_machine():

    Dout22 = "CRISTART 1234 CMD DOUT 22 false CRIEND"
    Dout21 = "CRISTART 1234 CMD DOUT 21 false CRIEND"
    Dout22 = Dout22.encode('utf-8')
    array = bytearray(Dout22)
    sock.sendall(array)
    time.sleep(0.5)
    Dout21 = Dout21.encode('utf-8')
    array = bytearray(Dout21)
    sock.sendall(array)

    messageDisconnect = "CRISTART 1234 CMD Disconnect CRIEND"
    messageDisable = "CRISTART 1234 CMD Disable CRIEND"

    encodedDisconnect = messageDisconnect.encode('utf-8')
    encodedDisable = messageDisable.encode('utf-8')
    array = bytearray(encodedDisconnect)
    sock.sendall(array)
    time.sleep(1)
    array = bytearray(encodedDisable)
    sock.sendall(array)
    time.sleep(1)
    print("close the machine")


try:
    start_machine()
    print("Keeping connection alive")
    alive_client.start()
    reference_machine()

finally:
    print("Finally")
    close_machine()
    stop_event.set()
    alive_client.join()
    sock.close()
    time.sleep(0.5)

    exit()
