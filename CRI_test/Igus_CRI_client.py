import socket
import re
from numpy.linalg import norm
import numpy as np
import time
from threading import Thread, Event

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Enter the IP address of the robot (192.168.3.11) here if you're not using a CPRog/iRC simulation
# server_address = ('localhost', 3920)
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

        # cartesian position
        self.cartesian_position = None
        # received extra signal
        self.yaw = None
        # digital input
        self.din = []

    def run(self):
        global sock
        i_f = 0
        while True:
            i_f = i_f + 1
            if i_f > 20:
                sock.sendall(self.arrayAliveJog)
                i_f = 0
            data = sock.recv(1024).decode()
            # print(data)
            try:
                result1 = re.findall(r'DIN \d+', data, re.DOTALL)
                a = result1[0].split()
                din = int(a[1])
                self.din = [i + 1 for i,
                            j in enumerate(bin(din)[:1:-1]) if j == "1"]
            except:
                pass

            try:
                result2 = re.findall(
                    r'POSCARTROBOT(?: \-?\d+\.?\d+){3}', data, re.DOTALL)
                b = result2[0].split()
                self.cartesian_position = [float(ele) for ele in b[1:]]
            except:
                pass
            try:
                result3 = re.findall(
                    r'POSJOINTCURRENT(?: \-?\d+\.?\d+){9}', data, re.DOTALL)
                b = result3[0].split()
                self.yaw = -1* float(b[7]) + 118
                # self.yaw = 30 - float(b[7]) * 10
            except:
                pass
            # shutdown the thread
            if self.event.is_set():
                break
            # TODO search the website to figure out the synchronization between client and server
            time.sleep(0.02)


def cartesian_move_array(position):
    x = position[0]
    y = position[1]
    z = position[2]
    yaw = -1 * position[3] + 118
    message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 {yaw} 0 0 200 CRIEND"
    encoded = message.encode('utf-8')
    move_array = bytearray(encoded)
    return move_array


def gripper(sock, D20: int = None, D21: int = None):
    if D20 != None:
        if D20 == 0:
            message = "CRISTART 1234 CMD DOUT 20 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 20 true CRIEND"
        print(message)
        encoded = message.encode('utf-8')
        Dout_array = bytearray(encoded)
        sock.sendall(Dout_array)

    elif D21 != None:
        if D21 == 0:
            message = "CRISTART 1234 CMD DOUT 21 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 21 true CRIEND"
        print(message)
        encoded = message.encode('utf-8')
        Dout_array = bytearray(encoded)
        sock.sendall(Dout_array)


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
    time.sleep(60)
    print("finish the robot referencing.")


def close_machine():
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


def main():
    start_machine()
    stop_event = Event()
    alive_client = AliveDecode(stop_event)

    # the position is [x(mm), y(mm), z(mm), theta(degree)]
    home = [0, 0, 300, 0]
    position = []
    position1 = [0, 0, 300, 0]
    position2 = [0, 0, 300, 0]
    # position3 = [0, 0, 300, 0]

    position.append(position1)
    position.append(position2)
    # position.append(position3)

    alive_client.start()

    array = cartesian_move_array(home)
    sock.sendall(array)

    time.sleep(2)
    print("Start the project!")

    actual_pos = alive_client.cartesian_position
    actual_yaw = alive_client.yaw
    while actual_pos is None or actual_yaw is None:
        time.sleep(0.1)
        actual_pos = alive_client.cartesian_position
        actual_yaw = alive_client.yaw
        print(f"actual_pos is {actual_pos}")

    # time.sleep(1)
    # print("Blowoff ON")
    # gripper(sock, D20=1)
    # time.sleep(5)
    # print("Blowoff OFF")
    # gripper(sock, D20=0)
    # time.sleep(1)
    # print("Suction ON")
    # gripper(sock, D21=1)
    # time.sleep(5)
    # print("Suction OFF")
    # gripper(sock, D21=0)
    # time.sleep(1)

    t = time.time()
    for i in range(len(position)):

        array = cartesian_move_array(position[i])
        sock.sendall(array)

        time.sleep(0.2)
        actual_pos = alive_client.cartesian_position
        actual_yaw = alive_client.yaw
        actual_pos = [*actual_pos, actual_yaw]

        while norm(np.array(actual_pos) - np.array(position[i])) > 1:
            actual_pos = alive_client.cartesian_position
            actual_yaw = alive_client.yaw
            actual_pos = [*actual_pos, actual_yaw]
            print(f"actual position is {actual_pos}")
            time.sleep(0.05)
        time.sleep(1)
    print(f"elapsed time is {time.time()-t}.")
    print("Finally")

    stop_event.set()
    alive_client.join()
    close_machine()
    sock.close()
    time.sleep(1)
    print("shutdown everything")
    exit()


if __name__ == "__main__":
    main()
