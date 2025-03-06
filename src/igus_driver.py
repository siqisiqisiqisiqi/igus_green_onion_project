import numpy as np


class IgusDriverEncoder():
    def cartesian_move(self, position, yaw, vel=100):
        """_summary_

        Parameters
        ----------
        position : array
            desired position in mm
        yaw : float
            desired orientation in degree
        vel : int, optional
            robot velocity mm/s, by default 500

        Returns
        -------
        _type_
            _description_
        """
        x = int(position[0])
        y = int(position[1])
        z = int(position[2])
        yaw = -1 * yaw + 115
        # constranit the work range
        if z < 75:
            z = 75
            print("z exceed the range!!!")
        if z > 300:
            z = 300
            print("z exceed the range!!!")
        if abs(x) > 270:
            x = np.sign(x) * 270
            print("x exceed the range!!!")
        if abs(y) > 250:
            y = np.sign(y) * 250
            print("y exceed the range!!!")
        message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 {yaw} 0 0 {vel} CRIEND"
        # message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 0 0 0 200 CRIEND"
        encoded = message.encode('utf-8')
        move_array = bytearray(encoded)
        return message

    def gripper(self, D20: int = None, D21: int = None):
        if D20 != None:
            if D20 == 0:
                message = "CRISTART 1234 CMD DOUT 20 false CRIEND"
            else:
                message = "CRISTART 1234 CMD DOUT 20 true CRIEND"
            encoded = message.encode('utf-8')
            Dout_array = bytearray(encoded)
            return message

        elif D21 != None:
            if D21 == 0:
                message = "CRISTART 1234 CMD DOUT 21 false CRIEND"
            else:
                message = "CRISTART 1234 CMD DOUT 21 true CRIEND"
            encoded = message.encode('utf-8')
            Dout_array = bytearray(encoded)
            return message

        else:
            return None
        
    def conveyor(self, D22: int = 0):
        if D22 == 0:
            message = "CRISTART 1234 CMD DOUT 22 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 22 true CRIEND"
        return message
