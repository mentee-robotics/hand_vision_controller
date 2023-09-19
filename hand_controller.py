import serial
import time
import struct
import random
from enum import Enum

class ControlLoop(Enum):
    POS_PID = 1
    VEL_PID = 2
    IMPEDANCE = 3
    CALIBRATE_UP = 4
    CALIBRATE_DOWN = 5
    CHECK_CALIBRATION = 6


HEADER = 200
TAIL = 199

RIGHT_HAND = 0XD2
LEFT_HAND = 0XD7



class SerialCommunication:
    def __init__(self):
        self.serial_port = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

    def send_action_to_hand(self, hand_id, thumb_pos1, thumb_pos2, index_pos, middle_pos, ring_pos, pinky_pos):
        packet_to_send = bytearray(32)
        
        packet_to_send[0:2] = struct.pack('H', hand_id)
        packet_to_send[2] = HEADER
        packet_to_send[3] = HEADER
        packet_to_send[4] = 24  # 6 floats * 4 bytes each
        
        packet_to_send[5:9] = struct.pack('f', thumb_pos1)
        packet_to_send[9:13] = struct.pack('f', thumb_pos2)
        packet_to_send[13:17] = struct.pack('f', index_pos)
        packet_to_send[17:21] = struct.pack('f', middle_pos)
        packet_to_send[21:25] = struct.pack('f', ring_pos)
        packet_to_send[25:29] = struct.pack('f', pinky_pos)
        
        # TODO: Calculate the checksum properly
        packet_to_send[30] = 11  # Checksum
        packet_to_send[31] = TAIL
        
        self.serial_port.write(packet_to_send)  # Send the packet to the serial port

    def send_control_packet(self, hand_id, kp, kd, control_loop: ControlLoop ):
        # Each float is 4 bytes, plus bytes for the other fields.
        packet_to_send = bytearray(16)
        
        motor_id_bytes = struct.pack('H', hand_id)
        packet_to_send[0:2] = motor_id_bytes
        packet_to_send[2] = HEADER
        packet_to_send[3] = HEADER
        packet_to_send[4] = 10  # 2 floats * 4 bytes each + 2 bytes
        
        # Convert the float values to bytes and add them to the packet
        packet_to_send[5:9] = struct.pack('f', kp)
        packet_to_send[9:13] = struct.pack('f', kd)
        packet_to_send[13] = control_loop.value
        
        # TODO: Calculate the checksum properly
        packet_to_send[14] = 11  # Checksum
        packet_to_send[15] = TAIL
        
        self.serial_port.write(packet_to_send)  # Send the packet to the serial port

    def receive_data(self):
        data_array = []
        data = self.serial_port.read(14) # need to change the size to the correct value
        if len(data)> 13:
            for i in range(len(data)):
                data_array.append(data[i])

            return data_array
        

if __name__ == "__main__":
    serial_comm = SerialCommunication()
    while True:
        pass
        


