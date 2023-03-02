#!/usr/bin/env python3
"""
Copyright 2019, David Pierce Walker-Howell, All rights reserved
Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
Last Modified: 06/22/2019
Description: This module is the driver program for the Sparton AHRS.
Update 5/29/2022: Code being re-used for Scion. Modified from original source of mechatronics-2019 repo. No longer using
mechos and using a ROS driver instead for communication. Will therefore be imported by the ROS driver.
"""
import sys
import serial
import time
import struct

class SpartonAHRSDataPackets:
    """
    This class implements the getters and setters commands for the Sparton AHRS.
    It handles the sending values to configure the device and request data specified
    data.
    """
    def __init__(self, _com_port="/dev/ttyUSB0"): #ftdi, sometimes this changes to USB0 etc etc, in terminal type "sudo dmesg | grep -i serial"
        """
        Initializes connection with the AHRS device at a baud rate of 115200.
        Sets up the array of location values.
        Parameters:
            _com_port: A string value of the serial COM port that the AHRS
                            device is connected to.
        Returns:
            N/A
        """

        self.ahrs_serial = serial.Serial(_com_port, 115200)
        self.success_header_byte = 0xA4
        self.error_header_byte = 0xAE
        self.termination_byte = 0xA0

        # Refer to page 38-43 for more information about locationarray
        self.location_array = {"raw_magnetics": [0x01, 9], "true_heading": [0x02, 5],
                               "magnetic_heading": [0x09, 5], "magnetic_variation": [0x83, 5],
                               "auto_magnetic_variation": [0x0F, 5], "latitude": [0x8B, 5],
                               "longitude": [0x8C, 5], "altitude": [0x8D, 5], "day": [0x8E, 5],
                               "magnetic_vector": [0x04, 11], "raw_acceleration": [0x05, 9],
                               "pitch_roll": [0x06, 7], "accleration_vector": [0x07, 11],
                               "temperature": [0x11, 5], "baud_rate": [0x57, 4],
                               "mounting_config": [0x4A, 4]}

    def get_raw_magnetics(self):
        """
        Send request to get the raw magnetics from the magnetometers (Mx, My,
        and Mz). These are raw sensor readings and do not yet have any
        calibration. Note that this function is really only used for test purposes.
        NOT PRACTICAL FOR OFFICIAL USE.
        Reference: Software-Interface-Manual page 38
         Send: 3 Byte (0xA4,0x01,0xA0)
        Response: 9 Bytes (0xA4,0x01,<Mx>,<My>,<Mz as 16-bit integers MS
         byte first>,0xA0)
         Parameters:
            N/A
        Returns:
            raw_magnetics: 6-byte list of raw magnetics.
        """
        self.ahrs_serial.write(bytearray([0xA4, 0x01, 0xA0]))
        return self._unpack("raw_magnetics")

    def get_true_heading(self):
        """
        Send request to get the true heading. True heading is the magnetic
        heading.
        Reference: Software-Interface-Manual page 38.
        Send: 3 Bytes (0xA4, 0x02, 0xA0)
        Responds: 5 Bytes <0xA4, 0x02, <Heading as a 16-bit signed integer>,
                            0xA0)
        Heading (degrees) = 16-bit Heading value * (360/4096)
        Heading Range = 0.0 to + 359.9
        Returns:
            true_heading: The true heading in degrees from 0.0 to 359.9. If the
                            data could not be received, return an empty list.
        """
        self.ahrs_serial.write(bytearray([0xA4, 0x02, 0xA0]))
        true_heading = self._unpack("true_heading")
        if true_heading is None:
            return None
        if len(true_heading) == 2:
            # convert the raw true heading data into degree
            true_heading = struct.pack('H', (true_heading[0] << 8) | (true_heading[1]))
            true_heading = struct.unpack('h', true_heading)[0]*(360.0/4096.0)
            return true_heading
        return None

    def get_pitch_roll(self):
        """
        Send request and receive data to get the pitch and roll of the platform.
        Reference: Sofware-Inference-Manual page 42.
        Send: 3 Bytes (0xA4, 0x06, 0xA0)
        Response: 7 Bytes (0xA4, 0x06, Pitch, Roll as 16-bit signed integers, 0xA0)
        Pitch(in degrees) = (Response) * 90/4096
        Pitch Range: -90 to +90
        Roll (in degrees) = (Response) * 180/4096
        Acceleration Vector Roll Range = -180 to 180
        Returns:
            pitch_roll: A list containing the pitch and roll
        """
        self.ahrs_serial.write(bytearray([0xA4, 0x06, 0xA0]))
        pitch_roll = self._unpack("pitch_roll")
        if pitch_roll is None:
            return [None, None]
        if len(pitch_roll) == 4:
            # structs are used to make pitch signed
            pitch = struct.pack('H', (pitch_roll[0] << 8) | pitch_roll[1])
            pitch = struct.unpack('h', pitch)[0] * (90.0/4096.0)

            roll = struct.pack('H', (pitch_roll[2] << 8) | pitch_roll[3])
            roll = struct.unpack('h', roll)[0] * (180.0/4096.0)
            return [pitch, roll]
        return [None, None]

    def _unpack(self, data_type):
        """
        Read in the transmission from AHRS and extract all the bytes of the
        packet.
        Parameters:
            data_type: The type of data being read. Note: This should be one of
                    the keys in the locationArray(see constructor)
        Returns:
            ahrs_data_in: The raw data packet of the given data type. If there is
                        an error reading the data, return None. If there are no
                        datapacket to be read, return an empty list of data.
        """
        # Read incoming data bytes
        ahrs_data_in = []
        if self.ahrs_serial.in_waiting > 0:
            # Read in header byte
            header_byte = ord(self.ahrs_serial.read())
            # Successful header byte
            if header_byte == self.success_header_byte:
                # Read second byte to confirm successful packet
                type_byte = ord(self.ahrs_serial.read())
                if type_byte == self.location_array[data_type][0]:
                    # Read the given number of data bytes specified by location array
                    for idx in range(0, self.location_array[data_type][1] - 2):
                        ahrs_data_in.append(ord(self.ahrs_serial.read()))
                    # Confirm correct termination of data packet
                    if ahrs_data_in[-1] != self.termination_byte:
                        return None
            # Error header byte
            elif header_byte == self.error_header_byte:
                return None
        # Note: Don't include the header or termination bytes in the raw data
        return ahrs_data_in[:-1]


def main(com_port):
    """Test Driver for basic AHRS functionality.
    """
    ahrs = SpartonAHRSDataPackets(_com_port=com_port)
    print('Post connect')
    while True:
        yaw = ahrs.get_true_heading()
        pitch, roll = ahrs.get_pitch_roll()
        if yaw is not None:
            print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
        time.sleep(.01)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ahrs_dev_port = sys.argv[1].replace(' ', '')
        main(com_port=ahrs_dev_port)
    else:
        print("Error: not enough arguments. (Was the ahrs dev port passed?)")
        sys.exit(1)
