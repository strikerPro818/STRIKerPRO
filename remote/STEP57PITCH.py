# RMD-X8 Python Library
# Copyright 2022 Sanjay Sunil

import can
import os
import time


class STEP57PITCH:
    """
    A class to read and write on the RMD-X8 motor.

    ...

    Attributes
    ----------
    bus : type
        the can bus channel used to communicate with the motor
    identifier : type
        the motor's identifier on the can bus

    Methods
    -------
    setup():
        Setup the can bus connection.
    send_cmd(data, delay):
        Send a frame data to the motor.
    read_pid():
        Read the motor's current PID parameters.
    write_pid_ram(data):
        Write PID parameters to the RAM.
    write_pid_rom(data):
        Write PID parameters to the ROM.
    read_acceleration():
        Read the motor's acceleration data.
    write_acceleration_ram(data):
        Write the acceleration to the RAM of the motor.
    read_encoder():
        Read the current position of the encoder.
    write_encoder_offset(data):
        Set the motor's encoder offset.
    write_motor_zero_rom():
        Write the current position of the motor to the ROM
        as the motor zero position.
    read_multi_turns_angle():
        Read the multi-turn angle of the motor.
    read_single_turn_angle():
        Read the single circle angle of the motor.
    motor_off():
        Turn off the motor, while clearing the motor operating
        status and previously received control commands.
    motor_stop():
        Stop the motor, but do not clear the operating state and
        previously received control commands.
    motor_running():
        Resume motor operation from the motor stop command.
    read_motor_status_1():
        Reads the motor's error status, voltage, temperature and
        other information.
    read_motor_status_2():
        Reads the motor temperature, voltage, speed and encoder
        position.
    read_motor_status_3():
        Reads the phase current status data of the motor.
    clear_motor_error_flag():
        Clears the error status of the currrent motor.
    torque_closed_loop(data):
        Control torque current output of the motor.
    speed_closed_loop(data):
        Control the speed of the motor.
    position_closed_loop_1(data):
        Control the position of the motor (multi-turn angle).
    position_closed_loop_2(data):
        Control the position of the motor (multi-turn angle).
    position_closed_loop_3(data):
        Control the position of the motor (single-turn angle).
    position_closed_loop_4(data):
        Control the position of the motor (single-turn angle).
    """

    def __init__(self, identifier):
        """
        Constructs all the necessary attributes for the RMDX8 object.
        """
        self.bus = None
        self.identifier = identifier

    def setup(self):
        """
        Setup the can bus connection.

        Returns
        -------
        self.bus : type
            The bus used to communicate with the motor.
        """
        try:
            # os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
            time.sleep(0.1)
        except Exception as e:
            print(e)

        try:
            bus = can.interface.Bus(bustype='socketcan', channel='can0')
        except OSError:
            print('err: PiCAN board was not found.')
            exit()
        except Exception as e:
            print(e)

        print("Bus:",bus)
        self.bus = bus
        return self.bus

    def send_cmd(self, data, delay=1, identifier=None):
        """
        Send frame data to the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.
        delay : int/float
            The time to wait after sending data to the motor.
        identifier : int, optional
            The identifier to use for the CAN frame.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        if identifier is None:
            identifier = self.identifier

        message = can.Message(arbitration_id=identifier, data=data, is_extended_id=True)
        self.bus.send(message)
        print(message)

        # received_message = self.bus.recv(timeout=0.01)
        # if received_message is None:
        #     print("No message received within the specified timeout.")
        # else:
        #     # print()
        #     print(received_message)
        #     # print()
        #
        # return received_message


    def read_step_pid(self):
        """
        Read the motor's current PID parameters.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x21, 0x6B]
        return self.send_cmd(message, 0.01)


    def write_pid_ram(self, data):
        """
        Write PID parameters to the RAM.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x31, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.01)

    def write_pid_rom(self, data):
        """
        Write PID parameters to the ROM.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x32, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.01)

    def read_acceleration(self):
        """
        Read the motor's acceleration data.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def write_acceleration_ram(self, data):
        """
        Write the acceleration to the RAM of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x34, 0x00, 0x00, 0x00,
                   data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.01)

    def read_encoder(self):
        """
        Read the current position of the encoder.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def write_encoder_offset(self, data):
        """
        Set the motor's encoder offset.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x91, 0x00, 0x00, 0x00,
                   0x00, 0x00, data[0], data[1]]
        return self.send_cmd(message, 0.01)

    def write_motor_zero_rom(self):
        """
        Write the current position of the motor to the ROM as the motor zero position.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x19, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def read_multi_turns_angle(self):
        """
        Read the multi-turn angle of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x92, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.5)

    def read_multi_turns_angle_original(self):
        """
        Read the multi-turn angle of the motor.

        Returns
        -------
        angle : float
            The actual angle of the motor in degrees.
        """
        message = [0x92, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        received_message = self.send_cmd(message, 0.5)

        # Extract the angle from the received_message using indices 4 to 7 (inclusive)
        raw_angle = int.from_bytes(received_message[4:8], byteorder='little', signed=True)

        # Convert the raw angle to degrees by multiplying with 0.01
        angle = raw_angle * 0.01

        return angle

    def read_single_turn_angle(self):
        """
        Read the single circle angle of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x94, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def motor_off(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xFF, 0xFF, 0xFF, 0xFF,
                   0xFF, 0xFF, 0xFF, 0xFD]
        return self.send_cmd(message, 0.01)
    def step_calibreate(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x06, 0x45, 0x6B]
        return self.send_cmd(message, 0.01)

    def step_on(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xF3, 0xAB, 0x01, 0x00, 0x6B]
        return self.send_cmd(message, 0.01)

    def step_off(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xF3, 0x00, 0x6B]
        return self.send_cmd(message, 0.01)

    def step_zero(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x0A, 0x6D, 0x6B]
        return self.send_cmd(message, 0.01)

    def step_unlock(self):
        """
        Turn off the motor, while clearing the motor operating status and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x0E, 0x52, 0x6B]
        return self.send_cmd(message, 0.01)

    def step_test(self):
        """
        Stop the motor, but do not clear the operating state and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xFD, 0x14, 0xFF, 0x00,
                   0x00, 0x0C, 0x80, 0x6B]
        return self.send_cmd(message, 0.01)

    def step_position(self, position, speed):
        """
        Sends a command to control the motor in direct position limit mode.

        Parameters
        ----------
        direction : int
            Direction of movement (1 for positive, 0 for negative).
        speed : float
            Speed of movement in RPV (Rotations per Minute).
        position : float
            Position to reach in degrees.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        if position >= 0:
            direction = 0
        else:
            direction = 1

        speed_bytes = int(speed * 10).to_bytes(2, 'big')  # Convert speed to 10 times the value as per example
        # position_bytes = int(position * 10).to_bytes(4, 'big')  # Convert position to 10 times the value as per example
        position_bytes = int(abs(position) * 10).to_bytes(4, 'big', signed=True)


        message1 = [0xFB, direction, speed_bytes[0], speed_bytes[1], position_bytes[0], position_bytes[1], position_bytes[2]]
        message2 = [0xFB, position_bytes[3], 0x01, 0x00, 0x6B]

        # Print the messages in hexadecimal format
        hex_message1 = ' '.join(f'{byte:02X}' for byte in message1)
        hex_message2 = ' '.join(f'{byte:02X}' for byte in message2)
        print(hex_message1)
        print(hex_message2)

        self.send_cmd(message1)
        # print('here')
        self.send_cmd(message2, identifier=0x0201)

    def step_position_simple(self, position):
        """
        Sends a command to control the motor in direct position limit mode.

        Parameters
        ----------
        direction : int
            Direction of movement (1 for positive, 0 for negative).
        speed : float
            Speed of movement in RPV (Rotations per Minute).
        position : float
            Position to reach in degrees.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        if position >= 0:
            direction = 0
        else:
            direction = 1

        # speed_bytes = int(speed * 10).to_bytes(2, 'big')  # Convert speed to 10 times the value as per example
        # position_bytes = int(position * 10).to_bytes(4, 'big')  # Convert position to 10 times the value as per example
        position_bytes = int(abs(position) * 10).to_bytes(4, 'big', signed=True)


        message1 = [0xFA, direction, position_bytes[0], position_bytes[1], position_bytes[2]]
        message2 = [0xFA, position_bytes[3], 0x00, 0x00, 0x6B]

        # Print the messages in hexadecimal format
        hex_message1 = ' '.join(f'{byte:02X}' for byte in message1)
        hex_message2 = ' '.join(f'{byte:02X}' for byte in message2)
        print(hex_message1)
        print(hex_message2)

        self.send_cmd(message1, 0.01)
        # print('here')
        self.send_cmd(message2, 0.01,identifier=0x0101)
    def step_position_acdc(self, position, speed):
        """
        Sends a command to control the motor in direct position limit mode.

        Parameters
        ----------
        direction : int
            Direction of movement (1 for positive, 0 for negative).
        speed : float
            Speed of movement in RPV (Rotations per Minute).
        position : float
            Position to reach in degrees.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        if position >= 0:
            direction = 0
        else:
            direction = 1



        speed_bytes = int(speed * 10).to_bytes(2, 'big')  # Convert speed to 10 times the value as per example
        # position_bytes = int(position * 10).to_bytes(4, 'big')  # Convert position to 10 times the value as per example
        position_bytes = int(abs(position) * 10).to_bytes(4, 'big', signed=True)


        message1 = [0xFD, direction, speed_bytes[0], speed_bytes[1], position_bytes[0], position_bytes[1], position_bytes[2]]
        message2 = [0xFD, position_bytes[3], 0x00, 0x00, 0x6B]

        # Print the messages in hexadecimal format
        hex_message1 = ' '.join(f'{byte:02X}' for byte in message1)
        hex_message2 = ' '.join(f'{byte:02X}' for byte in message2)
        print(hex_message1)
        print(hex_message2)

        self.send_cmd(message1, 0.01)
        # print('here')
        self.send_cmd(message2, 0.01,identifier=0x0101)
    # def step_write_pid(self, position_P):
    #
    #
    #
    #     message1 = [0x4A, 0xC3, ]
    #     message2 = [0x4A, 0xC3, 0x6B]
    #
    #     # Print the messages in hexadecimal format
    #     hex_message1 = ' '.join(f'{byte:02X}' for byte in message1)
    #     hex_message2 = ' '.join(f'{byte:02X}' for byte in message2)
    #     print(hex_message1)
    #     print(hex_message2)
    #
    #     self.send_cmd(message1, 0.01)
    #     # print('here')
    #     self.send_cmd(message2, 0.01,identifier=0x0101)
    def step_write_pid(self, position_P,speed_P,speed_I):

        save_config = 0x00
        trapezoidal_mode_kp = 366640
        direct_mode_kp = position_P
        speed_loop_kp = speed_P
        speed_loop_Ri = speed_I

        save_config_bytes = save_config.to_bytes(1, 'big')
        trapezoidal_mode_kp_bytes = trapezoidal_mode_kp.to_bytes(4, 'big')
        direct_mode_kp_bytes = direct_mode_kp.to_bytes(4, 'big')
        speed_loop_kp_bytes = speed_loop_kp.to_bytes(4, 'big')
        speed_loop_Ri_bytes = speed_loop_Ri.to_bytes(4, 'big')


        # Create the base message components
        message1 = [0x4A, 0xC3]
        message2 = [0x4A]
        message3 = [0x4A]
        # message4 = [0x4A, 0xC3]


        message1 = message1 + list(save_config_bytes) + list(trapezoidal_mode_kp_bytes) + [direct_mode_kp_bytes[0]]

        # Construct message2 by adding position_P_bytes and checksum
        message2 = message2 + [direct_mode_kp_bytes[1]] + [direct_mode_kp_bytes[2]] + [direct_mode_kp_bytes[3]] + [speed_loop_kp_bytes[0]] + [speed_loop_kp_bytes[1]] + [speed_loop_kp_bytes[2]] + [speed_loop_kp_bytes[3]]
        message3 = message3 + list(speed_loop_Ri_bytes) + [0X6B]
        # message4 = message4 + list(speed_loop_Ri_bytes) + [0x6B]

        # Print the messages in hexadecimal format
        hex_message1 = ' '.join(f'{byte:02X}' for byte in message1)
        hex_message2 = ' '.join(f'{byte:02X}' for byte in message2)
        hex_message3 = ' '.join(f'{byte:02X}' for byte in message3)
        # hex_message4 = ' '.join(f'{byte:02X}' for byte in message4)


        print(hex_message1)
        print(hex_message2)
        print(hex_message3)
        print()
        # print(hex_message4)


        # Send the commands with appropriate delay and identifier
        self.send_cmd(message1, 0.01)
        self.send_cmd(message2, 0.01, identifier=0x0201)
        self.send_cmd(message3, 0.01, identifier=0x0202)
        # self.send_cmd(message4, 0.01, identifier=0x0103)




    def read_step_position(self):

        message = [0x36, 0x6B]
        return self.send_cmd(message, 0.01)

    def test_command(self):
        message = [0xFB, 0x01, 0x4E, 0x20, 0x00, 0x00, 0x8C, 0xA0, 0x00, 0x00
                   ,0x6B]
        return self.send_cmd(message, 0.01)




    def motor_stop(self):
        """
        Stop the motor, but do not clear the operating state and previously received control commands.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x81, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def motor_run(self):
        """
        Resume motor operation from the motor stop command.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xFF, 0xFF, 0xFF, 0xFF,
                   0xFF, 0xFF, 0xFF, 0xFC]
        return self.send_cmd(message, 0.01)
    def motor_pi(self):
        """
        Resume motor operation from the motor stop command.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xC3, 0xF5, 0x48, 0x40,
                   0x00, 0x00, 0x80, 0x3F]
        return self.send_cmd(message, 0.01)

    def motor_zero(self):
        """
        Resume motor operation from the motor stop command.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x80, 0x3F]
        return self.send_cmd(message, 0.01)

    def targetSpeedMode(self, data):
        """
        Resume motor operation from the motor stop command.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [data[0], data[1], data[2], data[3],
                   data[4], data[5], data[6], data[7]]

        return self.send_cmd(message, 0.01)


    def read_motor_status_1(self):
        """
        Reads the motor's error status, voltage, temperature and other information.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9A, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def read_motor_status_2(self):
        """
        Reads the motor temperature, voltage, speed and encoder position.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9C, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)


    def read_motor_status_3(self):
        """
        Reads the phase current status data of the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9D, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def clear_motor_error_flag(self):
        """
        Clears the error status of the currrent motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0x9B, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def torque_closed_loop(self, data):
        """
        Control torque current output of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA1, 0x00, 0x00, 0x00,
                   data[0], data[1], 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def speed_closed_loop(self, data):
        """
        Control the speed of the motor.

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA2, 0x00, 0x00, 0x00,
                   data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.01)

    def position_closed_loop_1(self, data):
        """
        Control the position of the motor (multi-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA3, 0x00, 0x00, 0x00,
                   data[0], data[1], data[2], data[3]]
        return self.send_cmd(message, 0.01)

    def position_closed_loop_2(self, data):
        """
        Control the position of the motor (multi-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA4, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.01)


    def position_closed_loop_6(self, data):
        """
        Control the position of the motor (multi-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA8, 0x00, data[0], data[1],
                   data[2], data[3], data[4], data[5]]
        return self.send_cmd(message, 0.01)

    def position_closed_loop_3(self, data):
        """
        Control the position of the motor (single-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA5, data[0], 0x00, 0x00,
                   data[1], data[2], 0x00, 0x00]
        return self.send_cmd(message, 0.01)

    def position_closed_loop_4(self, data):
        """
        Control the position of the motor (single-turn angle).

        Parameters
        ----------
        data : list
            The frame data to be sent to the motor.

        Returns
        -------
        received_message : list
            Frame data received from the motor after receiving the command.
        """
        message = [0xA6, data[0], data[1], data[2],
                   data[3], data[4], 0x00, 0x00]
        return self.send_cmd(message, 0.01)




