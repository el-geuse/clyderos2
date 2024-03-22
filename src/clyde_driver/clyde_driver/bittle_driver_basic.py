#!/usr/bin/env python3
import re
import time
import rclpy
import serial
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

dir_dict = {1: 'kwkF',          # walk forward
            -1: 'kbk',          # walk backwards
            2: 'kcrR',         # crawl to the right
            3: 'kcrL',         # crawl to the left
            0: 'kbalance'}     # balance (stay still)


class Driver(Node):

    def __init__(self, port='/dev/ttyS0'):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.vel_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # JointState publisher
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.joint_state_callback)  # 10Hz
        self.process_next_as_values = False
        
        self.joint_names = ["neck_joint", "shrfs_joint", "shrft_joint", "shrrs_joint", "shrrt_joint", \
                            "shlfs_joint", "shlft_joint", "shlrs_joint", "shlrt_joint"]

        self.dir = 0

        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)

    def vel_callback(self, msg):
        self.get_logger().info("Received a /cmd_vel message!")
        self.get_logger().info(
            "Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        self.get_logger().info(
            "Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

        if msg.linear.x > 0:
            dir = 1
        elif msg.linear.x < 0:
            dir = -1
        elif msg.angular.z > 0:
            dir = 2
        elif msg.angular.z < 0:
            dir = 3
        else:
            dir = 0

        if self.dir != dir:
            self.wrapper([dir_dict[dir], 0])
            self.dir = dir
    
    # def joint_state_callback(self):
    #     self.ser.write(b'j\n')  # Command to request joint angles
    #     time.sleep(0.05)  # Short pause to ensure data is fully received
    #     if self.ser.in_waiting:
    #         data = self.ser.readline().decode('utf-8').strip()  # Reading the response
    #         #self.get_logger().info(f"Raw data received: {data}")  # Log the raw data for debugging

    #         # Use regular expression to find all numbers in the string
    #         numbers = re.findall(r"[-+]?\d*\.\d+|\d+", data)

    #         # Attempt to parse the numbers into floats
    #         try:
    #             if numbers:
    #                 angles = list(map(float, numbers))[:9]  # Convert to float and take first 9
    #                 # Create and publish the JointState message
    #                 msg = JointState()
    #                 msg.header.stamp = self.get_clock().now().to_msg()
    #                 msg.name = self.joint_names[:len(angles)]  # Ensure names list matches number of angles
    #                 msg.position = angles
    #                 self.joint_state_publisher.publish(msg)
    #                 self.get_logger().info(f'Publishing JointState: {angles}')
    #             else:
    #                 self.get_logger().warn("No valid numbers found in the data.")
    #         except ValueError as e:
    #             self.get_logger().error(f"Error parsing joint angles: {e}")

    def joint_state_callback(self):
        self.ser.write(b'j\n')  # Request joint angles
        time.sleep(0.05)  # Wait for the Bittle's response
        if self.ser.in_waiting:
            data = self.ser.readline().decode('utf-8').strip()  # Read the response
            self.get_logger().info(f"Raw data received: {data}")  # Log for debugging

            # Use regular expression to find all numbers in the string
            numbers = re.findall(r"[-+]?\d*\.\d+|\d+", data)

            if self.process_next_as_values:  # Ensure this is the array with angles
                try:
                    if numbers:
                        # Extract relevant joint angles using list comprehension and mapping
                        # Include joint 0 and joints 9-15
                        relevant_angles = [float(numbers[0])] + [float(numbers[i]) for i in range(9, 16)]
                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        # Adjust this list to match the specific joints you're working with
                        msg.name = ['joint0'] + [f'joint{i}' for i in range(9, 16)]
                        msg.position = relevant_angles
                        self.joint_state_publisher.publish(msg)
                        self.get_logger().info(f'Publishing JointState: {relevant_angles}')
                    else:
                        self.get_logger().warn("No valid numbers found in the data.")
                except ValueError as e:
                    self.get_logger().error(f"Error parsing joint angles: {e}")
            self.process_next_as_values = not self.process_next_as_values  # Toggle for next set



    def wrapper(self, task):  # Structure is [token, var=[], time]
        print(task)
        if len(task) == 2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0], int):
            self.serialWriteNumToByte(task[0], task[1])
        else:
            self.serialWriteByte(task[1])
        time.sleep(task[-1])

    def serialWriteNumToByte(self, token, var=[]):  # Only to be used for c m u b i l o
        # print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
        if token == 'l' or token == 'i':
            var = list(map(lambda x: int(x), var))
            instrStr = token+struct.pack('b' * len(var), *var)+'~'

        elif token == 'c' or token == 'm' or token == 'u' or token == 'b':
            instrStr = token+str(var[0])+" "+str(var[1])+'\n'
        print("!!!!" + instrStr)
        self.ser.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        if (token == 'c' or token == 'm' or token == 'b' or token == 'u') and len(var) >= 2:
            instrStr = ""
            for element in var:
                instrStr = instrStr+element+" "
        elif token == 'l' or token == 'i':
            if(len(var[0]) > 1):
                var.insert(1, var[0][1:])
            var[1:] = list(map(lambda x: int(x), var[1:]))
            instrStr = token+struct.pack('b'*len(var[1:]), *var[1:])+'~'
        elif token == 'w' or token == 'k':
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        print("!!!!!!! "+instrStr)
        self.ser.write(instrStr.encode())

def main(args=None):
    rclpy.init()

    driver = Driver()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    