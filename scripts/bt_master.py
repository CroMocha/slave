#!/usr/bin/env python
# Uses Bluez for Linux
#
# sudo apt-get install bluez python-bluez
#
# Taken from: https://people.csail.mit.edu/albert/bluez-intro/x232.html
# Taken from: https://people.csail.mit.edu/albert/bluez-intro/c212.html


import bluetooth
import socket
from subprocess import Popen, call, PIPE
import rospy
import time
from select import *
from std_msgs.msg import String
from aquascrub.msg import velocity

slave_addr = "B8:27:EB:1D:4F:A3"

#master is the client, master connects to slave
class BluetoothMaster:
    def __init__(self):
        rospy.init_node('bt_comms_with_slave', anonymous=True) #name the node

        self.pub = rospy.Publisher('bt_receive_from_slave', String, queue_size=10)

        rospy.Subscriber('bt_send_to_slave', velocity, self.write_to_slave) #send speed info to slave

        #rospy.Service('bluetooth_follower_modeset', CmdAck, self.mode_callback)
        self.connected = False #set marker to determine if connected
        self.port = 1 #set port
        self.connect_to_slave() #attempt to connect to server
        while not rospy.is_shutdown():
            self.read_from_slave()
    # Bluetooth connection functions

    def connect_to_slave(self):
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Or SO_REUSEPORT
        try:
            self.sock.connect((slave_addr, self.port))
            self.connected = True
            print "connected!"
        except Exception as e:
            error_code = str(e)
            print("Follower bt: " + error_code)
            if "(4," in error_code:
                exit()
        '''    if "(12," in error_code or "(52," in error_code or "(115," in error_code:
                self.reset_bluetooth()
                self.port = 0

            self.port += 1
            if self.port == 15:
                time.sleep(0.1)
                self.port = 0
                self.reset_bluetooth()
        else:
            print("Follower connection successful")
            self.sock.setblocking(False)'''

    def close_connections(self):
        time.sleep(1)
        try: self.sock.close()
        except: pass

    def reset_bluetooth(self):
        self.close_connections()
        call(["sudo", "hciconfig", "hci0", "reset"])


    def read_from_slave(self):
        print "attempting to read"
        while self.connected == True:
            try:
                print "connected, reading!"
                readable, writable, excepts = select([self.sock], [], [], 1)
                if self.sock in readable:
                    data = self.sock.recv(1024)
                    print "received and publishing!"
                    self.pub.publish(data)
            except Exception as e:
                print("Bt Slave Read: "+ str(e))
                if "(4," in str(e):
                    exit()

    def write_to_slave(self, data):
        while self.connected == True:
            try:
                readable, writable, excepts = select([], [self.sock], [], 1)
                if self.sock in writable:
                    self.sock.send(data.spd)
            except Exception as e:
                print("Bt Slave Write: "+ str(e))
                if "(4," in str(e):
                    exit()

    def close_connections(self):
        self.sock.close()

if __name__ == '__main__':
    f = BluetoothMaster()
