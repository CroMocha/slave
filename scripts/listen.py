#!/usr/bin/env python
# Uses Bluez for Linux
#
# sudo apt-get install bluez python-bluez
# 
# Taken from: https://people.csail.mit.edu/albert/bluez-intro/x232.html
# Taken from: https://people.csail.mit.edu/albert/bluez-intro/c212.html


import bluetooth
from subprocess import Popen, call, PIPE
import rospy
import time
from select import *
from std_msgs.msg import String


class BluetoothSlave:
    def __init__(self):
        rospy.init_node('bt_comms_with_master', anonymous=True) #name the node

        self.pub = rospy.Publisher('bt_receive_from_master', String, queue_size=10)

        rospy.Subscriber('bt_send_to_master', String, self.write_to_master)

        #rospy.Service('bluetooth_follower_modeset', CmdAck, self.mode_callback)
        self.connected = False
        self.bluetooth_init()
        while not rospy.is_shutdown():
            self.read_from_master()

    def bluetooth_init(self):
        discoverable = False #BT Discovery

        while not discoverable:
            #call(["hciconfig", "hci0", "reset"])
            call(["sudo", "hciconfig", "hci0", "piscan"])
            p = Popen(['hciconfig'], stdin=PIPE, stdout=PIPE, stderr=PIPE)
            output_string = p.stdout.read()

            if (output_string.find('UP') != -1 and
                output_string.find('RUNNING') != -1 and
                output_string.find('PSCAN') != -1 and
                output_string.find('ISCAN') != -1):
                discoverable = True
                self.server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
                self.port = 1
                self.server_sock.bind(("",self.port))
                self.server_sock.listen(1)
                print "hello im here!"
                self.client_sock,address = self.server_sock.accept()
                print "Accepted connection from " + str(address)
                self.connected = True
            else:
                discoverable = False
                print "cant connect"
                time.sleep(1)

    def read_from_master(self):
        while self.connected == True:
            try:
                readable, writable, excepts = select([self.client_sock], [], [], 1)
                if self.client_sock in readable:
                    data = self.client_sock.recv(1024)
                    self.pub.publish(data)
            except Exception as e:
                print("Bt Master Read: "+ str(e))

    def write_to_master(self, data):
        while self.connected == True:
            try:
                readable, writable, excepts = select([], [self.client_sock], [], 1)
                if self.client_sock in writable:
                    self.client_sock.send(data.data)
                    break
            except Exception as e:
                print("Bt Master Write: "+ str(e))

    def close_connections(self):
        self.client_sock.close()
        self.server_sock.close()

    def lookUpNearbyBluetoothDevices():
        nearby_devices = bluetooth.discover_devices()
        for bdaddr in nearby_devices:
            print str(bluetooth.lookup_name( bdaddr )) + " [" + str(bdaddr) + "]"

if __name__ == '__main__':
    f = BluetoothSlave() 
