#!/usr/bin/env python

import rospy
import bz2 
from std_msgs.msg import String, Char
import binascii

class Bz2_listener():
    def __init__(self):

        self.bz2_message = ''
        self.rate = rospy.Rate(5)
        self.empty_dict = {}
        self.letter = ''
        self.solution = Char()        
        rospy.Subscriber('/bz2_message', String, self.sub_callback)
        self.pub = rospy.Publisher('/solution', Char, queue_size=10)
        self.publisher()

    def sub_callback(self, msg):
        
        if self.bz2_message != msg:
            self.bz2_message = binascii.unhexlify(msg.data) # converting hexadecimal to binary 
            self.bz2_message = bz2.decompress(self.bz2_message) #decompressing message
            self.highest_repeat(self.bz2_message)
        else :
            pass
    # Function to hold message letters in dictionary and find highest repeated letter
    def highest_repeat(self, data):
        for i in data: 
            if i == ' ' or i == ',' or i == '.':
                pass 
            elif i not in self.empty_dict:
                self.empty_dict[i] = 1
            else:
                self.empty_dict[i] += 1

        max = 0
        for key, value in self.empty_dict.items() :
            
            # We find max value of dictionary and hold its key 
            if max < value:
                max = value
                self.letter = key
            else: 
                pass

            
    def publisher(self):
        self.rate.sleep()
        while not rospy.is_shutdown():
            self.solution.data = ord(self.letter) 
            self.pub.publish(self.solution) 
            self.rate.sleep()

if __name__ == '__main__':
    try: 
        rospy.init_node("bz2_listener")

        letter = Bz2_listener()
        
    except rospy.ROSInterruptException:
        pass 