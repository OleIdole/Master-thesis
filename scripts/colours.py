"""
    colours.py

    Experimental

    Created: 2020/03/17
    Author: Ole
"""

#from enum import Enum

#class  Colours(Enum):
    #RED = 'red'
    #GREEN = 'green'
    #BLUE = 'blue'
    #YELLOW = 'yellow'

class ColourValues():
    #hueHigh = 255
    #hueLow = 0
    #satHigh = 255
    #satLow = 0
    #valHigh = 255
    #valLow = 0

    def __init__(self, hueHigh, hueLow, satHigh, satLow, valHigh, valLow):
        self.hueHigh = hueHigh
        self.hueLow = hueLow
        self.satHigh = satHigh
        self.satLow = satLow
        self.valHigh = valHigh
        self.valLow = valLow

if __name__ == '__main__':
    print "Starting ROS Beacon Detector module"
    #rospy.init_node('beacon_node', anonymous=True)
    blue = ColourValues(40, 30, 200, 50, 90, 20)
    print blue.hueHigh
    #try:
        #rospy.spin()
    #except KeyboardInterrupt:
        #print "Shutting down ROS Beacon Detector module"