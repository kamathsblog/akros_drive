#!/usr/bin/python

"""
Class for the low level control of a 4wd car with Ackermann steering - with a DC motor for throttle and a servo motor for steering. It assumes ros-12cpwmboard has been installed.
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time


class ServoConvert():
    def __init__(self, id=1, center_value=333, center_range=5, range=90, direction=1):
        self.value         = 0.0
        self.value_out     = center_value
        self._center       = center_value
        self._center_range = center_range
        self._range        = range
        self._half_range   = 0.5*range
        self._dir          = direction
        self.id            = id
        
        #--- Convert its range in [-1, 1]
        self._sf           = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value         = value_in
        
        if self.id == 1:
            if value_in < 0:
                self.value_out     = int(self._dir*value_in*self._half_range + self._center-self._center_range)
            elif value_in >= 0:
                self.value_out     = int(self._dir*value_in*self._half_range + self._center)
        elif self.id == 2:
            self.value_out  = int(self._dir*value_in*self._half_range + self._center)
        #print self.id, self.value_out
        return(self.value_out)

class DriveAckermann():
    def __init__(self):

        rospy.init_node('drive_ackermann')
        rospy.loginfo("[ACK] Ackermann Drive application initialized")
        
        if rospy.has_param('/drive'):
            center_throttle = rospy.get_param('/drive/center_throttle')
            center_steering = rospy.get_param('/drive/center_steering')
            rospy.loginfo("[ACK] Loaded config paramters: /drive")
        else:
            center_throttle = 333;
            center_steering = 333;
            rospy.logerr("[ACK] Config parameter not found: /drive, center_throttle/steering set to 333")
                    
        self.actuators = {}
        self.actuators['throttle']  = ServoConvert(id=1, center_value = center_throttle, center_range = 9)
        self.actuators['steering']  = ServoConvert(id=2, center_value = center_steering) #-- positive left
        rospy.loginfo("[ACK] Actuators initialized")

        self._servo_msg = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("[ACK] Servo Publisher initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("[ACK] Twist Subscriber initialized")

        #--- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5

        rospy.loginfo("[ACK] Initialization complete")

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        rospy.loginfo("[ACK] Received command: Linear = %2.1f , Angular = %2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()
                                            
                                                               
    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo_once("[ACK] Actuators set to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
        
        rospy.loginfo("[ACK] Sending a command: Throttle = %d , Steering = %d"%(self._servo_msg.servos[0].value, self._servo_msg.servos[1].value))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        #print time.time() - self._last_time_cmd_rcv
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #print self._last_time_cmd_rcv, self.is_controller_connected
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    jetracer2 = DriveAckermann()
    jetracer2.run()
