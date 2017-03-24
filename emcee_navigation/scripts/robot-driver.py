#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf
import math



class RobotDriver:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.listener = tf.TransformListener()

    def drive_forward_odom(self, distance):
        start_transform = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0), timeout=rospy.Duration(100));
        self.listener.lookupTransform()

        base_cmd = Twist()
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = 0.25

        rate = rospy.Rate(10)

        done = False
        while not done and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(base_cmd)
            rate.sleep()

            try:
                current_transform = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0));
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo(e)

            relative_transform = start_transform.inverse() * current_transform
            dist_moved = relative_transform.get_origin().length

def main():
    rospy.init_node('robot_driver')




def callback(objects):
    if (objects.data):
        obj_id = objects.data[0]
        rospy.loginfo(rospy.get_caller_id() + str(obj_id))
        print(obj_id)
        make_decisions(obj_id)



def make_decisions(obj_id):
    rate = rospy.Rate(10)

    rospy.loginfo(1)
    if (obj_id in odwallas):
        pub.publish(1)
    elif (obj_id in sprite):
        pub.publish(2)
    elif (obj_id in water):
        pub.publish(3)
    else:
        pub.publish(0)



def listener():
    rospy.Subscriber("objects", Float32MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    try:
        main();
    except rospy.ROSInterruptException:
        pass
