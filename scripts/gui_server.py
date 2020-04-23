#!/usr/bin/env python

# Python Imports
import sys

# ROS Imports
import rospy
import roslib
import actionlib
from nav_msgs.msg import Odometry
import ros_gui_server.msg

class gui_server():

    # create messages that are used to publish feedback/result
    _odomCaptureFeedback = ros_gui_server.msg.OdomCaptureFeedback()
    _odomCaptureResult = ros_gui_server.msg.OdomCaptureResult()

    # Initialization Function
    def __init__(self):

        # Initializing class variables
        self.currentOdom = Odometry()
        self.odomArray = []
        self.capture_action_name = "/odom_capture"

        # Subscriptions and action setup
        self.odomSubscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.odomRecieveCallback) # Subscribe the the Odometry topic for Odometry message recieving
        self.odomCaptureServer = actionlib.SimpleActionServer(self.capture_action_name, ros_gui_server.msg.OdomCaptureAction, execute_cb=self.odomCaptureCallback, auto_start=False) # Setup Odometry Capture server for recording /nav_msgs/Odometry messages on demand
        
        # Starting actions
        self.odomCaptureServer.start()

    # Callback method for our nav_msgs/Odometry subscriber
    def odomRecieveCallback(self, data):

        self.currentOdom = data # Update the current odometry message variable with the most recently recieved one
        #rospy.loginfo(rospy.get_caller_id() + " recieved Odometry message from frame:" + self.currentOdom.header.frame_id + "\nX: " + str(self.currentOdom.pose.pose.position.x) + "\nY: " + str(self.currentOdom.pose.pose.position.y)) # Log that the message was recieved

    def odomCaptureCallback(self, goal):
        
        # Setting up variables
        appendOdom = Odometry()
        pose_position_x_sum = 0.00
        pose_position_y_sum = 0.00
        pose_position_z_sum = 0.00
        pose_orientation_x_sum = 0.00
        pose_orientation_y_sum = 0.00
        pose_orientation_z_sum = 0.00
        pose_orientation_w_sum = 0.00
        twist_linear_x_sum = 0.00
        twist_linear_y_sum = 0.00
        twist_linear_z_sum = 0.00
        twist_angular_x_sum = 0.00
        twist_angular_y_sum = 0.00
        twist_angular_z_sum = 0.00

        # helper variables
        r = rospy.Rate(49)
        success = True
        
        if goal.samples == 0: # 0 samples are to be taken, simply interrupt the callback
            self._odomCaptureResult.exit_status = success
            self._odomCaptureFeedback.percent_complete = 100.00
            self.odomCaptureServer.publish_feedback(self._odomCaptureFeedback)
            self._odomCaptureResult.exit_status = False
            rospy.loginfo('%s: Succeeded' % self.capture_action_name)
            self.odomCaptureServer.set_succeeded(self._odomCaptureResult)

        else: # exactly 1 sample is to be taken
            appendOdom = self.currentOdom
            self.odomArray.append(appendOdom) # Appending the latest nav_msgs/Odometry reading to the array/list
            self._odomCaptureFeedback.percent_complete = 100.00
            self.odomCaptureServer.publish_feedback(self._odomCaptureFeedback)
            success = (self.odomArray[len(self.odomArray) - 1].header.stamp == appendOdom.header.stamp)
            self._odomCaptureResult.exit_status = success
            rospy.loginfo('%s: Succeeded' % self.capture_action_name)
            self.odomArrayPrint()
            self.odomCaptureServer.set_succeeded(self._odomCaptureResult)

        # else: # an arbitrary number of samples >1 is to be taken, averaging their values (pose, twist, and their corresponding covariances)
        #     for i in range (1, goal.samples):

        #         # Check that preempt has not been requested by the client
        #         if self.odomCaptureServer.is_preempt_requested():
        #             rospy.loginfo('%s: Preempted' % self.capture_action_name)
        #             success = False
        #             break

        #         # Summing the float64 standalone variables
        #         pose_position_x_sum = pose_position_x_sum + self.currentOdom.pose.pose.position.x
        #         pose_position_y_sum = pose_position_y_sum + self.currentOdom.pose.pose.position.y
        #         pose_position_z_sum = pose_position_z_sum + self.currentOdom.pose.pose.position.z
        #         pose_orientation_x_sum = pose_orientation_x_sum + self.currentOdom.pose.pose.orientation.x
        #         pose_orientation_y_sum = pose_orientation_y_sum + self.currentOdom.pose.pose.orientation.y
        #         pose_orientation_z_sum = pose_orientation_z_sum + self.currentOdom.pose.pose.orientation.z
        #         pose_orientation_w_sum = pose_orientation_w_sum + self.currentOdom.pose.pose.orientation.w
        #         twist_linear_x_sum = twist_linear_x_sum + self.currentOdom.twist.twist.linear.x
        #         twist_linear_y_sum = twist_linear_y_sum + self.currentOdom.twist.twist.linear.y
        #         twist_linear_z_sum = twist_linear_z_sum + self.currentOdom.twist.twist.linear.z
        #         twist_angular_x_sum = twist_angular_x_sum + self.currentOdom.twist.twist.angular.x
        #         twist_angular_y_sum = twist_angular_y_sum + self.currentOdom.twist.twist.angular.y
        #         twist_angular_z_sum = twist_angular_z_sum + self.currentOdom.twist.twist.angular.z

        #         # Averaging the Pose covariances
        #         if i is 1:
        #             pose_covariance = self.currentOdom.pose.covariance

        #         else:
        #             covariance_pair = (pose_covariance, self.currentOdom.pose.covariance)
        #             for i,j in covariance_pair:
        #                 returnTuple.append((i+j)/2)
        #             pose_covariance = returnTuple 
        #         # Averaging the Twist covariances
        #         if i is 1:
        #             twist_covariance = self.currentOdom.twist.covariance

        #         else:
        #             covariance_pair = (pose_covariance, self.currentOdom.twist.covariance)
        #             for i,j in covariance_pair:
        #                 returnTuple.append((i+j)/2)
        #             twist_covariance = returnTuple

        #         # Updating and publishing the feedback
        #         self._odomCaptureFeedback.percent_complete = float(i/goal.samples) * 100
        #         self.odomCaptureServer.publish_feedback(self._odomCaptureFeedback)
                
        #         # ROS sleep call to make sure we're sampling at a lower rate than the Odometry publisher
        #         r.sleep()
            
        #     # Finding averages for the float64 standalone variables (divide the sums by the number of samples)
        #     pose_position_x_avg = pose_position_x_sum/goal.samples
        #     pose_position_y_avg = pose_position_y_sum/goal.samples
        #     pose_position_z_avg = pose_position_z_sum/goal.samples
        #     pose_orientation_x_avg = pose_orientation_x_sum/goal.samples
        #     pose_orientation_y_avg = pose_orientation_y_sum/goal.samples
        #     pose_orientation_z_avg = pose_orientation_z_sum/goal.samples
        #     pose_orientation_w_avg = pose_orientation_w_sum/goal.samples
        #     twist_linear_x_avg = twist_linear_x_sum/goal.samples
        #     twist_linear_y_avg = twist_linear_y_sum/goal.samples
        #     twist_linear_z_avg = twist_linear_z_sum/goal.samples
        #     twist_angular_x_avg = twist_angular_x_sum/goal.samples
        #     twist_angular_y_avg = twist_angular_y_sum/goal.samples
        #     twist_angular_z_avg = twist_angular_z_sum/goal.samples

        #     # Preparing the Odometry message to be appended
        #     appendOdom = self.currentOdom # Giving the append Odometry message the current header and child_frame_id

        #     # Pose with Covariance
        #     appendOdom.pose.pose.position.x = pose_position_x_avg
        #     appendOdom.pose.pose.position.y = pose_position_y_avg
        #     appendOdom.pose.pose.position.z = pose_position_z_avg
        #     appendOdom.pose.pose.orientation.x = pose_orientation_x_avg
        #     appendOdom.pose.pose.orientation.y = pose_orientation_y_avg
        #     appendOdom.pose.pose.orientation.z = pose_orientation_z_avg
        #     appendOdom.pose.pose.orientation.w = pose_orientation_w_avg
        #     appendOdom.pose.covariance = pose_covariance

        #     # Twist with covariance
        #     appendOdom.twist.twist.linear.x = twist_linear_x_avg
        #     appendOdom.twist.twist.linear.y = twist_linear_y_avg
        #     appendOdom.twist.twist.linear.z = twist_linear_z_avg
        #     appendOdom.twist.twist.angular.x = twist_angular_x_avg
        #     appendOdom.twist.twist.angular.y = twist_angular_y_avg
        #     appendOdom.twist.twist.angular.z = twist_angular_z_avg
        #     appendOdom.twist.covariance = twist_covariance

        #     if success:
        #         self.odomArray.append(appendOdom)
        #         self._odomCaptureResult.exit_status = success
        #         rospy.loginfo('%s: Succeeded' % self.capture_action_name)
        #         self.odomCaptureServer.set_succeeded(self._odomCaptureResult)
    
    def odomArrayPrint(self):

        for i in range(len(self.odomArray)):
            printOdom = self.odomArray[i]
            printStr = str(i) + ":\n\n   header:\n    seq: " + str(printOdom.header.seq) + "\n    stamp: " + str(printOdom.header.stamp) + "\n    frame_id: " + printOdom.header.frame_id + "\n   child_frame_id: " + printOdom.child_frame_id \
            + "\n   pose:\n    pose:\n        position:\n            x: " + str(printOdom.pose.pose.position.x) + "\n            y: " + str(printOdom.pose.pose.position.y) + "\n            z: " + str(printOdom.pose.pose.position.z) \
            + "\n        orientation:\n            x: " + str(printOdom.pose.pose.orientation.x) + "\n            y: " + str(printOdom.pose.pose.orientation.y) + "\n            z: " + str(printOdom.pose.pose.orientation.z) + "\n            w: " + str(printOdom.pose.pose.orientation.w) \
            + "\n   twist:\n    twist:\n        linear:\n            x: " + str(printOdom.twist.twist.linear.x) + "\n            y: " + str(printOdom.twist.twist.linear.y) + "\n            z: " + str(printOdom.twist.twist.linear.z) \
            + "\n        angular:\n            x: " + str(printOdom.twist.twist.angular.x) + "\n            y: " + str(printOdom.twist.twist.angular.y) + "\n            z: " + str(printOdom.twist.twist.angular.z) + "\n\n"
            print printStr


def main():
    rospy.init_node('gui_server') # init node
    server = gui_server()
    print "gui_server is ready to serve!"

if __name__ == "__main__":
    main()
    rospy.spin()