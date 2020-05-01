#!/usr/bin/env python

# Python Imports
import sys

# ROS Imports
import rospy
import roslib
import actionlib
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal, MoveBaseResult
import ros_gui_server.msg

class gui_server():

    # create messages that are used to publish feedback/result
    _odomCaptureFeedback = ros_gui_server.msg.OdomCaptureFeedback()
    _odomCaptureResult = ros_gui_server.msg.OdomCaptureResult()
    _odomClearFeedback = ros_gui_server.msg.OdomClearFeedback()
    _odomClearResult = ros_gui_server.msg.OdomClearResult()
    _odomMoveFeedback = ros_gui_server.msg.MoveToMarkersFeedback()
    _odomMoveResult = ros_gui_server.msg.MoveToMarkersResult()

    # Initialization Function
    def __init__(self, argv):

        # Initializing class variables
        self.currentOdom = Odometry()
        self.odomArray = []
        self.capture_action_name = "/ros_gui_server/odom_capture"
        self.clear_action_name = "/ros_gui_server/odom_clear"
        self.move_action_name = "/ros_gui_server/move_to_markers"

        # Set up persistent Point array for passing to new messages 
        self.points = []

        # Command Line Arguments
        if len(argv) > 1:
            self.odometryTopic = argv[1]
        else:
            self.odometryTopic = "/odometry/filtered"

        # Subscriber, Publisher and Action Server/Client setup
        self.odomSubscriber = rospy.Subscriber(self.odometryTopic, Odometry, self.odomRecieveCallback) # Subscribe the the Odometry topic for Odometry message recieving
        self.odomCaptureServer = actionlib.SimpleActionServer(self.capture_action_name, ros_gui_server.msg.OdomCaptureAction, execute_cb=self.odomCaptureCallback, auto_start=False) # Setup Odometry Capture server for recording /nav_msgs/Odometry messages on demand
        self.odomClearServer = actionlib.SimpleActionServer(self.clear_action_name, ros_gui_server.msg.OdomClearAction, execute_cb=self.odomClearCallback, auto_start=False) # Setup Odometry Clear server for clearing all odometry entries
        self.moveToMarkersServer = actionlib.SimpleActionServer(self.move_action_name, ros_gui_server.msg.MoveToMarkersAction, execute_cb=self.moveToMarkersCallback, auto_start=False) # Setup move to markers server for providing move_base functionality
        self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.rvizMarkerServer = rospy.Publisher("/ros_gui_server/marker_server", Marker, queue_size=10)
        
        # Starting actions
        self.odomCaptureServer.start()
        self.odomClearServer.start()
        self.moveToMarkersServer.start()
        self.moveBaseClient.wait_for_server()

    # Callback method for our nav_msgs/Odometry subscriber
    def odomRecieveCallback(self, data):

        self.currentOdom = data # Update the current odometry message variable with the most recently recieved one
        #rospy.loginfo(rospy.get_caller_id() + " recieved Odometry message from frame:" + self.currentOdom.header.frame_id + "\nX: " + str(self.currentOdom.pose.pose.position.x) + "\nY: " + str(self.currentOdom.pose.pose.position.y)) # Log that the message was recieved

    def odomCaptureCallback(self, goal):
        
        # Setting up variables
        appendOdom = Odometry()
        newPoint = Point()
        newPoints = Marker()
        newLineStrip = Marker()
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
            success = (self.odomArray[len(self.odomArray) - 1].header.stamp == appendOdom.header.stamp)
            self._odomCaptureResult.exit_status = success

            # set up new Point message for the points array and append it
            newPoint.x = appendOdom.pose.pose.position.x
            newPoint.y = appendOdom.pose.pose.position.y
            newPoint.z = appendOdom.pose.pose.position.z
            self.points.append(newPoint)

            # set up Marker message common parameters
            newPoints.header.frame_id = newLineStrip.header.frame_id = "/odom"
            newPoints.header.stamp = newLineStrip.header.stamp = rospy.get_rostime()
            newPoints.ns = newLineStrip.ns = "/ros_gui_server/marker_server"
            newPoints.action = newLineStrip.action = Marker.ADD
            newPoints.pose.orientation.w = newLineStrip.pose.orientation.w = 1.0
            
            # set up Marker message type
            newPoints.type = Marker.POINTS
            newLineStrip.type = Marker.LINE_STRIP

            # set up Marker message ID
            newPoints.id = 0
            newLineStrip.id = 1

            # set up newPoints scaling and color
            newPoints.scale.x = 0.2
            newPoints.scale.y = 0.2
            newPoints.color.a = 0.95
            newPoints.color.r = 0.502
            newPoints.color.g = 0.0
            newPoints.color.b = 0.126

            # set up newLineStrip scaling and color
            newLineStrip.scale.x = 0.2
            newLineStrip.scale.y = 0.2
            newLineStrip.color.a = 1.0
            newLineStrip.color.r = 0.729
            newLineStrip.color.g = 0.047
            newLineStrip.color.b = 0.184

            # give latest points array
            newPoints.points = self.points
            newLineStrip.points = self.points

            # finally, publish the new markers
            self.rvizMarkerServer.publish(newPoints)
            self.rvizMarkerServer.publish(newLineStrip)

            # Feedback publish
            self._odomCaptureFeedback.percent_complete = 100.00
            self.odomCaptureServer.publish_feedback(self._odomCaptureFeedback)

            # complete the action callback
            rospy.loginfo('%s: Succeeded' % self.capture_action_name)
            self.odomArrayPrint()
            self.odomCaptureServer.set_succeeded(self._odomCaptureResult)
    
    def odomClearCallback(self, data):

        # Determine if there are any markers to clear
        if len(self.odomArray) < 1:

            # if there are not points in the array, do nothing and pass an exit failure
            
            # Feedback publish
            self._odomClearFeedback.percent_complete = 0.00
            self.odomClearServer.publish_feedback(self._odomClearFeedback)

            # complete action callback
            rospy.loginfo('%s: Failed: No Markers to delete' % self.clear_action_name)
            self._odomClearResult.exit_status = False
            self.odomClearServer.set_aborted(self._odomClearResult)

        else:

            # Clear Odometry Array and Points Array
            self.odomArray = []
            self.points = []

            # Set up delete markers
            deletePoints = Marker()
            deleteLineStrip = Marker()
            deletePoints.header.frame_id = deleteLineStrip.header.frame_id = "/odom"
            deletePoints.header.stamp = deleteLineStrip.header.stamp = rospy.get_rostime()
            deletePoints.ns = deleteLineStrip.ns = "/ros_gui_server/marker_server"
            deletePoints.action = deleteLineStrip.action = Marker.DELETEALL
            
            # set up Marker message IDs
            deletePoints.id = 0
            deleteLineStrip.id = 1

            # finally, publish the delete markers
            self.rvizMarkerServer.publish(deletePoints)
            self.rvizMarkerServer.publish(deleteLineStrip)

            # Feedback publish
            self._odomClearFeedback.percent_complete = 100.00
            self.odomClearServer.publish_feedback(self._odomClearFeedback)

            # complete action callback
            rospy.loginfo('%s: Succeeded' % self.clear_action_name)
            self._odomClearResult.exit_status = True
            self.odomClearServer.set_succeeded(self._odomClearResult)
    
    def moveToMarkersCallback(self, data):

        # Determine if there are any markers to move to
        if len(self.odomArray) < 1:

            # if there are not points in the array, do nothing and pass an exit failure
            
            # Feedback publish
            self._odomMoveFeedback.percent_complete = 0.00
            self.moveToMarkersServer.publish_feedback(self._odomMoveFeedback)

            # complete action callback
            rospy.loginfo('%s: Failed: No Markers to move to' % self.move_action_name)
            self._odomMoveResult.exit_status = False
            self.moveToMarkersServer.set_aborted(self._odomMoveResult)

        else:

            # Send Goals and wait for feedback
            for num,name in enumerate (self.odomArray):

                # Time this goal was sent
                time = rospy.get_rostime()

                # Init MoveBaseGoal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = name.header.frame_id
                goal.target_pose.header.stamp = time
                goal.target_pose.pose = name.pose.pose

                # send the goal
                self.moveBaseClient.send_goal_and_wait(goal)

                # send feedback to client
                self._odomMoveFeedback.percent_complete = num/len(self.odomArray)
                self.moveToMarkersServer.publish_feedback(self._odomMoveFeedback)
            
            # If we get here, all of our sent goals succedded, and so did we! Complete action callback
            rospy.loginfo('%s: Succeeded' % self.move_action_name)
            self._odomMoveResult.exit_status = True
            self.moveToMarkersServer.set_succeeded(self._odomMoveResult)


    def odomArrayPrint(self):

        for i in range(len(self.odomArray)):
            printOdom = self.odomArray[i]
            printStr = str(i) + ":\n\n   header:\n    seq: " + str(printOdom.header.seq) + "\n    stamp: " + str(printOdom.header.stamp) + "\n    frame_id: " + printOdom.header.frame_id + "\n   child_frame_id: " + printOdom.child_frame_id \
            + "\n   pose:\n    pose:\n        position:\n            x: " + str(printOdom.pose.pose.position.x) + "\n            y: " + str(printOdom.pose.pose.position.y) + "\n            z: " + str(printOdom.pose.pose.position.z) \
            + "\n        orientation:\n            x: " + str(printOdom.pose.pose.orientation.x) + "\n            y: " + str(printOdom.pose.pose.orientation.y) + "\n            z: " + str(printOdom.pose.pose.orientation.z) + "\n            w: " + str(printOdom.pose.pose.orientation.w) \
            + "\n   twist:\n    twist:\n        linear:\n            x: " + str(printOdom.twist.twist.linear.x) + "\n            y: " + str(printOdom.twist.twist.linear.y) + "\n            z: " + str(printOdom.twist.twist.linear.z) \
            + "\n        angular:\n            x: " + str(printOdom.twist.twist.angular.x) + "\n            y: " + str(printOdom.twist.twist.angular.y) + "\n            z: " + str(printOdom.twist.twist.angular.z) + "\n\n"
            rospy.loginfo(printStr)


def main(args):
    rospy.init_node('ros_gui_server_node') # init node
    server = gui_server(args)
    print "ros_gui_server_node is ready to serve!"

if __name__ == "__main__":
    main(sys.argv)
    rospy.spin()