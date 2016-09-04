#!/usr/bin/env python
from __future__ import division
import rospy
import tf
import threading
from math import radians, cos, sin, pi
from mmmros.msg import Movement

quaternion = tf.transformations.quaternion_from_euler
euler = tf.transformations.euler_from_quaternion

class TFNode(object):

    def __init__(self):
        # Dimensions (in meters)
        self.baseToChest = rospy.get_param("mmm/dimensions/baseToChest")
        self.chestToShoulder = rospy.get_param("mmm/dimensions/chestToShoulder")
        self.shoulderToElbow = rospy.get_param("mmm/dimensions/shoulderToElbow")
        self.elbowToWrist = rospy.get_param("mmm/dimensions/elbowToWrist")
        # Variable for keeping track of state
        self.moving = False
        # Initialize the teleop node
        rospy.init_node("tf_node")
        # Create tf publisher
        self.br = tf.TransformBroadcaster()
        # Create tf listener
        self.ls = tf.TransformListener()
        # Create the subscriber object for receiving sensor data
        self.sub = rospy.Subscriber("mmm/move_commands", Movement, self.sendTransforms)
        # Send initial default state
        rospy.sleep(1)
        self.br.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "base_link", "map")
        rospy.sleep(1)
        self.sendTransforms(Movement())
        # Prevent from closing
        rospy.spin()

    def sendTransforms(self, msg):
        t = rospy.Time.now()
        # map -> base
        if msg.leftWheelSpeed != 0 or msg.rightWheelSpeed != 0:
            self.moving = True
            threading.Thread(target=self.moveWheels, args=(msg, t)).start()
        if msg.leftWheelSpeed == 0 and msg.rightWheelSpeed == 0:
            self.moving = False
            trans, rot = self.ls.lookupTransform("map", "base_link", rospy.Time(0))
            self.br.sendTransform(trans, rot, t, "base_link", "map")
        # Base -> shoulder
        self.br.sendTransform((-self.chestToShoulder,0,self.baseToChest), quaternion(0,0,radians(180-msg.leftShoulderAngle)), t, "left_shoulder", "base_link")
        self.br.sendTransform((self.chestToShoulder,0,self.baseToChest), quaternion(0,0,radians(msg.rightShoulderAngle)), t, "right_shoulder","base_link")
        # Shoulder -> elbow
        self.br.sendTransform((self.shoulderToElbow,0,0), quaternion(0,-radians(msg.leftElbowAngle),0), t, "left_elbow", "left_shoulder")
        self.br.sendTransform((self.shoulderToElbow,0,0), quaternion(0,-radians(msg.rightElbowAngle),0), t, "right_elbow", "right_shoulder")
        # Elbow -> wrist
        self.br.sendTransform((self.elbowToWrist,0,0), (0,0,0,1), t, "left_wrist", "left_elbow")
        self.br.sendTransform((self.elbowToWrist,0,0), (0,0,0,1), t, "right_wrist", "right_elbow")
        # Wrist -> hand
        self.br.sendTransform((msg.leftArmExtension,0,0), (0,0,0,1), t, "left_hand", "left_wrist")
        self.br.sendTransform((msg.rightArmExtension,0,0), (0,0,0,1), t, "right_hand", "right_wrist")

    def moveWheels(self, msg, t):
        checkDelay = 1 / 20
        while self.moving:
            trans, rot = self.ls.lookupTransform("map", "base_link", rospy.Time(0))
            angle = euler(rot)[2]
            x = trans[0]
            y = trans[1]
            # Forward
            if msg.leftWheelSpeed > 0 and msg.rightWheelSpeed > 0:
                x += msg.leftWheelSpeed * checkDelay * cos(angle+pi/2)
                y += msg.leftWheelSpeed * checkDelay * sin(angle+pi/2)
            # Backward
            if msg.leftWheelSpeed < 0 and msg.rightWheelSpeed < 0:
                x += msg.leftWheelSpeed * checkDelay * cos(angle+pi/2)
                y += msg.leftWheelSpeed * checkDelay * sin(angle+pi/2)
            # Rotate Left
            if msg.leftWheelSpeed < 0 and msg.rightWheelSpeed > 0:
                angularSpeed = msg.leftWheelSpeed / self.chestToShoulder
                angle += angularSpeed * checkDelay
            # Rotate Right
            if msg.leftWheelSpeed > 0 and msg.rightWheelSpeed < 0:
                angularSpeed = msg.leftWheelSpeed / self.chestToShoulder
                angle += angularSpeed * checkDelay
            self.br.sendTransform((x,y,0), quaternion(0,0,angle), t, "base_link", "map")
            rospy.sleep(checkDelay)


if __name__ == '__main__':
    tf_node = TFNode()
