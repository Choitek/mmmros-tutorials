#!/usr/bin/env python
from __future__ import division
import rospy
import os
from Tkinter import *
from mmmros.msg import Movement, SensorData


class ControlApp(Tk, object):

    def __init__(self):
        super(ControlApp, self).__init__()
        # Set speeds for control
        self.wheelSpeed = 0.18  # meters/s
        self.angularSpeed = 30  # degrees/s
        self.angularResolution = 1  # degree
        self.armSpeed = 0.01  # meters/s
        self.armResolution = 0.001 # meters

        # Inititalize robot in default state
        self.state = Movement()
        self.moving = False

        # Initialize the teleop node
        rospy.init_node("teleop_node")
        # Create the publisher object for sending movement commands to the mmm node
        self.pub = rospy.Publisher("mmm/move_commands", Movement, queue_size=1)
        # Create the subscriber object for receiving sensor data
        self.sub = rospy.Subscriber("mmm/sensor_data", SensorData, self.display_sensor_data)

        # Set up the interface
        self.geometry("100x100")
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        self.sensorText = StringVar()
        self.sensorLabel = Label(self, textvariable=self.sensorText)
        self.sensorLabel.pack()
        self.mainloop()

    def display_sensor_data(self, msg):
        s = "Left: " + str(msg.leftRange) + "\nRight: " + str(msg.rightRange)
        self.sensorText.set(s)

    def keydown(self, event):
        # On/off actuators
        if event.keysym == "Up":
            self.state.leftWheelSpeed = self.wheelSpeed
            self.state.rightWheelSpeed = self.wheelSpeed
            self.pub.publish(self.state)
        elif event.keysym == "Down":
            self.state.leftWheelSpeed = -self.wheelSpeed
            self.state.rightWheelSpeed = -self.wheelSpeed
            self.pub.publish(self.state)
        elif event.keysym == "Left":
            self.state.leftWheelSpeed = -self.wheelSpeed
            self.state.rightWheelSpeed = self.wheelSpeed
            self.pub.publish(self.state)
        elif event.keysym == "Right":
            self.state.leftWheelSpeed = self.wheelSpeed
            self.state.rightWheelSpeed = -self.wheelSpeed
            self.pub.publish(self.state)
        # Continuous actuators
        if not self.moving:
            self.moving = True
            # SHOULDERS
            if event.keysym == "q":
                self.move("leftShoulderAngle", +self.angularResolution, self.angularSpeed)
            elif event.keysym == "a":
                self.move("leftShoulderAngle", -self.angularResolution, self.angularSpeed)
            elif event.keysym == "y":
                self.move("rightShoulderAngle", +self.angularResolution, self.angularSpeed)
            elif event.keysym == "h":
                self.move("rightShoulderAngle", -self.angularResolution, self.angularSpeed)
            # ELBOWS
            elif event.keysym == "w":
                self.move("leftElbowAngle", +self.angularResolution, self.angularSpeed)
            elif event.keysym == "s":
                self.move("leftElbowAngle", -self.angularResolution, self.angularSpeed)
            elif event.keysym == "t":
                self.move("rightElbowAngle", +self.angularResolution, self.angularSpeed)
            elif event.keysym == "g":
                self.move("rightElbowAngle", -self.angularResolution, self.angularSpeed)
            # ARMS
            elif event.keysym == "e":
                self.move("leftArmExtension", +self.armResolution, self.armSpeed)
            elif event.keysym == "d":
                self.move("leftArmExtension", -self.armResolution, self.armSpeed)
            elif event.keysym == "r":
                self.move("rightArmExtension", +self.armResolution, self.armSpeed)
            elif event.keysym == "f":
                self.move("rightArmExtension", -self.armResolution, self.armSpeed)

    def move(self, part, amount, speed):
        minval = rospy.get_param("mmm/" + part + "/min")
        maxval = rospy.get_param("mmm/" + part + "/max")
        if self.moving and minval <= getattr(self.state, part) + amount <= maxval:
            setattr(self.state, part, getattr(self.state, part) + amount)
            self.pub.publish(self.state)
            self.after(int(abs(amount / speed) * 1000), self.move, part, amount, speed)

    def keyup(self, event):
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            self.state.leftWheelSpeed = 0
            self.state.rightWheelSpeed = 0
            self.pub.publish(self.state)
        elif event.keysym in ["q", "a", "w", "s", "e", "d", "r", "f", "t", "g", "y", "h"]:
            self.moving = False


if __name__ == '__main__':
    os.system('xset r off')
    control = ControlApp()
    os.system('xset r on')
