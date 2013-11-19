#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Driver for reading data from the PyGame API. Used from Inpyt.py for reading input data.
Hacked to include AI 

You will need to modify the following files as shown below
+++ b/lib/cfclient/ui/main.py   
-        self.joystickReader = JoystickReader()
+        self.joystickReader = JoystickReader(cf=self.cf)


+++ b/lib/cfclient/utils/input.py   
+from cfclient.utils.aicontroller import AiController 

-    def __init__(self, do_device_discovery=True):
+    def __init__(self, do_device_discovery=True, cf=None):

-        self.inputdevice = PyGameReader()
+        self.inputdevice = AiController(cf)

You will also need to map the "exit" button to your controller.  This will server as 
the on/off switch for the AI.

You will also likely have to open the tools->parameters tab in the PC-Client which will load the TOC.  

"""

__author__ = 'Steven Arroyo and Lee Mira'
__all__ = ['AiController']

import pygame
from pygame.locals import *

import time
import logging
import copy


logger = logging.getLogger(__name__)

class AiController():
    """Used for reading data from input devices using the PyGame API."""
    def __init__(self,cf):
        self.cf = cf
        self.inputMap = None
        pygame.init()

        # AI variables
        self.timer1 = 0
        self.lastTime = 0
	self.error = 0
	self.minError = 100000000 
	self.attempedOnSameParamter = 0
	self.minLandingThrust = 0.7
	self.trainingInterval = 0.4
        self.lastTrained = 0.0
        self.timer2 = 0
        self.yawDelta = 0.001
	self.height = 0
	
	self.actualData={'Roll':0,'Pitch':0,'Yaw':0}

        # ---AI tuning variables---
        # This is the thrust of the motors duing hover.  0.5 reaches ~1ft depending on battery
        self.maxThrust = 0.8
        # Determines how fast to take off
        self.thrustInc = 0.01
        self.takeoffTime = 0.5
        # Determines how fast to land
        self.thrustDec = -0.01
        self.hoverTime = 15
        # Sets the delay between test flights
        self.repeatDelay = 0

        # parameters pulled from json with defaults from crazyflie pid.h
        # perl -ne '/"(\w*)": {/ && print $1,  "\n" ' lib/cflib/cache/27A2C4BA.json
        self.cfParams = {
            'pid_rate.pitch_kp': 85, 
            'pid_rate.pitch_kd': 0.1452, 
            'pid_rate.pitch_ki': 0.121, 
            'pid_rate.roll_kp': 85, 
            'pid_rate.roll_kd': 0.0865800865801, 
            'pid_rate.roll_ki': 0.0869740796394, 
            'pid_rate.yaw_kp': 47.4545454545, 
            'pid_rate.yaw_kd': 0.0, 
            'pid_rate.yaw_ki': 25.0, 
            'pid_attitude.pitch_kp': 3.86418554256, 
            'pid_attitude.pitch_kd': 0.0, 
            'pid_attitude.pitch_ki': 2.3, 
            'pid_attitude.roll_kp': 5.3805675, 
            'pid_attitude.roll_kd': 0.0, 
            'pid_attitude.roll_ki': 1.5026296018, 
            'pid_attitude.yaw_kp': 0.0, 
            'pid_attitude.yaw_kd': 0.0, 
            'pid_attitude.yaw_ki': 0.0, 
            'sensorfusion6.kp': 0.800000011921, 
            'sensorfusion6.ki': 0.00200000009499, 
            'imu_acc_lpf.factor': 32 }
	
	self.cfParamsFlag = copy.deepcopy(self.cfParams)
	#for the closed Loop Ziegler thing:
	self.amplitude = []
	self.previousError = 100
	self.ultimateGain = 0
	self.oscillationPeriod = 0

	
	self.initFlag()

    
    #initialize the optimization flags
    def initFlag(self):
	for k in self.cfParamsFlag:
	    self.cfParamsFlag[k] = 0

    def checkOptimizationFinished(self):
	for k in self.cfParamsFlag:
	    if self.cfParamsFlag[k] != 3:
		return False
	return True

            
    def readInput(self):
        """Read input from the selected device."""

        # First we read data from controller as normal
        # ----------------------------------------------------
        # We only want the pitch/roll cal to be "oneshot", don't
        # save this value.
        self.data["pitchcal"] = 0.0
        self.data["rollcal"] = 0.0
        for e in pygame.event.get():
          if e.type == pygame.locals.JOYAXISMOTION:
            index = "Input.AXIS-%d" % e.axis 
            try:
                if (self.inputMap[index]["type"] == "Input.AXIS"):
                    key = self.inputMap[index]["key"]
                    axisvalue = self.j.get_axis(e.axis)
                    # All axis are in the range [-a,+a]
                    axisvalue = axisvalue * self.inputMap[index]["scale"]
                    # The value is now in the correct direction and in the range [-1,1]
                    self.data[key] = axisvalue
            except Exception:
                # Axis not mapped, ignore..
                pass          

          if e.type == pygame.locals.JOYBUTTONDOWN:
            index = "Input.BUTTON-%d" % e.button 
            try:
                if (self.inputMap[index]["type"] == "Input.BUTTON"):
                    key = self.inputMap[index]["key"]
                    if (key == "estop"):
                        self.data["estop"] = not self.data["estop"]
                    elif (key == "exit"):
                        # self.data["exit"] = True
                        self.data["exit"] = not self.data["exit"]
                        logger.info("Toggling AI %d", self.data["exit"])
                    else: # Generic cal for pitch/roll
                        self.data[key] = self.inputMap[index]["scale"]
            except Exception:
                # Button not mapped, ignore..
                pass

        # Second if AI is enabled overwrite selected data with AI
        # ----------------------------------------------------------
        if self.data["exit"]:
            self.augmentInputWithAi()

        # Return control Data
        return self.data


    # ELEC424 TODO:  Improve this function as needed
    def setActualData(self,roll,pitch,yaw):
	self.actualData['Roll'] = float(roll)
	self.actualData['Pitch'] = float(pitch)
	self.actualData['Yaw'] = float(yaw)

    def setHeight(self,height):
        self.height = height

    def augmentInputWithAi(self):
        """
        Overrides the throttle input with a controlled takeoff, hover, and land loop.
        You will to adjust the tuning vaiables according to your crazyflie.  
        The max thrust has been set to 0.3 and likely will not fly.  
        I have found that a value  of 0.5 will reach about 1ft off the ground 
        depending on the battery's charge.
        """

        # Keep track of time
        currentTime = time.time()
        #print currentTime
        timeSinceLastAi = currentTime - self.lastTime
        self.timer1 = self.timer1 + timeSinceLastAi
        self.timer2 = self.timer2 + timeSinceLastAi
        self.lastTime = currentTime
        self.updateError()
        if self.timer2 > 0.005:
            #print "miramira"
            self.timer2 = 0
            self.addYaw(self.yawDelta)

            
        if not(self.checkOptimizationFinished()):
            if self.timer2 > 1:
                #print "miramira"
                #print self.timer2
                #self.pidTuner()
                #self.timer2 = 0
                #self.addYaw(self.yawDelta)
                pass
	else:
	    print "Optimization finished"
	    self.initFlag()

        
        # Basic AutoPilot steadly increase thrust, hover, land and repeat
        # -------------------------------------------------------------
        # delay before takeoff 
        if self.timer1 < 0:
            thrustDelta = 0
        # takeoff
        elif self.timer1 < self.takeoffTime :
            thrustDelta = self.thrustInc
	    
        # hold
        elif self.timer1 < self.takeoffTime + self.hoverTime : 
            thrustDelta = self.adjustThrust(self.height,36)
        # land
        elif self.timer1 < 2 * self.takeoffTime + self.hoverTime :
	    if self.aiData["thrust"] <= self.minLandingThrust:
		thrustDelta = 0
	    else:
		thrustDelta = self.thrustDec

        # repeat
        else:
            self.timer1 = -self.repeatDelay
            thrustDelta = 0
            # Example Call to pidTuner
	    

	    print "cycle error"


        self.addThrust( thrustDelta )

 

        # override Other inputs as needed
        # --------------------------------------------------------------
        # self.data["roll"] = self.aiData["roll"]
        # self.data["pitch"] = self.aiData["pitch"]
        self.data["yaw"] = self.aiData["yaw"]
        # self.data["pitchcal"] = self.aiData["pitchcal"]
        # self.data["rollcal"] = self.aiData["rollcal"]
        # self.data["estop"] = self.aiData["estop"]
        # self.data["exit"] = self.aiData["exit"]

    def addThrust(self, thrustDelta):
        # Increment thrust
        self.aiData["thrust"] = self.aiData["thrust"] + thrustDelta 
        # Check for max
        if self.aiData["thrust"] > self.maxThrust:
            self.aiData["thrust"] = self.maxThrust
        # check for min 
        elif self.aiData["thrust"] < 0:
            self.aiData["thrust"] = 0
        
        # overwrite joystick thrust values
        self.data["thrust"] = self.aiData["thrust"]

    def addYaw(self,yawDelta):
        self.aiData["yaw"] = self.aiData["yaw"] + self.yawDelta
        if (self.aiData["yaw"] > 0.72):
            self.aiData["yaw"] = self.aiData["yaw"] - 1.44
        
        
    # ELEC424 TODO: Implement this function
    def pidTuner(self):
	key = ''
	
	tuneRates = [1.2,1.1,1.05,1.01]
	fixGroup=['sensorfusion6.ki',"imu_acc_lpf.factor","sensorfusion6.kp",'pid_rate.yaw_kp', 'pid_rate.yaw_kd', 'pid_rate.yaw_ki','pid_attitude.pitch_kp']
	changeGroup=['pid_attitude.pitch_ki']
        for k in self.cfParams:
            if k in changeGroup:
                key = k
                #print key
                tuneScale = tuneRates[self.attempedOnSameParamter]
                if self.cfParamsFlag[k] == 0:
                    self.cfParamsFlag[k] = 1 #it starts to increase
                    self.cfParams[k] = self.cfParams[k] * tuneScale
                    self.minError = self.error #restart for everyvalue
                    break
                elif self.cfParamsFlag[k] == 1:
                    if self.error < self.minError:
                        self.cfParams[k] = self.cfParams[k] * tuneScale
                        self.minError = self.error
                        break
                    else:
                        self.cfParamsFlag[k] = 2 #it starts to decrease
                        self.cfParams[k] = self.cfParams[k] / tuneScale
                        break

                elif self.cfParamsFlag[k] == 2:
                    if self.error < self.minError:
                        self.cfParams[k] = self.cfParams[k] / tuneScale
                        self.minError = self.error
                        break
                    else:
                        if self.attempedOnSameParamter < 3:
                            self.attempedOnSameParamter = self.attempedOnSameParamter + 1
                            self.cfParamsFlag[k] = 0
                        else:
                            self.cfParamsFlag[k] = 3 #it achieved optimal value
                            self.cfParams[k] = self.cfParams[k] * tuneScale
                            self.attempedOnSameParamter = 0
                        break

        self.error = 0
	print str(key) + ":" + str(self.minError) + ":" + str(self.cfParams[key])
        self.updateCrazyFlieParam(key)

    def zieglerTuning(self):
        for param in cfParams:
            if "kp" in param:
                break
        pass
    
    #update using the goodness function
    def updateError(self):
	self.error = self.error + abs(self.actualData["Roll"]) + abs(self.actualData["Pitch"]) + abs(self.actualData["Yaw"]) * 0.0


    # update via param.py -> radiodriver.py -> crazyradio.py -> usbRadio )))
    def updateCrazyFlieParam(self, completename ):
        self.cf.param.set_value( unicode(completename), str(self.cfParams[completename]) )


    def start_input(self, deviceId, inputMap):
        """Initalize the reading and open the device with deviceId and set the mapping for axis/buttons using the
        inputMap"""
        self.data = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False}
        self.aiData = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False}
        self.inputMap = inputMap
        self.j = pygame.joystick.Joystick(deviceId)
        self.j.init()


    def enableRawReading(self, deviceId):
        """Enable reading of raw values (without mapping)"""
        self.j = pygame.joystick.Joystick(deviceId)
        self.j.init()

    def disableRawReading(self):
        """Disable raw reading"""
        # No need to de-init since there's no good support for multiple input devices
        pass

    def readRawValues(self):
        """Read out the raw values from the device"""
        rawaxis = {}
        rawbutton = {}

        for e in pygame.event.get():
            if e.type == pygame.locals.JOYBUTTONDOWN:
                rawbutton[e.button] = 1
            if e.type == pygame.locals.JOYBUTTONUP:
                rawbutton[e.button] = 0
            if e.type == pygame.locals.JOYAXISMOTION:
                rawaxis[e.axis] = self.j.get_axis(e.axis)

        return [rawaxis,rawbutton]

    def getAvailableDevices(self):
        """List all the available devices."""
        dev = []
        pygame.joystick.quit()
        pygame.joystick.init()
        nbrOfInputs = pygame.joystick.get_count()
        for i in range(0,nbrOfInputs):
            j = pygame.joystick.Joystick(i)
            dev.append({"id":i, "name" : j.get_name()})
        return dev

    def adjustThrust(self, sensorHeight, targetHeight):
	diff = sensorHeight - targetHeight
	print diff
        if(diff >0):
	    return -0.005
	else:
	    return 0.005
