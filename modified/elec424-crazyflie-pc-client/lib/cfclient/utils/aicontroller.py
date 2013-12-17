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

__author__ = ' Lee and Mira <3'
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
	self.outputStr = "";
        # AI variables
        self.timer1 = 0
        self.lastTime = 0
	self.error = 0
	
	self.pitchError = 0
	self.rollError = 0

	self.heighErrorIntegral = 0

	self.minError = 100000000 
	self.attempedOnSameParamter = 0
	self.minLandingThrust = 0.60
	self.minControlThrust = 0.72
	self.trainingInterval = 0.3
        self.lastTrained = 0.0
        self.timer2 = 0
	self.timer3 = 0
	self.timerError = 0
	self.timerWrite = 0
	self.rollList = []
	self.pitchList = []

        #=============Height and yaw==========
	
	self.calibrationHeightList = []
        self.yawDelta = 20
	self.yawUpdateInterval = 2
        self.startHeight = 0.0
        self.isCalibrating = True
	self.height = 5.0
	self.front = 1000
	self.smoothingHeightList = []
	self.smoothingCnt = 10
	self.yawTargetList = [0,40,80,120,160,120,80,40,0,-40,-80,-120,-160,-120,-80,-40]
	self.yawIter = 0

	#=====================================
	self.alreadySet = False
	self.previousHeight = 0	
	self.f = open('errorData.txt', 'w')
	self.froll = open('errorDataRoll.txt', 'w')
	self.fpitch = open('errorDataPitch.txt', 'w')
	self.actualData = {'Roll':0,'Pitch':0,'yaw':0}

	#==============================================================
	#variables related to pitch-based avoid
	self.avoiding = False
	self.prevDistanceError = 0
	self.DistanceErrorIntegral = 0
	self.calibrationScaleToInches = 26.52

	#negative pitch equals avoiding obstacle
	self.correctionPitchTarget = 0
	self.minControlPitch = -2;
	#==============================================================

        # ---AI tuning variables---
        # This is the thrust of the motors duing hover.  0.5 reaches ~1ft depending on battery
        self.maxThrust = 0.99
        self.calibrationTime = 1.5
        # Determines how fast to take off
        self.thrustInc = 0.001
        self.takeoffTime = 0.5
        # Determines how fast to land
        self.thrustDec = -0.01
        self.hoverTime = 10000
        # Sets the delay between test flights
        self.repeatDelay = 2

        # parameters pulled from json with defaults from crazyflie pid.h
        # perl -ne '/"(\w*)": {/ && print $1,  "\n" ' lib/cflib/cache/27A2C4BA.json
        self.cfParams = {
            'pid_rate.pitch_kp': 600.0,
            'pid_rate.pitch_kd': 0.0,
            'pid_rate.pitch_ki': 1.5,
            'pid_rate.roll_kp': 400.0,
            'pid_rate.roll_kd': 0.0,
            'pid_rate.roll_ki': 0.5,
            'pid_rate.yaw_kp': 50.0,
            'pid_rate.yaw_kd': 0.0,
            'pid_rate.yaw_ki': 25.0,
            'pid_attitude.pitch_kp': 4.0,
            'pid_attitude.pitch_kd': 0.0,
            'pid_attitude.pitch_ki': 2,
            'pid_attitude.roll_kp': 3.5,
            'pid_attitude.roll_kd': 0.0,
            'pid_attitude.roll_ki': 2,
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
	self.actualData['yaw'] = float(yaw)

    def setAdc(self,front,height):
        if self.isCalibrating:
            self.height = height
        else:
            '''if len(self.smoothingHeightList)<50:
                self.smoothingHeightList.append(height - self.startHeight)
            else:
                self.height = sum(self.smoothingHeightList)/len(self.smoothingHeightList)
                self.smoothingHeightList = []'''
	    self.height = height - self.startHeight
        self.front = front

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
	self.timer3 = self.timer3 + timeSinceLastAi
	self.timerError = self.timerError + timeSinceLastAi
	self.timerWrite = self.timerWrite + timeSinceLastAi
        self.lastTime = currentTime

	if not self.alreadySet:
	    self.alreadySet = True
	    for key in self.cfParams:
		self.updateCrazyFlieParam(key)
            

	#self.aiData["yaw"] = 1
        
        # Basic AutoPilot steadly increase thrust, hover, land and repeat
        # -------------------------------------------------------------
        # delay before takeoff
 
        if self.timer1 < 0:
            thrustDelta = 0
        # calibration
        elif self.timer1 < self.calibrationTime :
            thrustDelta = 0
            self.calibrationHeightList.append(self.height)
        
        # takeoff
        elif self.timer1 < self.takeoffTime + self.calibrationTime :
            self.isCalibrating = False
            self.startHeight = float(sum(self.calibrationHeightList)/len(self.calibrationHeightList))
            thrustDelta = self.thrustInc
	    
        # hold
        elif self.timer1 < self.takeoffTime + self.hoverTime + self.calibrationTime : 
	    thrustDelta = 0;
            thrustDelta = self.adjustThrust(self.height,36)
	    if self.timer3 >self.yawUpdateInterval:
                #print "miramira"
                self.timer3 = 0
                #self.addyaw()
	    
	    #self.obstacleAvoidance(self.front, 24)

        # land
        elif self.timer1 < 2 * self.takeoffTime + self.hoverTime + self.calibrationTime :
	    if self.aiData["thrust"] <= self.minLandingThrust:
		thrustDelta = 0
	    else:
		thrustDelta = self.thrustDec
	 
        # repeat
        else:
            self.timer1 = -self.repeatDelay
            thrustDelta = 0

        self.addThrust( thrustDelta )

 	if self.timerError > 0.1:
	    self.rollList.append(self.actualData["Roll"])
	    self.pitchList.append(self.actualData["Pitch"])
	    self.timerError = 0
	    #print self.actualData["yaw"]
	    
	    
	
	if self.timerWrite > 0.5:
            print 'height: '+str(self.height)
            print 'front: '+str(self.front)
	    self.timerWrite = 0
	    string = ','.join(str(e) for e in self.rollList)
	    string2 = ','.join(str(e) for e in self.pitchList)
	    self.f.write(string)
	    self.f.write("\n next \n")
	    self.f.write(string2)
	    self.f.write("\n\n End\n\n")
	
	    '''curError = abs(self.actualData["Roll"]) + abs(self.actualData["Pitch"])
	    self.outputStr = str(currentTime) + ";" + str(abs(self.actualData["Roll"])) + "\n"
	    self.f.write(self.outputStr)
	    self.outputStr = str(currentTime) + ";" + str(abs(self.actualData["Roll"])) + "\n"
	    self.f.write(self.outputStr)'''

            

        # override Other inputs as needed
        # --------------------------------------------------------------
        # self.data["roll"] = self.aiData["roll"]
        self.data["pitch"] = self.aiData["pitch"]
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
	#self.data["pitch"] = 0
	#self.data["roll"] = 0

    def addyaw(self):
	# do not change yaw when crazyflie is avoiding obstacles
	
	if not self.avoiding:
	    print "Actual yaw"
	    print self.actualData['yaw']
	    print self.yawTargetList[self.yawIter]
	    #self.aiData["yaw"] = self.convertDegreeToTarget(self.actualData['yaw'] + self.yawDelta)
	    self.aiData["yaw"] = self.convertDegreeToTarget(self.yawTargetList[self.yawIter])
	    self.yawIter = (self.yawIter+1)%len(self.yawTargetList)
	    
	
    def convertDegreeToTarget(self, targetDegree):
	k = 0.72/180/6*40
	if(targetDegree > 0):
	    return targetDegree * k + 0.2
	elif(targetDegree<0):
	    return targetDegree * k - 0.2
	else:
	    return 0

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
        kp = 0.1
        ki = 0.0000
        kd = 5 #the new sensor has much higher noise -> smaller kd to be safe

	sensorHeight = sensorHeight * self.calibrationScaleToInches
	diff = sensorHeight - targetHeight
        self.heighErrorIntegral = self.heighErrorIntegral + diff
	heightDiv = sensorHeight - self.previousHeight
	
	#print "height"
	#print sensorHeight

	self.previousHeight = sensorHeight
	thrustDelta = -(diff * kp + ki * self.heighErrorIntegral + kd*heightDiv)
	
	if(self.aiData["thrust"] < self.minControlThrust):
	    self.aiData["thrust"] = self.minControlThrust
	    thrustDelta = 0
	#print "thrust"
	#print self.aiData["thrust"]
	#print self.data["thrust"]
	return thrustDelta

    def obstacleAvoidance(self, sensorDistance, targetDistance):
	if(sensorDistance <= targetDistance):
	    self.avoiding=True
	else:
	    self.avoiding=False
	
	if(self.avoiding):
	    self.aiData["yaw"] = self.convertDegreeToTarget(self.actualData['yaw'])
   	    pitchDelta = self.pitchPID(sensorDistance, targetDistance)
	    self.correctionPitchTarget = self.correctionPitchTarget + pitchDelta
	    print "pitch target:"
	    print self.correctionPitchTarget
	else:
	    self.prevDistanceError = 0
	    self.DistanceErrorIntegral = 0
	    self.correctionPitchTarget = 0
	
	self.aiData["pitch"] = self.correctionPitchTarget


    def pitchPID(self, sensorDistance, targetDistance):
	kp = 0.001
        ki = 0.0000
        kd = 0.01

	diff = sensorDistance - targetDistance
        self.DistanceErrorIntegral = self.DistanceErrorIntegral + diff
	distanceDiv = sensorDistance - self.prevDistanceError
	
	#print diff
	self.prevDistanceError = sensorDistance
	pitchDelta = (diff * kp + ki * self.DistanceErrorIntegral + kd*distanceDiv)

	if(self.correctionPitchTarget < self.minControlPitch):
	    self.correctionPitchTarget = self.minControlPitch
	    pitchDelta = 0
	
	#max should be 0
	if(self.correctionPitchTarget >= 0):
		self.correctionPitchTarget  = 0
	
	print "Pitch tune:"
	print pitchDelta

	return pitchDelta
	
