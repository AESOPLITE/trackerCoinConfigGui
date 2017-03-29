import string
import time
import logging
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import datetime
import csv

#All of the functions are stored in another file
from AESOP_cmd import *


# import Tkinter as Tk
from Tkinter import *
from numpy import dtype
import Tkinter
from numpy.matlib import rand
# from nltk.downloader import TKINTER
# import Tkinter

#Script to test out the AESOP-Lite Tracker board
#R. Johnson	 February 9, 2016
#Brian Lucas	8/30/16
#
#V0.01  09/19/16 Initial version setups and polls events, cannot detect different events need AESOP_cmd update
#V0.02  10/06/16 Surveys over trig delay and window settings.  Cannot display events
#V0.03  10/27/16 Changed to for AESOP_cmd.py version and firmware
#V0.04  11/07/16 Added triggerSetup to resetBoards.  Added missed trigger buttons.
#V0.05  11/09/16 Changed to use a trigger notice on external (temp will be changed again)
#V0.06  11/09/16 Changed to poll event number
#V0.07  11/10/16 Changed Survey to include Buffer Speed
#V0.08	11/17/16 Added more survey sliders. Also Plot Events check will plot one event
#V0.09	11/21/16 Fixed some references to eventMetrics variable 
#V0.10	11/28/16 Changed Survey graph display to ns x-axis and to include error bars
#V0.11	11/30/16 Added counters above the sliders to indicate survey progress
#V0.12	12/14/16 Changed some of the task scheduling 
#V0.13	12/19/16 Changed to interactive mode for plotting events
#V0.14	12/22/16 Survey series changed from Trig Delay var to FPGA Delay
#V0.15	01/09/17 Added FPGA buf spinbox
#V0.16	01/17/17 Added many things to for full board integration. CSV even data file, Clear plot each event, Tracker Geometry with diff series for bending plane, ASIC power off button, new labels
#V0.17	01/25/17 Added mask for internal trig.  Upgraded AESOP_cmd to merge in changes and add closeCOM(). readTriggerNotice() doesn't work yet so still polling on internal triggers
#V0.18	01/25/17 Fixed some issues with internal trigger, adding a force button in case of missed notification.
#V0.19	02/13/17 Improvements for new int wait trigger.  Works intermittently looking at serial stream issues in AESOP_cmd.
#V0.20	03/01/17 Improvements for new int wait trigger including single nonbend trig plane w/ AESOP_cmd changes.  Survey changed for external only to use the trigger settings on GUI
#V0.21	03/09/17 Changed metric for survey to be number of layers rather than number of chips to reduce impact of multi chip hits in a single board
#V0.22	03/13/17 Created errorEventDump() to send debug commands and stop reading events
#V0.23	03/13/17 Additions to errorEventDump()
#V0.24	03/27/17 Additions to errorEventDump() and AESOP_cmd



titleVer = "Tracker Config V0.24"
#
#TODO coin rate 40hz set poll rate accordingly

#Globals
Boards =[]
trgList = dict({'0 ext' : 0, '1 ext & int' : 1, '2 ext or int' : 2, '3 int' : 3, '4 int wait' : 4})
nError = 0
eventNum = 0
countEvents = 0
countChips = 0
errorEventExit = False



#Functions for buttons
def setCOM():
	comChannel = "COM" + comSpin.get()	 #Careful, this channel assignment can change if the PC is rebooted.
	logging.info("Opening COM channel %s for UART communication with the master board via USB." % comChannel)
	openCOM(comChannel)
	
def endCOM():
	closeCOM()
	logging.info("Closed COM channel")

def setBoards():
	global Boards    # modify global copy of Boards
	Boards = range(int(boardsSpin.get()))
	logging.info("Boards list %s" % Boards)
	
def resetBoards():
	#reset the board and issue one time configuration settings
	global nError
	logging.info("Reseting these Boards %s" % Boards)
	logging.info("Send a reset to the FPGAs")
	for board in Boards:
		resetFPGA(board)
	
	
	logging.info("Reset the FPGA configuration")
	for board in Boards:
		resetFPGAconfig(board)
	
	logging.info("Set the number of layers in the readout")
	numLayers = len(Boards)
	setNumLayers(numLayers)
	for board in Boards:
		getNumLayers(board)
	
	
	logging.info("Send command to read the FPGA code verion")
	for board in Boards:
		getCodeVersion(board)
	
# 	response = raw_input("Enter something")
	
	logging.info("Turn the power on to the ASICs")
	ASICpowerON(7)
	
	time.sleep(0.1)
	
	logging.info("Send a hard reset to the ASICs")
	ASIChardReset(7,31)
	
	#time.sleep(0.1)
	
	logging.info("Send a soft reset to the ASICs")
	ASICsoftReset(7,31)
	
	logging.info("Set the ASIC default configuration")
	for board in Boards:
		ASICinitialize(board)
		time.sleep(0.1)
	
	logging.info("Reset the i2c state machines")
	i2cReset(7)
	
	for board in Boards:
		getBusVoltage(board,'flash18')
		getBusVoltage(board,'fpga12')
		getBusVoltage(board,'digi25')
		getBusVoltage(board,'i2c33')
		getBusVoltage(board,'analog21')
		getBusVoltage(board,'analog33')
		getShuntCurrent(board,'flash18')
		getShuntCurrent(board,'fpga12')
		getShuntCurrent(board,'digi25')
		getShuntCurrent(board,'i2c33')
		getShuntCurrent(board,'analog21')
		getShuntCurrent(board,'analog33')
		getShuntCurrent(board,'bias100')
		getTemperature(board)
	
	
	logging.info("Read the i2c error codes")
	for board in Boards:
		i2cErrors(board)
	
	# These config settings could only need to be run once with no GUI variables so placing them here.  Ordering of commands should work
	# Load all of the calibration DACs and read them back and check the values
	for board in []:
			for chip in range(12):
				Range = 0
				Value = 30
				logging.info("Load the ASIC calibration DAC for chip %d of board %d" % (chip,board))
				loadCalDAC(board,chip,Range,Value)
				logging.info("Read the ASIC calibration DAC for chip %d",chip)
				response = readCalDAC(board,chip)
				if int(response[0],2) != Range: 
					logging.error('		Wrong range returned by chip %d for the calibration DAC setting',chip)
					nError = nError + 1
				if int(response[1:8],2) != Value:
					logging.error('		Wrong value returned by chip %d for the calibration DAC setting',chip)
					nError = nError + 1
	
	# Load all of the threshold DACs and read them back and check the values
	Thresholds = [22,22,22,22,22,22,22]
	for board in Boards:
			for chip in range(12):
				Range = 0
				Value = Thresholds[board]		
				logging.info("Load the ASIC threshold DAC for chip %d of board %d" % (chip,board))
				loadThrDAC(board,chip,Range,Value)
				logging.info("Read the ASIC threshold DAC for chip %d",chip)
				response = readThrDAC(board,chip)
				logging.debug("DAC register read response=%s",response)
				if int(response[0],2) != Range: 
					logging.error('		Wrong range returned by chip %d for the threshold DAC setting',chip)
					nError = nError + 1
				if int(response[1:8],2) != Value: 
					logging.error('		Wrong value returned by chip %d for the threshold DAC setting',chip)	
					nError = nError + 1
			
	# Load all of the calibration masks and read them back and check the settings
	for board in []:	 
			for Chip in range(11,-1,-1):
				Mask= '0010010000000001110000000000000000000000000000000100000000000001'
			#		  '0123456789012345678901234567890123456789012345678901234567890123'
				logging.info("Load an ASIC calibration mask for chip %d on board %d with %s" % (Chip,board,Mask))
				loadCalMask(board,Chip,Mask)
				logging.info("Read the ASIC calibration mask")
				response = readCalMask(board,Chip)
				if response != Mask:
					logging.error('	 Wrong calibration mask returned by chip %d: ' + response,Chip)
					nError = nError + 1
		
	# Load all of the data masks and read them back and check the settings 
	for board in []: 
			for Chip in range(12):
				Mask= '1111111111111111111111111111111111111111111111111111111111111111'
			#		  '0123456789012345678901234567890123456789012345678901234567890123'
				logging.info("Load an ASIC data mask for chip %d of board %d with %s" % (Chip,board,Mask))
				loadDataMask(board,Chip,Mask)
				logging.info("Read the ASIC data mask")
				response = readDataMask(board,Chip)
				if response != Mask:
					logging.error('	 Wrong data mask returned by chip %d',Chip)
					nError = nError + 1
	
			
	# Load all of the trigger masks and read them back and check the settings	
	for board in Boards:
			for Chip in range(12):
				if Chip == 0 or Chip == 6:	#Disable trigger from 1st and last strips of each SSD
					Mask=	 '1111111111111111111111111111111111111111111111111111111111111110'
				else: 
					if Chip == 5 or Chip == 11: 
						Mask= '0111111111111111111111111111111111111111111111111111111111111111'
					else:
						Mask= '1111111111111111111111111111111111111111111111111111111111111111'
			#		          '0123456789012345678901234567890123456789012345678901234567890123'
				logging.info("Load an ASIC trigger mask for chip %d on board %d with %s" % (Chip,board,Mask))
				loadTrgMask(board,Chip,Mask)
				logging.info("Read the ASIC trigger mask")
				response = readTrgMask(board,Chip)
				if response != Mask:
					logging.error('	 Wrong trigger mask returned by chip %d' % (Chip)) 
					nError = nError + 1		
	
	
# 	triggerSetup(0,0,1)


	logging.info("The number of register load/read errors= %d",nError)
	
def asicOff():
	ASICpowerOFF(7)	
	logging.info("Turn the power off to the ASICs")

	
def configBoards():
	# Get trigger variables and Load all of the configuration registers and read them back, checking the settings
	configTrgReg(configBSSpin.get(), configTDSpin.get(), configTWVar.get())
	triggerSetup(7,int(configFDSpin.get()),1)


	
def configTrgReg(bufSpeed, trgDly, trgWin):
	# Load all of the configuration registers and read them back, checking the settings
	global nError
# 	global bufSpeed
# 	Address = 0
	Polarity = 0
	oneShot = 0
	Gain = 0
	shapingTime = 1
# 	bufSpeed = 3
# 	TrgDly = 3
# 	TrgDly = configTDSpin.get()
# 	TrgWin = 1
	Drive = 0
	MxClus = 10
	for board in Boards:
		for Chip in range(12):
			logging.info("Load the ASIC configuration for chip %d on board %d" % (Chip,board))
			Setting= loadConfigReg(board,Chip,Polarity,oneShot,Gain,shapingTime,int(bufSpeed),int(trgDly),int(trgWin),Drive,MxClus)
			logging.info("Read the ASIC configuration for chip %d on board %d" % (Chip,board))
			response = readConfigReg(board,Chip)
			if response[3:22] != Setting[0:19]: 
				logging.error('	 Wrong configuration returned by chip %d on board %d' % (Chip,board))
				nError = nError + 1
			logging.info("		Error flags from Chip %d are " + response[0:3],Chip)
		
	
	
	logging.info("The number of register load/read errors= %d",nError)
	
def setTrg():
	global trgNoticeVar
	global trgEventWaitVar
# 	trigStr = trgMenu.get()
	trgKey = trgList.get(trgVar.get(), -1)
	logging.info("trigvar %s %s", trgVar.get(), trgKey)
	if trgKey != -1 :
		logging.info("Setting trigger source to %d", trgKey)
		setTriggerSource(trgKey)
		if trgKey > 0 : #the internal trigger must be set from the relevant spinboxes
			triggerDelay = int(configIntTDSpin.get())
			triggerWidth = int(configIntTWSpin.get())
			logging.info("Setting int trigger %d, %d", triggerDelay,triggerWidth)
			intTriggerSetup(7,triggerDelay,triggerWidth)
			endBoards = []
			tempBoard = trgNonBendEndVar.get()
			if tempBoard != "":
				endBoards.append(int(tempBoard))
			if intTrgDualVar.get() :
				tempBoard = trgBendEndVar.get()
				if tempBoard != "":
					endBoards.append(int(tempBoard))
			for board in Boards:
				intTrgMaskVarVal = intTrgMaskVar[board].get()

				logging.info("Setting int Mask %d, %d", board,intTrgMaskVarVal)
				intTriggerType(board,'and')
				setTriggerMask(board,intTrgMaskVarVal)
				
				if i in endBoards :
					setTriggerEndLayer(i,1)
				else :
					setTriggerEndLayer(i,0)
				
				Range = 0
				Value = 22
				if intTrgMaskVarVal :
					Value = 28
				for chip in range(12):
					logging.info("Set the ASIC threshold DAC for chip %d of board %d to %d" % (chip,board,Value))
					loadThrDAC(board,chip,Range,Value)
					logging.info("Read the ASIC threshold DAC for chip %d",chip)
					response = readThrDAC(board,chip)
					logging.info("DAC register read response=%s",response)
					if int(response[0],2) != Range: 
						logging.error('    Wrong range returned by chip %d for the threshold DAC setting',chip)
						#nError = nError + 1
					intResponse= int(response[1:8],2)
					if intResponse != Value: 
						logging.error('    Wrong value returned by chip %d for the threshold DAC setting',chip)  
						#nError = nError + 1
			if trgKey == 4 :
				setDualTriggers(intTrgDualVar.get())
				setGoClockCycles(trigWaitScl.get())
				getGoClockCycles()
				trgNoticeVar.set(False)
				trgEventWaitVar.set(True)
			else :
				trgNoticeVar.set(True)
				trgEventWaitVar.set(False)
		else :
			trgNoticeVar.set(False)
			trgEventWaitVar.set(False)

		
def routineErrorCheck():
	
		getFPGAconfig(0)		
	
		time.sleep(0.1)		
						
		logging.info(" ")
		logging.info("Check the ASIC error flags. . .")
		for board in Boards:
				for Chip in range(12):
					logging.info("Read the ASIC configuration for FPGA %d chip %d" % (board,Chip))
					response = readConfigReg(board,Chip)
					logging.info("		Error flags from Chip %d are " + response[0:3],Chip)
		
		logging.info("Turn the analog power off to the ASICs")
		#ASICpowerOFF(7)
		
		for board in Boards:
			getShuntCurrent(board,'digi25')
		
def pollEnableChkCmd():
	#Check box is clicked, start or stop eventPolling task
	global eventPollingTask

	
	if (eventPollingEnable.get()):
		logging.info("Enable Trigger")
		enableTrigger()
# 		eventPolling()
		eventPollingTask = rootTk.after(1, eventPolling)

	else : 
		#plt.ioff()
		
		newEventVar.set(True)
		rootTk.after_cancel(eventPollingTask)
		rootTk.after_cancel(waitNewEventTask)
		logging.info("Disable Trigger")
		disableTrigger()
		#routineErrorCheck()

	
def plotEnableChkCmd():
	if (eventPlotEnable.get()) :
		plt.ion()
		plt.draw()
		logging.info("Plot interactive mode ")
	else:
		plt.ioff()
		plt.clf()
#		plt.close()
		
def eventPolling():
	#infinite loop to poll the tracker for events
	global newEventVar
	global eventPollingTask
	global waitNewEventTask
	global errorEventExit
# 	global eventNum
	
	errorEventExit = False
# # 	logging.info("init polling")
# # 		logging.info("test polling")
# 	trackerEventNum = getTriggerCount(0)
# 	logging.info("eventNum = %r trackerEventNum = %r", eventNum, trackerEventNum)
	
	newEventVar.set(False)
	waitNewEventTask = rootTk.after(1, waitNewEvent)
	rootTk.wait_variable(newEventVar)
# 	if (trackerEventNum != eventNum):
# 		eventNum = trackerEventNum
# 		logging.info("trackerEventNum = %r", trackerEventNum)
# 	getEvent(eventPlotEnable.get())
	if (eventPollingEnable.get() ):
		getEvent(eventPlotEnable.get())
		if not errorEventExit : eventPollingTask = rootTk.after(20, eventPolling)
		
	

# 	else: 
# 		logging.info("Disable Trigger")
# 		disableTrigger()
# 		getFPGAconfig(0)		
# 	
# 		time.sleep(0.1)		
# 						
# 		logging.info(" ")
# 		logging.info("Check the ASIC error flags. . .")
# 		for board in Boards:
# 				for Chip in range(12):
# 					logging.info("Read the ASIC configuration for FPGA %d chip %d" % (board,Chip))
# 					response = readConfigReg(board,Chip)
# 					logging.info("		Error flags from Chip %d are " + response[0:3],Chip)
# 		
# 		logging.info("Turn the analog power off to the ASICs")
# 		ASICpowerOFF(7)
# 		
# 		for board in Boards:
# 			getShuntCurrent(board,'digi25')
	
def waitNewEvent():
	global eventNum
	global newEventVar
	global waitNewEventTask
	
	if newEventVar.get() :
		logging.info("New Event variable not set %r", newEventVar)
	else :
		if trgNoticeVar.get() :
			if readTriggerNotice() == True :
# 				logging.info("Notice Trig")
				newEventVar.set(True)
			else :
				#logging.info("Schedule New task")
				waitNewEventTask = rootTk.after(1, waitNewEvent)
		elif trgEventWaitVar.get() :
			if readEventWait():
				newEventVar.set(True)
			else :
# 				logging.info("Schedule New task")
				waitNewEventTask = rootTk.after(20, waitNewEvent)
		else :
			trackerEventNum = getTriggerCount(0)
			if (eventNum != trackerEventNum) :
				logging.debug("eventNum = %r trackerEventNum = %r", eventNum, trackerEventNum)
		#		trackerEventNumHex = (trackerEventNum[2])[2:3] + (trackerEventNum[3])[2:3]
	# 			trackerEventNumHex = trackerEventNum[3]
	# 			trackerEventNum2Byte = 256 * int(binascii.hexlify(trackerEventNum[2]),16) + int(binascii.hexlify(trackerEventNum[3]),16)
	# 			if (trackerEventNum2Byte != eventNum):
				eventNum = trackerEventNum
				newEventVar.set(True)
	# 			else :
	# 				waitNewEventTask = rootTk.after(20, waitNewEvent)
			else :
	# 			logging.info("Schedule New task")
				waitNewEventTask = rootTk.after(20, waitNewEvent)
				
				
def eventCancelWaitTask():

	global newEventVar
	global waitNewEventTask
	

	newEventVar.set(True)
	if 'waitNewEventTask' in globals():
		rootTk.after_cancel(waitNewEventTask)
				
				
def getMissedTrg():
	global missedTVar
	missedTrgNum = getMissedTriggerCount(0)
	if len(missedTrgNum) > 2 :
		logging.debug("missedTrgNum = %r", missedTrgNum)
		missedTVar.set("Missed Triggers: " + str( int(binascii.hexlify(missedTrgNum[2]),16)))

def getMissedGo():
	global missedGoVar
	missedGoNum = getMissedGoCount()
	if len(missedGoNum) > 2 :
		logging.debug("missedTrgGo = %r", missedGoNum)
		missedGoVar.set("Missed GO: " + str( int(binascii.hexlify(missedGoNum[2]),16)))
		
def getASICDiag():
	getTriggersASIC()
	getASICBufferOverflows()
	
def getStateError():	
	for board in Boards:
		getStateVectors(board)
		if board != 0 : getEventsStreamed()
		for code in range(1,10) :
			getErrorCount(board, code)
		
	
def errorEventDump():	
	
	getStateError()
	
	routineErrorCheck()
			

	getStateError()
	
	getMissedTrg()
	getMissedGo()
	getASICDiag()
		
	
def getEvent(showPlot):
	#get a tracker event and plot it if showPlot is true
	
	global countEvents
	global countChips
	global countTrackerChips
	global eventNumVar
	global countChipsVar
	global countTrackerChipsVar
	global errorEventExit
	pitch = 0.228						 #pitch of the detector
	middlefirststrip = 0.035
	gapwidth = 0.075
	detectoredge = 1.088				#distance between the middle of the last strip and the edge of the detector
	middlefirststripchip6 = 87.359 + gapwidth + 2*detectoredge
	
	y= [212,192,126,60,40,20,0]
# 	enableTrigger()
# 	for j in range(20):
	#	nTry = 0
	#	while True:
	#		time.sleep(0.1)
	#		if readTriggerNotice(): break
	#		nTry = nTry + 1
	#		if nTry > 20:
	#			nTry = 0
# 	if j==29: disableTrigger()
# 	logging.info("j= %d",j)
# 	time.sleep(0.1)
	ReadTkrEventResult = ReadTkrEvent(0,False,True)	
	logging.debug("ReadTkrEventResult = %r", ReadTkrEventResult)
	[boardData,errorCode] = ReadTkrEventResult
	if (errorCode < 0) :
		logging.error("Event error code %d, running debug commands and halting run", errorCode)
		errorEventExit = True
		errorEventDump()
		return
	countEvents += 1
	chipHit = False
	if boardData :
		
		
		logging.debug("boardData = %r ", boardData)
# 		boardDataChips = int(boardData[0]['nChips'])
# 		countChips += boardDataChips #TODO actual calc of clusters 
# 		if (boardDataChips > 0) : #TODO more inclusive logic
# 			chipHit = True
		#boardDataChips is redundant 
	
		for Data in boardData:
		
			dataChips = int(Data['nChips'])
			dataRow=[]
			dataBoardAddress = int(Data['address']) % 8 #make address of master from 8 to 0	
			dataFirstStrip = Data['firstStrip']
# 			countTrackerChips[dataBoardAddress] += dataChips
# 			countChips += dataChips #TODO actual calc of clusters 
			dataRow.append(datetime.datetime.utcnow())
			dataRow.append(eventNum)
			dataRow.append(dataBoardAddress)
			dataRow.extend(dataFirstStrip)
# 			dataFirstStrip.insert(0, countEvents)
						
			
			csvW.writerow(dataRow)
			if (dataChips > 0) : #TODO more inclusive logic
				chipHit = True
				countTrackerChips[dataBoardAddress] += 1
				countChips += 1  
	
	eventNumVar.set("Event Num: " + str(eventNum))
	countChipsVar.set("Total Chip Hits: " + str(countChips))
	countTrackerChipsVar.set("Chip Hits per Board: " + str(countTrackerChips))			
	xList = [0]*7
	for lyr in range(7): xList[lyr]= []
	numberOfChips= [0]*7
#Convert the location of the hits to coordinates in mm, differentiate between the first 6 chips and that last 6 to account for the gap in between the detectors 
	nLyrHit = 0
	xpltNonBend = []
	ypltNonBend = []
	xpltBend = []
	ypltBend = []
	if (showPlot & chipHit):
		plt.clf()
		for Data in boardData:
			logging.debug("Data = %r", Data)
			for lyr in [8,1,2,3,4,5,6]:
				logging.debug("Data['address'] = %r lyr = %r", Data['address'], lyr)
				if Data['address'] == lyr:			
					if len(Data['firstStrip'])>0:
						nLyrHit = nLyrHit + 1
						if lyr == 8: lidx=0
						else: lidx=lyr
						if lidx in [0,4,6] :
							for i in Data['firstStrip']:
								if 0 <= i < 384:
									x = round((middlefirststrip+((i-0.5)*pitch)), 3)
									xList[lidx].append(x)
									xpltNonBend.append(x)
									ypltNonBend.append(y[lidx])
								else: 
									x= round((middlefirststripchip6+((i-384.5)*pitch)), 3)
									xList[lidx].append(x)
									xpltNonBend.append(x)
									ypltNonBend.append(y[lidx])
						else :
							for i in Data['firstStrip']:
								if 0 <= i < 384:
									x = round((middlefirststrip+((i-0.5)*pitch)), 3)
									xList[lidx].append(x)
									xpltBend.append(x)
									ypltBend.append(y[lidx])
								else: 
									x= round((middlefirststripchip6+((i-384.5)*pitch)), 3)
									xList[lidx].append(x)
									xpltBend.append(x)
									ypltBend.append(y[lidx])
						numberOfChips[lidx]=Data['nChips']
				continue
# 		plt.plot(xpltNonBend,ypltNonBend,linestyle='none',marker='o',label="NonBending Plane")
# 		plt.plot(xpltBend,ypltBend,linestyle='none',marker='o',label="Bending Plane")
# 		plt.ylim(-50,250)
# 		plt.xlim(-20,220)
# 		plt.xlabel('Position in Respective Plane (mm)')
# 		plt.ylabel('Z (mm)')
# 		plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., fontsize='medium')
		#fig = plt.figure()
		#ax = fig.add_axes([0.1, 0.1, 0.6, 0.85])
		#ax.plot(xpltNonBend,ypltNonBend,linestyle='none',marker='o',label="NonBending Plane")
		#ax.plot(xpltBend,ypltBend,linestyle='none',marker='o', color='red',label="Bending Plane")
		plt.plot(xpltNonBend,ypltNonBend,linestyle='none',marker='o',label="NonBending Plane")
		plt.plot(xpltBend,ypltBend,linestyle='none',marker='o', color='red',label="Bending Plane")
		#topMag = patches.Rectangle([0,134], 200, 50, color='grey')
		#botMag = patches.Rectangle([0,68], 200, 50, color='grey')
		#ax.add_patch(topMag)
		#ax.add_patch(botMag)
		#plt.add_patch(topMag)
		#plt.add_patch(botMag)
		plt.ylim(-50,250)
		plt.xlim(-20,220)
		plt.xlabel('Position in Respective Plane (mm)')
		plt.ylabel('Z (mm)')
		#ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., fontsize='medium')
		plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., fontsize='medium')
		plt.title("Tracker Event Geometry")
# 		plt.show()
# 		plt.draw()

def testPlot():
#used to test figure layout

	pitch = 0.228						 #pitch of the detector
	middlefirststrip = 0.035
	gapwidth = 0.075
	detectoredge = 1.088				#distance between the middle of the last strip and the edge of the detector
	middlefirststripchip6 = 87.359 + gapwidth + 2*detectoredge
	
	y= [212,192,126,60,40,20,0]

	xList = [0]*7
	for lyr in range(7): xList[lyr]= []
	numberOfChips= [0]*7
#Convert the location of the hits to coordinates in mm, differentiate between the first 6 chips and that last 6 to account for the gap in between the detectors 
	nLyrHit = 0
	xpltNonBend = []
	ypltNonBend = []
	xpltBend = []
	ypltBend = []
	
	for lidx in range(7) :
		i = lidx * 100
		if lidx in [0,4,6] :
			
			if 0 <= i < 384:
				x = round((middlefirststrip+((i-0.5)*pitch)), 3)
				xList[lidx].append(x)
				xpltNonBend.append(x)
				ypltNonBend.append(y[lidx])
			else: 
				x= round((middlefirststripchip6+((i-384.5)*pitch)), 3)
				xList[lidx].append(x)
				xpltNonBend.append(x)
				ypltNonBend.append(y[lidx])
		else :
			
			if 0 <= i < 384:
				x = round((middlefirststrip+((i-0.5)*pitch)), 3)
				xList[lidx].append(x)
				xpltBend.append(x)
				ypltBend.append(y[lidx])
			else: 
				x= round((middlefirststripchip6+((i-384.5)*pitch)), 3)
				xList[lidx].append(x)
				xpltBend.append(x)
				ypltBend.append(y[lidx])

	fig = plt.figure()
	ax = fig.add_axes([0.1, 0.1, 0.6, 0.85])
	ax.plot(xpltNonBend,ypltNonBend,linestyle='none',marker='o',label="NonBending Plane")
	ax.plot(xpltBend,ypltBend,linestyle='none',marker='o', color='red',label="Bending Plane")
	topMag = patches.Rectangle([0,134], 200, 50, color='grey')
	botMag = patches.Rectangle([0,68], 200, 50, color='grey')
	ax.add_patch(topMag)
	ax.add_patch(botMag)
	plt.ylim(-50,250)
	plt.xlim(-20,220)
	plt.xlabel('Position in Respective Plane (mm)')
	plt.ylabel('Z (mm)')
	ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., fontsize='medium')
	plt.title("Tracker Event Geometry")
	plt.show()
# 		plt.draw()
	
def surveyTrg():	
# 	Survey and graph all possible combinations of trig delay and window
# 	global countEvents
# 	global countChips
# 	global eventNum
	global newEventVar
	global waitNewEventTask
# 	global bufSpeed, fpgaTrgDly
	global surveyTrgIterVar, surveyTrgDlyVar, surveyTrgWinVar, surveyBufSpdVar, surveyFpgaTrgDlyVar
	global errorEventExit
	
	errorEventExit = False
	plt.ioff()
	
	minBufSpd = surveyBuffSpdMinScl.get()
	maxBufSpd = surveyBuffSpdMaxScl.get() + 1
	rangeBufSpd= range(minBufSpd,maxBufSpd)
	minTrgWin = surveyTrgWinMinScl.get()
	maxTrgWin = surveyTrgWinMaxScl.get() + 1
	rangeTrgWin = range(minTrgWin,maxTrgWin)
	minFpgaTrgDly = surveyFpgaTrgDlyMinScl.get()
	maxFpgaTrgDly = surveyFpgaTrgDlyMaxScl.get() + 1
	rangeFpgaTrgDly = range(minFpgaTrgDly,maxFpgaTrgDly)
	minTrgDly = surveyTrgDlyMinScl.get()
	maxTrgDly = surveyTrgDlyMaxScl.get() + 1
	rangeTrgDly= range(minTrgDly, maxTrgDly)
# 	eventMetrics = np.zeros((maxBufSpd,maxTrgWin,maxFpgaTrgDly,maxTrgDly),dtype=float,order='C')
	eventMetrics = np.zeros((maxBufSpd,maxTrgWin,maxTrgDly,maxFpgaTrgDly),dtype=float,order='C')

	sampleEvents = surveyTrgIterScl.get()
# 	for idxBufSpd in rangeBufSpd:
# 		for idxTrgWin in rangeTrgWin:
# 			for idxFpgaTrgDly in rangeFpgaTrgDly :
# 				timeDly = list(map(lambda x: ((((idxBufSpd + 1) * x) + idxFpgaTrgDly) * 100), rangeTrgDly))
# 				logging.info("timeDly[ %d , %d , %d] = %r", idxBufSpd, idxTrgWin, idxFpgaTrgDly, timeDly)

	for idxBufSpd in rangeBufSpd:
		surveyBufSpdVar.set(str(idxBufSpd))
		for idxTrgWin in rangeTrgWin:
			surveyTrgWinVar.set(str(idxTrgWin))
			for idxTrgDly in rangeTrgDly:
				surveyTrgDlyVar.set(str(idxTrgDly))
# 			for idxFpgaTrgDly in rangeFpgaTrgDly :
# 				surveyFpgaTrgDlyVar.set(str(idxFpgaTrgDly))
# 				for idxTrgDly in rangeTrgDly:
# 					surveyTrgDlyVar.set(str(idxTrgDly))
				for idxFpgaTrgDly in rangeFpgaTrgDly :
					surveyFpgaTrgDlyVar.set(str(idxFpgaTrgDly))
					resetBoards() # Full reset seems needed
					configTrgReg(idxBufSpd, idxTrgDly, idxTrgWin)
					triggerSetup(7,idxFpgaTrgDly,1)
					#setTriggerSource(0) 
					setTrg() #use exiting trigger options, must be set before survey starts
					enableTrigger()
		# 			setTrg()
		# 			trackerEventNum = getTriggerCount(0)
		# 			while (trackerEventNum == eventNum): 
		# 				trackerEventNum = getTriggerCount(0)
					#todo need to wait variable on a new event function
					newEventVar.set(False)
					waitNewEventTask = rootTk.after(1, waitNewEvent)
					rootTk.wait_variable(newEventVar)
					getEvent(False) # clear an event before starting measurement
		# 			eventNum = trackerEventNum
					lastCountEvents = countEvents
					lastCountClusters = countChips
					logging.info("countEvents = %d countChips = %d", countEvents, countChips)
					iterEvents = 0
					while ((iterEvents < sampleEvents) and not errorEventExit):
						surveyTrgIterVar.set(str(iterEvents))
		# 				while (trackerEventNum == eventNum): 
		# 					trackerEventNum = getTriggerCount(0)
						newEventVar.set(False)
						waitNewEventTask = rootTk.after(1, waitNewEvent)
						rootTk.wait_variable(newEventVar)
						getEvent(False)
		# 				eventNum = trackerEventNum
						iterEvents += 1
					if (lastCountEvents == countEvents):
# 						eventMetrics[idxBufSpd, idxTrgWin, idxFpgaTrgDly, idxTrgDly] = 0.0
						eventMetrics[idxBufSpd, idxTrgWin, idxTrgDly, idxFpgaTrgDly] = 0.0

					else:
# 						eventMetrics[idxBufSpd, idxTrgWin, idxFpgaTrgDly, idxTrgDly] = float(countChips - lastCountClusters) / float(countEvents - lastCountEvents)
						eventMetrics[idxBufSpd, idxTrgWin, idxTrgDly, idxFpgaTrgDly] = float(countChips - lastCountClusters) / float(countEvents - lastCountEvents)
						logging.info("eventMetrics[ %d , %d , %d , %d ] = %f", idxBufSpd, idxTrgWin, idxTrgDly, idxFpgaTrgDly, eventMetrics[idxBufSpd, idxTrgWin, idxTrgDly, idxFpgaTrgDly])
					
					if errorEventExit : return
					disableTrigger()
					
		
# 	logging.info("Eventmetrics = %d", eventMetrics[0][0])
	dotColors = ['w','g','r','c','m','y','k','b']
# 	dotTypes = ['o','+','x','v','s','p','H','D','8','^','.','1','2','3','4','*']
	dotTypes = ['o','+','x','v','s','p','H','D','8','^','.','1','2','3','4','*','o','+','x','v','s','p','H','D','8','^','.','1','2','3','4','*']
	lineTypes = ['--','-']
# 	maxPlots = len(rangeBufSpd) * len(rangeTrgWin) * len(rangeFpgaTrgDly)
# 	eventPlots = range(maxPlots)
# 	eventLabels = range(maxPlots)
	for idxBufSpd in rangeBufSpd:
		for idxTrgWin in rangeTrgWin:
# 			for idxFpgaTrgDly in rangeFpgaTrgDly :
			for idxTrgDly in rangeTrgDly:
# 				timeDly = list(map(lambda x: ((((idxBufSpd + 1) * x) - idxFpgaTrgDly) * 100), rangeTrgDly))
				timeDly = list(map(lambda x: ((((idxBufSpd + 1) * idxTrgDly) - x) * 100), rangeFpgaTrgDly))
	# 			logging.info("eventMetrics[ %d , %d , %d ] = %d", idxBufSpd, idxTrgWin, 0, eventMetrics[idxBufSpd, idxTrgWin, 0])
	# 			eventPlots[0 ] = plt.plot(rangeTrgDly, eventMetrics[idxBufSpd,idxTrgWin,:], 'ro-')#dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-')
	# 			ls = dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-'
	# 			logging.info("%r", ls)
# 				eventPlots[(idxBufSpd - minBufSpd) * (len(rangeFpgaTrgDly) * len(rangeTrgWin)) + (idxTrgWin - minTrgWin) ] = plt.plot(rangeTrgDly, eventMetrics[idxBufSpd,idxTrgWin,:], dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-', label= str('BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin))) #dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-')
# 				plt.plot(rangeTrgDly, eventMetrics[idxBufSpd,idxTrgWin,:], dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-', label= str('BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin))) #dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-')
# 				plt.plot(timeDly, eventMetrics[idxBufSpd,idxTrgWin,idxFpgaTrgDly,:], dotColors[idxBufSpd] + dotTypes[idxFpgaTrgDly] + lineTypes[idxTrgWin], label= str('BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin))) #dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-')
				error = ((idxBufSpd + 1) * (100 + idxTrgWin * 50))
				autoLabel = '_' #exclude from legend unless it meets the criteria in the next statement
# 				if (((idxFpgaTrgDly == rangeFpgaTrgDly[0]) & (idxTrgWin == rangeTrgWin[0])) | ((idxBufSpd == rangeBufSpd[-1]) & (idxTrgWin == rangeTrgWin[-1]))) : autoLabel = ''
# 				plt.errorbar(timeDly, eventMetrics[idxBufSpd,idxTrgWin,idxFpgaTrgDly,:], 0, error, dotColors[idxBufSpd] + dotTypes[idxFpgaTrgDly] + lineTypes[idxTrgWin], label= str(autoLabel + 'BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin) + ' FD ' + str(idxFpgaTrgDly))) 
				if (((idxTrgDly == rangeTrgDly[0]) & (idxTrgWin == rangeTrgWin[0])) | ((idxBufSpd == rangeBufSpd[-1]) & (idxTrgWin == rangeTrgWin[-1]))) : autoLabel = ''
				plt.errorbar(timeDly, eventMetrics[idxBufSpd,idxTrgWin,idxTrgDly,rangeFpgaTrgDly], 0, error, dotColors[idxBufSpd] + dotTypes[idxTrgDly] + lineTypes[idxTrgWin], label= str(autoLabel + 'BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin) + ' TD ' + str(idxTrgDly))) 
	# 			eventLabels[(idxBufSpd - minBufSpd) * len(rangeTrgWin) + (idxTrgWin - minTrgWin) ] = 'BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin)
# 	l1, l2 = plt.plot(rangeTrgDly, eventMetrics[0,:], 'bs-', rangeTrgDly, eventMetrics[1,:], 'ro-')
	plt.title("Detector Chips / Events (Sample: " + str(sampleEvents) + " events)")
# 	plt.figlegend(eventPlots, 'lower right')
	plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., fontsize='small')
	plt.xlabel("Delay in ns")
	plt.ylabel("Chips per Event")
	plt.show()
	
def surveyTest():	
# 	Survey and graph all possible combinations of trig delay and window
# 	global countEvents
# 	global countChips
# 	global eventNum
	global newEventVar
	global waitNewEventTask
# 	global bufSpeed, fpgaTrgDly
	global surveyTrgIterVar, surveyTrgDlyVar, surveyTrgWinVar, surveyBufSpdVar, surveyFpgaTrgDlyVar
	
	minBufSpd = surveyBuffSpdMinScl.get()
	maxBufSpd = surveyBuffSpdMaxScl.get() + 1
	rangeBufSpd= range(minBufSpd,maxBufSpd)
	minTrgWin = surveyTrgWinMinScl.get()
	maxTrgWin = surveyTrgWinMaxScl.get() + 1
	rangeTrgWin = range(minTrgWin,maxTrgWin)
	minFpgaTrgDly = surveyFpgaTrgDlyMinScl.get()
	maxFpgaTrgDly = surveyFpgaTrgDlyMaxScl.get() + 1
	rangeFpgaTrgDly = range(minFpgaTrgDly,maxFpgaTrgDly)
	minTrgDly = surveyTrgDlyMinScl.get()
	maxTrgDly = surveyTrgDlyMaxScl.get() + 1
	rangeTrgDly= range(minTrgDly, maxTrgDly)
	eventMetrics = np.zeros((maxBufSpd,maxTrgWin,maxFpgaTrgDly,maxTrgDly),dtype=float,order='C')
	sampleEvents = surveyTrgIterScl.get()
# 	for idxBufSpd in rangeBufSpd:
# 		for idxTrgWin in rangeTrgWin:
# 			for idxFpgaTrgDly in rangeFpgaTrgDly :
# 				timeDly = list(map(lambda x: ((((idxBufSpd + 1) * x) + idxFpgaTrgDly) * 100), rangeTrgDly))
# 				logging.info("timeDly[ %d , %d , %d] = %r", idxBufSpd, idxTrgWin, idxFpgaTrgDly, timeDly)

	for idxBufSpd in rangeBufSpd:
		surveyBufSpdVar.set(str(idxBufSpd))
		for idxTrgWin in rangeTrgWin:
			surveyTrgWinVar.set(str(idxTrgWin))
			for idxFpgaTrgDly in rangeFpgaTrgDly :
				surveyFpgaTrgDlyVar.set(str(idxFpgaTrgDly))
				for idxTrgDly in rangeTrgDly:
					surveyTrgDlyVar.set(str(idxTrgDly))
					
					eventMetrics[idxBufSpd, idxTrgWin, idxFpgaTrgDly, idxTrgDly] = float(idxBufSpd + 50 * idxTrgWin + idxFpgaTrgDly + idxTrgDly)
					
		
	dotColors = ['w','g','r','c','m','y','k','b']
	dotTypes = ['o','+','x','v','s','p','H','D','8','^','.','1','2','3','4','*']
	lineTypes = ['--','-']

	for idxBufSpd in rangeBufSpd:
		for idxTrgWin in rangeTrgWin:
			for idxFpgaTrgDly in rangeFpgaTrgDly :
				timeDly = list(map(lambda x: ((((idxBufSpd + 1) * x) + idxFpgaTrgDly) * 100), rangeTrgDly))
				error = ((idxBufSpd + 1) * (100 + idxTrgWin * 50))
				autoLabel = '_' #exclude from legend unless it meets the criteria in the next statement
				if (((idxFpgaTrgDly == rangeFpgaTrgDly[0]) & (idxTrgWin == rangeTrgWin[0])) | ((idxBufSpd == rangeBufSpd[-1]) & (idxTrgWin == rangeTrgWin[-1]))) : autoLabel = ''
				plt.errorbar(timeDly, eventMetrics[idxBufSpd,idxTrgWin,idxFpgaTrgDly,:], 0, error, dotColors[idxBufSpd] + dotTypes[idxFpgaTrgDly] + lineTypes[idxTrgWin], label= str(autoLabel + 'BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin) + ' FD ' + str(idxFpgaTrgDly))) 
				
# 				plt.plot(timeDly, eventMetrics[idxBufSpd,idxTrgWin,idxFpgaTrgDly,:], dotColors[idxBufSpd] + dotTypes[idxFpgaTrgDly] + lineTypes[idxTrgWin], label= str('BufS ' + str(idxBufSpd) + ' Win ' + str(idxTrgWin))) #dotColors[idxBufSpd] + dotTypes[idxTrgWin] + '-')
				
	plt.title("Detector Chips / Events (Sample: " + str(sampleEvents) + " events)")
	plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., fontsize='small')
	plt.xlabel("Delay in ns")
	plt.ylabel("Chips per Event")
	plt.show()

def fixRangeMin(value):	
# 	Make sure min and max on ranges fit
	
	minBufSpd = surveyBuffSpdMinScl.get()
	maxBufSpd = surveyBuffSpdMaxScl.get()
	minTrgWin = surveyTrgWinMinScl.get()
	maxTrgWin = surveyTrgWinMaxScl.get() 
	minTrgDly = surveyTrgDlyMinScl.get()
	maxTrgDly = surveyTrgDlyMaxScl.get()
	minFpgaTrgDly = surveyFpgaTrgDlyMinScl.get()
	maxFpgaTrgDly = surveyFpgaTrgDlyMaxScl.get()

	if (minBufSpd > maxBufSpd) :
		surveyBuffSpdMaxScl.set(minBufSpd)
	if (minTrgWin > maxTrgWin) :
		surveyTrgWinMaxScl.set(minTrgWin)
	if (minTrgDly > maxTrgDly) :
		surveyTrgDlyMaxScl.set(minTrgDly)
	if (minFpgaTrgDly > maxFpgaTrgDly) :
		surveyFpgaTrgDlyMaxScl.set(minFpgaTrgDly)
		
def fixRangeMax(value):	
# 	Make sure min and max on ranges fit
	
	minBufSpd = surveyBuffSpdMinScl.get()
	maxBufSpd = surveyBuffSpdMaxScl.get()
	minTrgWin = surveyTrgWinMinScl.get()
	maxTrgWin = surveyTrgWinMaxScl.get() 
	minTrgDly = surveyTrgDlyMinScl.get()
	maxTrgDly = surveyTrgDlyMaxScl.get()
	minFpgaTrgDly = surveyFpgaTrgDlyMinScl.get()
	maxFpgaTrgDly = surveyFpgaTrgDlyMaxScl.get()

	if (minBufSpd > maxBufSpd) :
		surveyBuffSpdMinScl.set(maxBufSpd)
	if (minTrgWin > maxTrgWin) :
		surveyTrgWinMinScl.set(maxTrgWin)
	if (minTrgDly > maxTrgDly) :
		surveyTrgDlyMinScl.set(maxTrgDly)
	if (minFpgaTrgDly > maxFpgaTrgDly) :
		surveyFpgaTrgDlyMinScl.set(maxFpgaTrgDly)



# def quitCmd() :
# # 	global waitNewEventTask
# # 	global eventPollingTask
# # 
# 	if eventPollingTask is not None : rootTk.after_cancel(eventPollingTask)
# 	if waitNewEventTask is not None : rootTk.after_cancel(waitNewEventTask)
# # 	if rootTk._job is not None : rootTk.after_cancel(rootTk._job)
# # 	quit()
# 	sys.exit()
			
# --------------- Main Program --------------------------------------------------------------
	
logFileName="AESOP_log.txt"
print "Opening log file under name ",logFileName
fileLevel=logging.DEBUG
consoleLevel=logging.INFO
setupLogging(logFileName,fileLevel,consoleLevel)

csvFileName = "AESOP_FirstStrips.csv"
print "Opening csv First Strips data file under name ",csvFileName
csvFile = open(csvFileName, 'w')
csvW = csv.writer(csvFile)
csvW.writerow(["Timestamp", "Event Number", "Board Address", "FirstStrips List"])

countTrackerChips = [0] * 7

logging.info("Running the AESOP Tracker Board Test Script %s" % time.ctime())

# create the Gui
rootTk= Tk()
rootTk.title(titleVer)
newEventVar = Tkinter.BooleanVar()
newEventVar.set(True)
trgNoticeVar = Tkinter.BooleanVar()
trgNoticeVar.set(False)
trgNoticeVar = Tkinter.BooleanVar()
trgNoticeVar.set(False)
trgEventWaitVar = Tkinter.BooleanVar()
trgEventWaitVar.set(False)
# Tk()
frameAL=Frame(rootTk)
# frameAL=Frame()

# frameAL.__init__(frameAL)
# frameAL.grid(row=2, column=2)
Grid.rowconfigure(rootTk, 0, weight=1)
Grid.columnconfigure(rootTk, 0, weight=1)

frameAL.grid(row=0, column=0, sticky=N+S+E+W)
gridAL=Frame(frameAL)
gridAL.grid(sticky=N+S+E+W, column=0, row=0, columnspan=1)
Grid.rowconfigure(frameAL, 0, weight=1)
Grid.columnconfigure(frameAL, 0, weight=1)

comSpin = Spinbox(frameAL, from_=1, to=20, width=2)
comBut = Button(frameAL, text="COM", command=setCOM)
comCloseBut = Button(frameAL, text="COM Close (after power cycle)", command=endCOM)
boardsSpin = Spinbox(frameAL, from_=1, to=7, width=1)
boardsBut = Button(frameAL, text="Boards", command=setBoards)
resetBut = Button(frameAL, text="Reset Boards", command=resetBoards)
asicOffBut = Button(frameAL, text="ASIC Power Off", command=asicOff)

configTDLbl = Label(frameAL, text="Trig Delay")
configTDSpin = Spinbox(frameAL, from_=0, to=31, width=2) #5 bits of trigger delay in register
configBut = Button(frameAL, text="Config Boards", command=configBoards)
configTWVar = IntVar()
configTWChk = Checkbutton(frameAL, text="Trig Win=1", variable=configTWVar)
configBSLbl = Label(frameAL, text="Buf Speed")
configBSSpin = Spinbox(frameAL, from_=0, to=7, width=1) #3 bits
configFDLbl = Label(frameAL, text="FPGA Delay")
configFDSpin = Spinbox(frameAL, from_=0, to=255, width=3) #5 bits of trigger delay in register
trgBut = Button(frameAL, text="Trigger", command=setTrg)
trgMenuLbl = Label(frameAL, text="Source")
trgVar = StringVar()
trgMenu = OptionMenu(frameAL,trgVar, *sorted(trgList.viewkeys()))
configIntTDSpin = Spinbox(frameAL, from_=0, to=255, width=3)
configIntTDSpinLbl = Label(frameAL, text="Int Delay")
configIntTWSpin = Spinbox(frameAL, from_=0, to=255, width=3)
configIntTWSpinLbl = Label(frameAL, text="Int Width")

intTrgMaskVar = [0]*7
intTrgMaskBox = [0]*7
for i in range(7) :
	color = "red"
	if i in [0, 4, 6] :
		color = "blue"
	intTrgMaskVar[i] = IntVar()
	intTrgMaskBox[i] = Checkbutton(frameAL, text="Mask " + str(i), variable=intTrgMaskVar[i], bg = color)
	intTrgMaskVar[i].set(True)
trgNonBendEndLbl = Label(frameAL, text="NonBending End =")
trgNonBendEndVar = StringVar()
trgNonBendEndMenu = OptionMenu(frameAL,trgNonBendEndVar, 4,6)
trgBendEndLbl = Label(frameAL, text="Bending End =")
trgBendEndVar = StringVar()
trgBendEndMenu = OptionMenu(frameAL,trgBendEndVar, 1,2,3,5)
trigWaitScl = surveyTrgIterScl = Scale(frameAL, from_=0, to=255, orient=HORIZONTAL, label="GO wait cycles")
intTrgDualVar = IntVar()
intTrgDualBox = Checkbutton(frameAL, text="Dual Trig", variable=intTrgDualVar)
intTrgDualVar.set(True)

eventPollingEnable = IntVar()
pollEnableChk = Checkbutton(frameAL, text="Poll Events", variable=eventPollingEnable, command=pollEnableChkCmd)
eventPlotEnable = IntVar()
plotEnableChk = Checkbutton(frameAL, text="Plot Events", variable=eventPlotEnable, command=plotEnableChkCmd)
eventCancelWaitBut = Button(frameAL, text="Cancel Event Wait", command=eventCancelWaitTask)
surveyTrgIterVar = StringVar()
surveyTrgIterLbl = Label(frameAL, textvariable=surveyTrgIterVar)
surveyTrgDlyVar = StringVar()
surveyTrgDlyLbl = Label(frameAL, textvariable=surveyTrgDlyVar)
surveyTrgWinVar = StringVar()
surveyTrgWinLbl = Label(frameAL, textvariable=surveyTrgWinVar)
surveyBufSpdVar = StringVar()
surveyBufSpdLbl = Label(frameAL, textvariable=surveyBufSpdVar)
surveyFpgaTrgDlyVar = StringVar()
surveyFpgaTrgDlyLbl = Label(frameAL, textvariable=surveyFpgaTrgDlyVar)
surveyTrgBut = Button(frameAL, text="Survey ASIC Trig Settings", command=surveyTrg)
surveyTrgIterScl = Scale(frameAL, from_=10, to=1000, orient=HORIZONTAL, label="Iterations/Sample")
surveyBuffSpdMinScl = Scale(frameAL, from_=0, to=7, orient=VERTICAL, command=fixRangeMin)
surveyBuffSpdMaxScl = Scale(frameAL, from_=0, to=7, orient=HORIZONTAL, label="Min|Buf Spd|Max", command=fixRangeMax)
surveyBuffSpdMaxScl.set(7)
surveyTrgWinMinScl = Scale(frameAL, from_=0, to=1, orient=VERTICAL, command=fixRangeMin)
surveyTrgWinMaxScl = Scale(frameAL, from_=0, to=1, orient=HORIZONTAL, label="Min|Trig Win|Max", command=fixRangeMax)
surveyTrgWinMaxScl.set(1)
surveyTrgDlyMinScl = Scale(frameAL, from_=0, to=31, orient=VERTICAL, command=fixRangeMin)
surveyTrgDlyMaxScl = Scale(frameAL, from_=0, to=31, orient=HORIZONTAL, label="Min|Trig Dly|Max", command=fixRangeMax)
surveyTrgDlyMaxScl.set(31)
surveyFpgaTrgDlyMinScl = Scale(frameAL, from_=0, to=255, orient=VERTICAL, command=fixRangeMin)
surveyFpgaTrgDlyMaxScl = Scale(frameAL, from_=0, to=255, orient=HORIZONTAL, label="Min|FPGA Dly|Max", command=fixRangeMax)
surveyFpgaTrgDlyMaxScl.set(255)
missedTBut = Button(frameAL, text="Get Missed", command=getMissedTrg)
missedTVar = StringVar()
missedTVar.set("Missed Triggers:    0")
missedTLbl = Label(frameAL, textvariable=missedTVar)
missedGoBut = Button(frameAL, text="Get Missed GO", command=getMissedGo)
missedGoVar = StringVar()
missedGoVar.set("Missed GO:    0")
missedGoLbl = Label(frameAL, textvariable=missedGoVar)
enableTBut = Button(frameAL, text="Enable Trig", command=enableTrigger)
disableTBut = Button(frameAL, text="Disable Trig", command=disableTrigger)
diagASICBut = Button(frameAL, text="Get ASIC Diag", command=getASICDiag)
stateErrBut = Button(frameAL, text="Get State & Error", command=getStateError)
checkASICBut = Button(frameAL, text="ASIC Error Codes", command=routineErrorCheck)

eventNumVar = StringVar()
eventNumVar.set("Event Num:    0")
eventNumLbl = Label(frameAL, textvariable=eventNumVar)
countChipsVar = StringVar()
countChipsVar.set("Total Chip Hits:    0")
countChipsLbl = Label(frameAL, textvariable=countChipsVar)
countTrackerChipsVar = StringVar()
countTrackerChipsVar.set("Chip Hits per Board: " + str(countTrackerChips))
countTrackerChipLbl = Label(frameAL, textvariable=countTrackerChipsVar)


# quitBut = Button(frameAL, text="QUIT", fg= "red", command=quitCmd)

currentRow = 0

comBut.grid(row=currentRow, column=0)
comSpin.grid(row=currentRow, column=1)
comCloseBut.grid(row=currentRow, column=2)

currentRow += 1
boardsBut.grid(row=currentRow, column=0)
boardsSpin.grid(row=currentRow, column=1)

currentRow += 1
resetBut.grid(row=currentRow, column=0)
configTDLbl.grid(row=currentRow, column=1)
asicOffBut.grid(row=currentRow, column=2)
configBSLbl.grid(row=currentRow, column=3)
configFDLbl.grid(row=currentRow, column=4)

currentRow += 1
configBut.grid(row=currentRow, column=0)
configTDSpin.grid(row=currentRow, column=1)
configTWChk.grid(row=currentRow, column=2)
configBSSpin.grid(row=currentRow, column=3)
configFDSpin.grid(row=currentRow, column=4)


currentRow += 1
trgMenuLbl.grid(row=currentRow, column=1)
configIntTDSpinLbl.grid(row=currentRow, column=2)
configIntTWSpinLbl.grid(row=currentRow, column=3)
for i in range(7) : 
	intTrgMaskBox[i].grid(row=currentRow, column=4+i)
	
currentRow += 1
trgBut.grid(row=currentRow, column=0)
trgMenu.grid(row=currentRow, column=1)
configIntTDSpin.grid(row=currentRow, column=2)
configIntTWSpin.grid(row=currentRow, column=3)
trigWaitScl.grid(row=currentRow, column=4)
trgNonBendEndLbl.grid(row=currentRow, column=5)
trgNonBendEndMenu.grid(row=currentRow, column=6)
intTrgDualBox.grid(row=currentRow, column=7)
trgBendEndLbl.grid(row=currentRow, column=8)
trgBendEndMenu.grid(row=currentRow, column=9)


currentRow += 1
pollEnableChk.grid(row=currentRow, column=0)
plotEnableChk.grid(row=currentRow, column=1)
surveyTrgIterLbl.grid(row=currentRow, column=2)
surveyTrgDlyLbl.grid(row=currentRow, column=3)
eventCancelWaitBut.grid(row=currentRow, column=4)
surveyTrgWinLbl.grid(row=currentRow, column=5)
surveyBufSpdLbl.grid(row=currentRow, column=7)
surveyFpgaTrgDlyLbl.grid(row=currentRow, column=9)

currentRow += 1
surveyTrgBut.grid(row=currentRow, column=0)
surveyTrgIterScl.grid(row=currentRow, column=1)
surveyTrgDlyMinScl.grid(row=currentRow, column=2)
surveyTrgDlyMaxScl.grid(row=currentRow, column=3)
surveyTrgWinMinScl.grid(row=currentRow, column=4)
surveyTrgWinMaxScl.grid(row=currentRow, column=5)
surveyBuffSpdMinScl.grid(row=currentRow, column=6)
surveyBuffSpdMaxScl.grid(row=currentRow, column=7)
surveyFpgaTrgDlyMinScl.grid(row=currentRow, column=8)
surveyFpgaTrgDlyMaxScl.grid(row=currentRow, column=9)

currentRow += 1
missedTBut.grid(row=currentRow, column=0)
missedTLbl.grid(row=currentRow, column=1)
enableTBut.grid(row=currentRow, column=2)
disableTBut.grid(row=currentRow, column=3)
missedGoBut.grid(row=currentRow, column=4)
missedGoLbl.grid(row=currentRow, column=5)
diagASICBut.grid(row=currentRow, column=6)
stateErrBut.grid(row=currentRow, column=7)
checkASICBut.grid(row=currentRow, column=8)

currentRow += 1

eventNumLbl.grid(row=currentRow, column=0)
countChipsLbl.grid(row=currentRow, column=1)
countTrackerChipLbl.grid(row=currentRow, column=2)

# testPlot()

# quitBut.grid(row=10, column=0)

# rootTk.after(1000, eventPolling)
rootTk.mainloop()

#TODO add current monitor button

