import binascii
import string
import serial
import time
import logging
import select
from bitstring import BitArray
import select
import struct

# Globals
cntHitPackets = 0
cntHitPacketBytes = 0

# Python functions used for testing the AESOP-Lite tracker boards

# ---- The user script needs to call this to set up the logging to console and file ------

def setupLogging(logFileName,fileLevel,consoleLevel):
  logging.basicConfig(level=fileLevel,
                      format='%(asctime)s %(name)-4s %(levelname)-4s %(message)s',
                      datefmt='%m-%d %H:%M',
                      filename=logFileName,
                      filemode='w')
  # define a Handler which writes INFO messages or higher to the sys.stderr
  console = logging.StreamHandler()
  console.setLevel(consoleLevel)
  # set a format which is simpler for console use
  formatter = logging.Formatter('%(name)-4s: %(levelname)-4s %(message)s')
  # tell the handler to use this format
  console.setFormatter(formatter)
  # add the handler to the root logger
  logging.getLogger('').addHandler(console)
  logging.info("Logging will go to file %s with level %d and to console with level %d" % (logFileName,fileLevel,consoleLevel))

# ------------------------- List and string handling utilities --------------------------

def byteToBitString(char):
  return bin(int(binascii.hexlify(char), 16))[2:] 

def bitStringToChar(bitString):
  return chr(int(bitString, 2))

def parity(binaryString):
  tempBinaryString = ""
  if (type(binaryString) == list):
    for char in binaryString:
      tempBinaryString = tempBinaryString + byteToBitString(char)
    binaryString = tempBinaryString
  par = 0
  binaryString = int(binaryString, 2)
  while (binaryString):
    par = ~par
    binaryString = binaryString & (binaryString - 1)
  return (abs(par))
  
def getBinaryString(byteList):
  strReturn = ""
  for byte in byteList:
    strByte = bin(int(binascii.hexlify(byte),16))[2:]
    L2 = len(strByte)
    for i in range(8-L2):
      strByte = '0' + strByte      
    strReturn = strReturn + strByte
  return strReturn

  
def printBinaryString(byteList):
  logging.info(getBinaryString(byteList))
  
def printHexString(list):
  for byte in list:
    print binascii.hexlify(byte), 
    

# ------------------------------ UART Control and Communication -----------------------

def openCOM(portName):
  global ser
  ser = serial.Serial(portName, 115200, timeout=.2)
  
def closeCOM():
  global ser
  ser.close()
  
def sendByte(hexbyte):
  ser.write(hexbyte)
#  logging.debug(binascii.hexlify(hexbyte)) #hex(hexbyte)
  
def send(byteList):
  for byte in byteList:
    sendByte(byte)

def floatToBits(f):
    s= struct.pack('>f', f)
    return struct.unpack('>l', s)[0]

def readASICreg(): 
  length = getRegDumpLength()
  lengthReturned = length
  logging.debug('ASIC register read; dumping ' + str(lengthReturned) + ' bytes.')
  response = []
  nByte = 0
  while (length > 0):
    Byte = ser.read()
    if Byte != '':
        intByte = int(binascii.hexlify(Byte),16)
        strByte = bin(intByte)
        logging.debug('  ASIC register byte %d: %s  %d',nByte,strByte,intByte)
        nByte = nByte + 1
        response.append(Byte)
        length = length - 1
    else: 
        logging.info('  Empty byte')
        length = 0   

  responseStrng = getBinaryString(response)
  logging.debug('    Response=' + responseStrng)
  if len(responseStrng)>3: type = regType[responseStrng[1:4]]
  else: type = "000"
  logging.debug('    Register type = ' + type)
  
  if type in dacList:
    range = dacRange[responseStrng[5]]
    setting = responseStrng[6:13]
    logging.debug('    DAC range =  ' + range)
    logging.debug('    DAC setting= ' + setting + " = %d", int(setting,2))
    regReturn = responseStrng[5:13]
  else:
    if type in maskList:
      mask = responseStrng[5:69]
      logging.debug('    Mask = ' + mask)
      regReturn = mask
    else:
      if type == "Configuration Register":
        regReturn = responseStrng[5:27]
        logging.info('    Configuration register=' + regReturn)
      else:
        regReturn = '0000000000000000000000000000000000000000000000000000000000000000'
  return regReturn

dacList = ["Calibration DAC","Threshold DAC"] 
maskList = ["Data Mask","Trigger Mask","Calibration Mask"]
regType = {
"000" : "Unknown",
"001" : "Calibration DAC",
"010" : "Threshold DAC",
"011" : "Configuration Register",
"100" : "Data Mask",
"101" : "Trigger Mask",
"110" : "Calibration Mask",
"111" : "Unknown"}  
dacRange = {
"0" : "low",
"1" : "high"}
  
def readReg(): 
  length = getRegDumpLength()
  if length == 0:
    logging.error('readReg called, but no data are available')
    return [0,"00"]
  lengthReturned = length
  logging.debug('FPGA register read; dumping ' + str(length) + ' bytes.')
  nextByte = ser.read()
  ID = binascii.hexlify(nextByte)
  logging.debug('ID=' + ID)
  if ID == "c7": 
      logging.debug('Register read')  
      Byte = ser.read() 
      nData = int(binascii.hexlify(Byte),16)-1
      logging.debug('    Number of data bytes being returned is %d', nData) 
      firstByte = ser.read() 
      secondByte = ser.read() 
      cmdCnt = int(binascii.hexlify(firstByte+secondByte),16)
      logging.debug('    Command count=%d' ,cmdCnt)      
      brdAddress = binascii.hexlify(ser.read())
      logging.debug('    The board address is ' + brdAddress)
      cmdCode = binascii.hexlify(ser.read())
      logging.debug('    The command code was ' + cmdCode)
      returnReg = [cmdCnt,cmdCode]
      for nByte in range(nData):
          Byte = ser.read()
          returnReg += Byte
          if Byte != '':
              intByte = int(binascii.hexlify(Byte),16)
              strByte = bin(intByte)
              logging.debug('      Data Byte %d: %s  %d',nByte,strByte,intByte)
          else: logging.debug('      Blank byte encountered')
      endByte = binascii.hexlify(ser.read())
      logging.debug('    End byte = ' + endByte)
      return returnReg      
  else:
    if ID == "f1":
      logging.debug('  Command echo:')
      firstByte = ser.read()
      secondByte = ser.read()
      cmdCnt = int(binascii.hexlify(firstByte+secondByte),16)
      logging.debug('    Command count=%d',cmdCnt)
      cmdCode= binascii.hexlify(ser.read())
      logging.debug('    Command code was ' + cmdCode)
      return [cmdCnt,cmdCode]
    else:
      if ID == "ab":
          logging.error("Trigger Notice byte received by readReg")
          return [0,ID]
      else: 
          logging.error("Unrecognized packet type %s", ID)
          if ID != "":
            length = length - 1
            nByte = 0
            intByte = int(ID,16)
            strByte = bin(intByte)
            logging.info('  Packet Byte %d: %s  %d',nByte,strByte,intByte)
            nByte = nByte + 1  
            response = [nextByte]
            while (length > 0):
              Byte = ser.read()
              if Byte != '':
                intByte = int(binascii.hexlify(Byte),16)
                strByte = bin(intByte)
                logging.info('  Packet Byte %d: %s  %d',nByte,strByte,intByte)
                response.append(Byte)
                length = length - 1
              else: 
                logging.error('  Empty byte')
                length = 0
              nByte = nByte + 1
            responseStrng = getBinaryString(response)
            logging.info('    Response=' + responseStrng)
          return [0,ID]
  
def getRegDumpLength():
  firstByte = ser.read()
  if (firstByte != ""):
    return int(binascii.hexlify(firstByte), 16)
  else:
    return 0

def dumpUART():   #Dump out everything in the UART, just for debugging
  logging.info('dumpUART: dumping out the entire UART buffer:')
  nByte = 0
  while True:
    Byte = ser.read()
    if Byte != '':
        intByte = int(binascii.hexlify(Byte),16)
        strByte = bin(intByte)
        logging.debug('  UART Byte %d: %s  %d',nByte,strByte,intByte)
        nByte = nByte + 1    
    else: 
        logging.info('  Empty byte')
        break
  logging.info('  dumpUART: number of bytes read = %d',nByte)
    
def readEvt(): 
  length = getEvtDumpLength()
  lengthReturned = length
  logging.debug('ReadEvt: dumping ' + str(length) + ' packet bytes.')
  response = []
  nByte = 0
  error = False
  if lengthReturned == 0 : error = True
  while (length > 0): 
    Byte = ser.read()
    if Byte != '':
        intByte = int(binascii.hexlify(Byte),16)
        strByte = bin(intByte)
        logging.debug('  Packet Byte %d: %s  %d',nByte,strByte,intByte)
        nByte = nByte + 1
        response.append(Byte)
        length = length - 1
    else: 
        logging.error('  Empty byte')
        length = 0
        error = True
        
  if error or response == '':    
      logging.error('   No data found by readEvt:')  
      getStateVectors(0)
      getErrorCount(0,1)
      getErrorCount(0,2)
      getErrorCount(0,3)
      getErrorCount(0,4)
      getErrorCount(0,5)
      getErrorCount(0,6)
      getErrorCount(0,7)
      getErrorCount(0,8)

  return [response, lengthReturned]

def readTriggerNotice():
  logging.debug(' Entering readTriggerNotice')
  Byte = ser.read()
  trig = False
  if Byte != '':
    intByte = int(binascii.hexlify(Byte),16)
    strByte = bin(intByte)
    logging.debug(' Trigger notice Byte= ' + strByte)
    if strByte == '0b10101011': trig = True
  logging.debug(' readTriggerNotice returning %d',trig)
  return trig

def readEventWait():
  #logging.debug(' Entering readEventWait')
  send(["\x00","\x57","\x00"])
  length = getRegDumpLength()
  event = False
  for nByte in range(length) :
    Byte = ser.read()
    if (6 == nByte) : #The response data byte
      intByte = int(binascii.hexlify(Byte),16)
      strByte = bin(intByte)
      logging.debug(' readEventWait Byte= ' + strByte)
      if strByte == '0b1011001': event = True # ASCII 'Y' indicates event waiting
  #logging.debug(' readEventWait returning %d',event)
  return event
  
def readTriggerOutput():
  length = getEvtDumpLength()
  lengthReturned = length
  logging.debug('Dumping ' + str(length) + ' packet bytes.')
  response = []
  nByte = 0
  while (length > 0):
    Byte = ser.read()
    if Byte != '':
        intByte = int(binascii.hexlify(Byte),16)
        strByte = bin(intByte)
        logging.debug('  Packet Byte %d: %s  %d',nByte,strByte,intByte)
        nByte = nByte + 1
        response.append(Byte)
    else: logging.info('  Empty byte')
    length = length - 1
  if not (response == []):
     response = getBinaryString(response)
     if len(response) > 40:
       fpga = int(response[1:5],2)
       logging.debug('    FPGA = %d',fpga)
       startTime = int(response[29:41],2)
       TOT = int(response[41:53],2)
       hits = response[53:65]
       logging.debug('    start time = %d; TOT = %d; hits= %s' %(startTime,TOT,hits))
     else:
       hits=""
       startTime=0
       TOT=0  
  else: 
     hits=""
     startTime=0
     TOT=0  
  return [hits,startTime,TOT]
  
def getEvtDumpLength():
  firstByte = ser.read()
  if firstByte != "": 
    return int(binascii.hexlify(firstByte),16)
  else:
    return 0

# ------------ Basic FPGA commands ---------------------------------------------------------

def resetFPGA(Address):
  hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x04","\x00"])
  stuff = readReg()
  if stuff[1] != '04': logging.error("resetFPGA, wrong command code echoed")
  return stuff

def getCodeVersion(Address):
  if (Address < 7):
    hexStr = "0%x" % Address
    send([binascii.unhexlify(hexStr),"\x0a","\x00"])
    return readReg()

def getBoardID(Address):
  if (Address < 7):
    hexStr = "0%x" % Address
    send([binascii.unhexlify(hexStr),"\x73","\x00"])
    return readReg()
  
def resetFPGAconfig(Address):
  hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x03","\x00"])
  stuff = readReg()
  if stuff[1] != '03': logging.error("resetFPGAconfig, wrong command code echoed")
  return stuff

def getFPGAconfig(Address):
  if (Address < 7):
    hexStr = "0%x" % Address
    send([binascii.unhexlify(hexStr),"\x0b","\x00"])
    logging.info("FPGA configuration:")
    return readReg()
  
def ASICpowerON(Address):
  hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x08","\x00"])
  stuff = readReg()
  if stuff[1] != '08': logging.error("ASICpowerON, wrong command code echoed")
  return stuff
  
def ASICpowerOFF(Address):
  hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x09","\x00"])
  stuff = readReg()
  if stuff[1] != '09': logging.error("ASICpowerOFF, wrong command code echoed")
  return stuff

def setNumLayers(numLayers):
  hexStr = "0%x" % numLayers
  send(["\x07","\x0f","\x01",binascii.unhexlify(hexStr)])
  stuff = readReg()
  if stuff[1] != '0f': logging.error("setNumLayers, wrong command code echoed")
  return stuff

def getNumLayers(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
    send([binascii.unhexlify(hexAdr),"\x1f","\x00"])
    return readReg()

def getStateVectors(Address):     # For debugging only
  if (Address < 7):
    hexAdr = '0%x' % Address
    send([binascii.unhexlify(hexAdr),"\x76","\x00"])
    return readReg()
    
def triggerSetup(Address,delay,width):
   hexAddr = "0%x" % Address
   if delay>15: hexStr = "%x" % delay
   else: hexStr = "0%x" % delay
   if width>15: hexStr2 = "%x" % width
   else: hexStr2 = "0%x" % width
   logging.info("Setting the trigger delay to " + hexStr + " and width to " + hexStr2 + " for address " + hexAddr)
   send([binascii.unhexlify(hexAddr),"\x06","\x02",binascii.unhexlify(hexStr),binascii.unhexlify(hexStr2)])
   return readReg()  

def intTriggerSetup(Address,delay,width):
   hexAddr = "0%x" % Address
   if delay>15: hexStr = "%x" % delay
   else: hexStr = "0%x" % delay
   if width>15: hexStr2 = "%x" % width
   else: hexStr2 = "0%x" % width
   logging.info("Setting the trigger delay to " + hexStr + " and width to " + hexStr2 + " for address " + hexAddr)
   send([binascii.unhexlify(hexAddr),"\x61","\x02",binascii.unhexlify(hexStr2),binascii.unhexlify(hexStr)])
   return readReg()  

def intTriggerType(Address,value):
   hexAddr = "0%x" % Address
   if value=='and': hexVal = '\x00'
   else: hexVal = '\x01'
   logging.info("Setting the internal trigger logic to " + value + " for board %d",Address)
   send([binascii.unhexlify(hexAddr),"\x63","\x01",hexVal])
   return readReg()
   
def getTriggerSetup(Address):
  if (Address < 7):
    hexAddr = "0%x" % Address
    send([binascii.unhexlify(hexAddr),"\x07","\x00"])
    return readReg()
  else: logging.error("getTriggerSetup: bad address %s",Address)

def getIntTriggerSetup(Address):
  if (Address < 7):
    hexAddr = "0%x" % Address
    send([binascii.unhexlify(hexAddr),"\x71","\x00"])
    return readReg()
  else: logging.error("getIntTriggerSetup: bad address %s",Address)

def setTriggerMask(Address,Value):
  logging.info("Setting the trigger mask for board %d to %d" % (Address,Value))
  hexAddr = "0%x" % Address
  hexVal = "0%x" % Value
  send([binascii.unhexlify(hexAddr),"\x62","\x01",binascii.unhexlify(hexVal)])
  stuff = readReg()
  if stuff[1] != '62': logging.error("setTriggerMask, wrong command code echoed")
  return stuff
  
def setTriggerSource(source):
  logging.info("Setting the trigger source to %d",source)
  hexNum = "0%x" % source
  send(["\x07","\x64","\x01",binascii.unhexlify(hexNum)])
  return readReg()

def setTriggerEndLayer(Address,Value):
  logging.info("Setting the trigger end layer for board %d to %d" % (Address,Value))
  hexAddr = "0%x" % Address
  hexVal = "0%x" % Value
  send([binascii.unhexlify(hexAddr),"\x5A","\x01",binascii.unhexlify(hexVal)])
  return readReg()
  
def setGoClockCycles(cycles):
  logging.info("Setting the GO clock cycles to %d",cycles)
  if cycles>15: hexNum = "%x" % cycles
  else: hexNum = "0%x" % cycles
  send(["\x00","\x56","\x01",binascii.unhexlify(hexNum)])
  return readReg()

def setDualTriggers(Value):
  logging.info("Setting the redundant trigger chain to %d",Value)
  hexVal = "0%x" % Value
  send(["\x00","\x5b","\x01",binascii.unhexlify(hexVal)])
  return readReg()

def getGoClockCycles():
  logging.info("Reading the GO clock cycles.")
  send(["\x00","\x59","\x00"])
  return readReg()

def enableTrigger():
  send(["\x07","\x65","\x00"])
  if readReg()[1] == "ab":
    readReg()
    return True
  return False
  
def disableTrigger():
  send(["\x07","\x66","\x00"])
  stuff = readReg()
  if stuff[1] == "ab":
    logging.error("Trigger notice received in disableTrigger!!")
    return readReg()
  return stuff;

def sendTrigger(Address):  # No command echo results from this command!
    logging.info("Sending a random trigger to board %d", Address) 
    if Address == 7 : logging.info("Sending a random trigger to all boards")
    hexAddr = "0%x" % Address 
    send([binascii.unhexlify(hexAddr),"\x67","\x00"])
    
def getTriggerCount(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
    send([binascii.unhexlify(hexAdr),"\x68","\x00"])
    stuff=readReg()
    if stuff[1] == '68':
      logging.debug("getTriggerCount, Byte2=%s",getBinaryString(stuff[2]))
      logging.debug("getTriggerCount, Byte3=%s",getBinaryString(stuff[3]))
      numb = getBinaryString(stuff[2]) + getBinaryString(stuff[3])
      logging.debug("getTriggerCount, Count=%s",numb)
      numbInt= int(numb,2)
      return numbInt
    else: return 0
    
def getMissedTriggerCount(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
    send([binascii.unhexlify(hexAdr),"\x75","\x00"])
    return readReg()

def getMissedGoCount(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x58","\x00"])
  return readReg()

def getTriggersASIC(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x54","\x00"])
  return readReg()

def getASICBufferOverflows(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x55","\x00"])
  return readReg()

def getEventsStreamed(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x5c","\x00"])
  return readReg()

def getEventsDumped(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x6a","\x00"])
  return readReg()

def getEventsAccepted(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x6b","\x00"])
  return readReg()

def getErrorCount(Address,Code):
  if (Address < 7):
    logging.info(" Getting the error count for board %d, error code %d" % (Address,Code))
    hexAdr = '0%x' % Address
    hexCode = '0%x' % Code
    send([binascii.unhexlify(hexAdr),"\x77","\x01",binascii.unhexlify(hexCode)])
    return readReg()


def measureTriggerRate(Address):
  if (Address < 7):
    logging.info("  Measuring the trigger rate for board %d" % (Address))
    hexAdr = '0%x' % Address
    send([binascii.unhexlify(hexAdr),"\x6C","\x00"])
  
def getTriggerRate(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
    send([binascii.unhexlify(hexAdr),"\x6D","\x00"])
    stuff=readReg()
    if stuff[1] == '6d':
      logging.info("getTriggerRate, Byte2=%s",getBinaryString(stuff[2]))
      logging.info("getTriggerRate, Byte3=%s",getBinaryString(stuff[3]))
      numb = getBinaryString(stuff[2]) + getBinaryString(stuff[3])
      logging.info("getTriggerRate, Count=%s",numb)
      numbInt= int(numb,2)
      return numbInt
    else: return 0

def getErrorReturnByte(Address):
  if (Address < 7):
    hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x78","\x00"])
  return readReg()

# def getErrorStateVectors(Address):
#   if (Address < 7):
#     hexAdr = '0%x' % Address
#   send([binascii.unhexlify(hexAdr),"\x79","\x00"])
#   return readReg()
    
# ------------ ASIC commands ---------------------------------------------------------------

def ASICinitialize(Address):  # Set the configuration register, threshold DAC, and data mask to the default
  hexAdr = '0%x' % Address
  send([binascii.unhexlify(hexAdr),"\x26","\x00"])
  return readReg()

def ASICsoftReset(Address,Chip):
  hexAdr = '0%x' % Address
  if Chip>15: hexStr = '%x' % Chip
  else: hexStr = '0%x' % Chip  
  send([binascii.unhexlify(hexAdr),"\x0c","\x01",binascii.unhexlify(hexStr)])
  return readReg()

def ASIChardReset(Address,Chip):
  hexAdr = '0%x' % Address
  if Chip>15: hexStr = '%x' % Chip
  else: hexStr = '0%x' % Chip  
  send([binascii.unhexlify(hexAdr),"\x05","\x01",binascii.unhexlify(hexStr)])
  return readReg()

def CalibrationStrobe(Address,Chip,TrgDly,TrgTag,FPGA):
  hexAdr = '0%x' % Address
  if Chip>15: hexStr = '%x' % Chip
  else: hexStr = '0%x' % Chip  
  hexFPGA = '0%x' % FPGA   # The FPGA that will send out TOT information from the trigger
  sTag = binNumb[TrgTag][3:5]
  sDly = bin(TrgDly)[2:]
  Ln = len(sDly)
  for i in range(6-Ln):
    sDly= '0' + sDly
  logging.debug('CalibrationStrobe:        sDly='+sDly)
  Strng = sDly + sTag
  logging.debug('CalibrationStrobe:        Strng='+Strng)
  Data= int(Strng,2)
  if Data>15: hexByte= '%x' % Data
  else: hexByte= '0%x' % Data
  logging.debug("    Calibration strobe command for address " + hexAdr + " for chip(s) " + hexStr + " data=" + hexByte + " FPGA=" + hexFPGA)
  send([binascii.unhexlify(hexAdr),"\x02","\x03",binascii.unhexlify(hexStr),binascii.unhexlify(hexByte),binascii.unhexlify(hexFPGA)])
  time.sleep(0.01)
  stuff = readTriggerOutput()
  return stuff

def ReadTkrEvent(tag,cal,verbose):
  global cntHitPackets, cntHitPacketBytes  
  if not cal:
    send(["\x07","\x01","\x01","\x00"])
  else:
    Value = tag + 4
    hexStr = '0%x' % Value
    logging.debug('  Sending data argument ' + hexStr + ' for the read event command')
    send(["\x07","\x01","\x01",binascii.unhexlify(hexStr)])
  stuff= readEvt()
  boardData = []
  if stuff[1]==0:
    logging.info('ReadTkrEvent: nothing returned')
    return [boardData,-1]
  response= stuff[0]
  if len(response)>=5:
    strByte0 = bin(int(binascii.hexlify(response[0]),16)) 
    strByte = bin(int(binascii.hexlify(response[1]),16)) 
    strByte2 = bin(int(binascii.hexlify(response[2]),16))
  else:
    logging.error('   ReadTkrEvent:  header response is too short, length=%d',len(response))
    strByte0 = ''
    strByte = ''
    strByte2 = '                                                        '
  if strByte0 != '0b11010011': logging.error('Wrong start code ' + strByte0  + ' detected in event header')
  strByte2 = strByte2[2:]
  L2 = len(strByte2)
  for i in range(8-L2):
    strByte2 = '0' + strByte2
  strEvtNum = strByte[2:] + strByte2
  EvtNum = int(strEvtNum,2)
  if verbose: logging.info('  Event number %d',EvtNum)  
  cmdCount = int(binascii.hexlify(response[3]),16)
  if verbose: logging.info('  Command count= %d',cmdCount)
  nDataPacks = int(binascii.hexlify(response[4]),16) % 32 #exclude 2 MSB to indicate bending and non-bending triggers
  if verbose: logging.info('  Expecting %d data packets...',nDataPacks)
  if nDataPacks > 7:
    logging.error('  Too many data packets specified:  n=%d',nDataPacks)
    return [boardData,-2]
  triggers = int(binascii.hexlify(response[4]),16) & 192 #mask off the trigger 2 MSB
  logging.info("Internally Triggered by %s", {192:"both planes", 128:"bending plane", 64:"non-bending plane", 0:"NO plane"}[triggers] )
  for i in range(nDataPacks):
    stuff= readEvt()
    if stuff[1]==0:
      logging.error('ReadTkrEvent: no data packet returned for i=%d',i)
      getStateVectors(i)
      getErrorCount(i,1)
      getErrorCount(i,2)
      getErrorCount(i,3)
      getErrorCount(i,4)
      getErrorCount(i,5)
      getErrorCount(i,6)
      getErrorCount(i,7)
      getErrorCount(i,8)
      return [boardData,-3]
    response= getBinaryString(stuff[0])
    chkStrng = '1' + response # add the start bit back in, as it is included in the CRC
    chkStrng = chkStrng.rstrip("0") # remove any trailing zeroes
    chkStrng = chkStrng[0:len(chkStrng)-2] # remove the 11 from the end of the string
    if not checkCRC6(chkStrng):
      logging.info(' CRC error detected in event response %d ' + response,i)
    logging.debug('    Response for packet %d is ' + response,i)
    numberOfChips, firstStripChip, FPGAaddress = ParseASIChitList(response, verbose)
    thisBoardData = {'nChips': numberOfChips, 'firstStrip': firstStripChip, 'address': FPGAaddress}
    boardData.append(thisBoardData)
    cntHitPackets += 1 #update hit packet counters
    cntHitPacketBytes += stuff[1]
  return [boardData,0]



# ---------------------------------- CRC Error checking ---------------------------------

def CRC6(bitString): # 1'100101
  divisor = "1100101"
  result = bitString 
  for i in range(len(result) - len(divisor)+1):
    #print result#result[0:(len(result)-len(divisor)+1)] , result[(len(result)-len(divisor)+1):]
    if (result[i] == "1"):
      for j in range(len(divisor)):
          if (result[i+j] == divisor[j]):
            result = result[0:i+j] + "0" + result[i+j+1:len(result)]
          else:
            result = result[0:i+j] + "1" + result[i+j+1:len(result)]
    #if (i == len(result)-len(divisor)-1):
      #print "-1", result[len(result)-6:],
    #print " "*i+divisor
  #print result
  #print CRC0
  return result[len(result)-6:]
  
def checkCRC6(bitString):
  #print bitString
  #print bitString[0:(len(bitString) - 6)]
  #print CRC6(bitString[0:(len(bitString) - 6)])
  #print bitString[(len(bitString) - 6):len(bitString)]
  if (CRC6(bitString[0:(len(bitString) - 6)]) == bitString[(len(bitString) - 6):len(bitString)]):
    return True
  else:
    return False    


def getHitPacketCounts():
  return [cntHitPackets, cntHitPacketBytes]

def ParseASIChitList(bitString, verbose):
  pointer = 0    
  firstStripChip = [] 
  if bitString == "":
    logging.error("ASIC hit list is empty")
    return -1, firstStripChip, 0
  if bitString[pointer:pointer+8] != "11100111": 
    logging.error("ASIC hit list does not begin with 11100111")
    return -1, firstStripChip, 0
  else:
    #logging.info("Identifier byte = %s", bitString[0:8])
    pointer = pointer + 9
    FPGAaddress = int(bitString[pointer:pointer+7],2)
    if verbose: logging.info("ASIC hit list for FPGA address %d",FPGAaddress)
    pointer = pointer + 7
    FPGAheader = bitString[pointer:pointer+12]
    pointer = pointer + 12
    numberOfChips = int(FPGAheader[8:12],2)
    if verbose: logging.info("    Event tag= %d,  Error flag= %s, Number of chips= %d" % (int(FPGAheader[0:7],2),FPGAheader[7],numberOfChips))
    if (numberOfChips != 0):
      for chip in range(numberOfChips):
        if len(bitString) < pointer + 12: continue
        asicHead=parseASICHeader(bitString[pointer:pointer+12])
        numberOfClusters = asicHead["numberOfClusters"]
        chipNum= int(asicHead["address"],2)
        if chipNum>12: logging.error("Chip number %d > 12",chipNum)
        if verbose: logging.info("    Chip %d: %d clusters, Overflow=%s, Error=%s, Parity error=%s" % (chipNum,numberOfClusters,asicHead["overflowBit"],asicHead["errorBit"],asicHead["parityErrorBit"]))
        firstStripList = printChipClusters(bitString[pointer:(pointer+12+numberOfClusters*12)], verbose)
        for item in firstStripList:
            firstStripChip.append(64*(chipNum+1) - item)                 #append all the first strip hit, for each cluster, for each chip 
        pointer = pointer + 12 + numberOfClusters*12
    return numberOfChips, firstStripChip, FPGAaddress

def printChipClusters(bitString, verbose): # takes a chip header and following clusters
  firstStripList = []                        #list of all the first strip hit list, to be appended over each cluster and each chip hit
  if bitString == "":
    if verbose: logging.info("      Empty cluster list. . .")
    return firstStripList
  pointer = 12
  nclust = getChipNumberOfClusters(bitString)
  if len(bitString) != 12 + 12*nclust:
    logging.error("       Wrong length cluster list. . .")
    return firstStripList
  for cluster in range(nclust):
    firstStrip = getChipFirstStrip(bitString[(pointer):(pointer+12)])
    ClusterWidth = getChipClusterWidth(bitString[pointer:pointer+12])
    clusterLoc = firstStrip + 0.5 + (ClusterWidth-1.0)/2.
    if verbose: logging.info("      Cluster width=%d   First strip=%d" % (ClusterWidth,firstStrip))
    firstStripList.append(clusterLoc)
    pointer = 12 + pointer
    #if verbose: logging.info("First Strip List is %s", firstStripList)
  return  firstStripList
  
  
def getChipClusterWidth(bitString): #
  return int(bitString[0:6],2) + 1
  
def getChipFirstStrip(bitString):
  return int(bitString[6:12], 2)
  
# -------------------------------------- ASIC Header --------------------------------

def parseASICHeader(bitString):
  return {
  "overflowBit": getChipOverflowBit(bitString), 
  "numberOfClusters": getChipNumberOfClusters(bitString),
  "errorBit": getChipErrorBit(bitString),
  "parityErrorBit": getChipParityErrorBit(bitString),
  "address": getChipDataAddress(bitString)
  }

def getChipOverflowBit(bitString):
  return bitString[0]
  
def getChipNumberOfClusters(bitString):
  return int(bitString[2:6], 2)
  
def getChipErrorBit(bitString): # takes ASIC header
  return bitString[6]
  
def getChipParityErrorBit(bitString):
  return bitString[7]
  
def getChipDataAddress(bitString): #returns a string address, map to strips
  return bitString[8:12]

# ------------ i2c monitoring functions ----------------------------------------------------

def powerDownTemperature(Address):
  logging.info("Powering down the TMP100 on board %d", Address)
  if Address>15: hexAddress = '%x' % Address
  else: hexAddress = '0%x' % Address
  i2cAddress = int(i2cAddresses['temp'],2)
  hexi2cAddress = '%x' % i2cAddress  
  logging.debug("  Set the i2c pointer register to x01 and load the TMP100 configuration")
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x01","\x01","\x00"])
  return readReg()
  
def getTemperature(Address):
  logging.info("Reading the temperature on board %d", Address)
  if Address>15: hexAddress = '%x' % Address
  else: hexAddress = '0%x' % Address
  i2cAddress = int(i2cAddresses['temp'],2)
  hexi2cAddress = '%x' % i2cAddress  
  logging.debug("  Set the i2c pointer register to x01 and load the TMP100 configuration")
  # Set config reg to 01100000,  the two 1's set the resolution to high
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x01","\x60","\x00"])
  readReg()
  time.sleep(0.1)
  logging.debug("  Set the i2c pointer register to x00 for temperature")
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x00","\x00","\x00"])   #Setting the pointer register to 2 for bus voltage
  readReg()   #Command echo
  time.sleep(0.1)
  send([binascii.unhexlify(hexAddress),"\x46","\x01",binascii.unhexlify(hexi2cAddress)])
  time.sleep(0.1)
  response = getBinaryString(readEvt()[0])
  L = len(response)
  Temperature = BitArray(bin=response[0:L-8])
  Tcelsius = (0.25/4.0)*(Temperature.int/16.)
  logging.info('  Temperature for board %d=%.2f degrees' % (Address,Tcelsius))
  intResponse = int(response,2)
  logging.debug("  response=%s %d" % (response,intResponse))
  logging.debug("  Set the i2c pointer register to x01 and reset the TMP100 configuration")  
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x01","\x61","\x00"])
  readReg()
  return intResponse
  
def powerOffIna(Address,item):
  logging.info("Powering off the ina226 chip for supply " + item + " on board %d", Address)
  if Address>15: hexAddress = '%x' % Address
  else: hexAddress = '0%x' % Address
  i2cAddress = int(i2cAddresses[item],2)
  hexi2cAddress = '%x' % i2cAddress  
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x00","\x41","\x20"]) 
  readReg()  
  time.sleep(0.1)
  send([binascii.unhexlify(hexAddress),"\x46","\x01",binascii.unhexlify(hexi2cAddress)])
  time.sleep(0.01)
  response = getBinaryString(readEvt()[0])
  logging.info("  Response from reading the ina226 configuration=" + response)
  return response
  
def getBusVoltage(Address,item):
  logging.info("Reading the bus voltage for supply " + item + " on board %d", Address)
  if Address>15: hexAddress = '%x' % Address
  else: hexAddress = '0%x' % Address
  i2cAddress = int(i2cAddresses[item],2)
  hexi2cAddress = '%x' % i2cAddress  
  logging.debug("  Set the i2c pointer register to x02")
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x02","\x00","\x00"])   #Setting the pointer register to 2 for bus voltage
  readReg()   #Command echo
  time.sleep(0.01)
  logging.debug("  Read the i2c bus voltage for " + item + " on board %d", Address)
  send([binascii.unhexlify(hexAddress),"\x46","\x01",binascii.unhexlify(hexi2cAddress)])
  time.sleep(0.01)
  response = getBinaryString(readEvt()[0])
  if len(response) > 7:
    L = len(response)
    intResponse = int(response[0:L-8],2)
    logging.debug("  response=%s %d" % (response,intResponse))
    busVoltage = 1.25*intResponse/1000.
    logging.info("  Bus voltage = %1.3f volts",busVoltage)
  else: 
    logging.info("  Bus voltage response = " + response)
    busVoltage = -999.
  return busVoltage

def i2cReset(Address):
  hexAddress = "0%x" % Address
  logging.info("Resetting the i2c state machine on board %d",Address)
  send([binascii.unhexlify(hexAddress),"\x49","\x00"])
  return readReg()

def i2cErrors(Address):
  hexAddress = "0%x" % Address
  send([binascii.unhexlify(hexAddress),"\x48","\x00"])
  logging.info("i2c error code readback for board %d",Address)
  return readReg()
  
def getShuntCurrent(Address,item):
  logging.info("Reading the shunt voltage for supply " + item + " on board %d", Address)
  if Address>15: hexAddress = '%x' % Address
  else: hexAddress = '0%x' % Address
  i2cAddress = int(i2cAddresses[item],2)
  hexi2cAddress = '%x' % i2cAddress  
  logging.debug("  Set the i2c pointer register to x01")
  send([binascii.unhexlify(hexAddress),"\x45","\x04",binascii.unhexlify(hexi2cAddress),"\x01","\x00","\x00"])   #Setting the pointer register to 1 for shunt voltage
  readReg()   #Command echo
  time.sleep(0.01)
  logging.debug("  Read the i2c shunt voltage for " + item + " on board %d", Address)
  send([binascii.unhexlify(hexAddress),"\x46","\x01",binascii.unhexlify(hexi2cAddress)])
  time.sleep(0.01)
  response = getBinaryString(readEvt()[0])
  logging.debug("  response=%s" % response)
  if response == "":
    logging.error("  No response for the shunt voltage.")
    return 0
  L = len(response)
  intResponse = int(response[0:L-8],2)
  logging.debug("  response=%s %d" % (response,intResponse))
  shuntVoltage = 2.5*intResponse/1000000.
  if item == 'bias100': R=100.
  else: R=0.03
  shuntCurrent = shuntVoltage*1000./R
  if item == 'bias100':
    V = shuntVoltage * 1000.
    I = shuntCurrent * 1000.
    logging.info("  Shunt voltage = %2.4f mV; Shunt current = %3.1f microamps" % (V,I))
  else: logging.info("  Shunt voltage = %1.4f volts; Shunt current = %3.1f milliamps" % (shuntVoltage,shuntCurrent))
  return shuntVoltage  

i2cAddresses = {
'flash18' : "1000101",
'fpga12'  : "1000000",
'digi25'  : "1000001",
'i2c33'   : "1000010",
'analog21': "1000100", 
'analog33': "1000011",
'bias100' : "1000110",
'temp'    : "1001000"
}

  
# ------------ ASIC register routines: -----------------------------------------------------

def loadCalDAC(Address,Chip,Range,Value):
  hexAdr = '0%x' % Address
  if Chip<16: hexStr = '0%x' % Chip  
  else: hexStr = '%x' % Chip  
  setting = Range*128 + Value
  if setting > 15: hexStr2 = '%x' % setting
  else: hexStr2 = '0%x' % setting
  send([binascii.unhexlify(hexAdr),"\x10","\x02",binascii.unhexlify(hexStr),binascii.unhexlify(hexStr2)])
  return readReg()

def readCalDAC(Address,chip):
  hexAdr = '0%x' % Address
  hexStr = '0%x' % chip  
  send([binascii.unhexlify(hexAdr),"\x20","\x01",binascii.unhexlify(hexStr)])
  response = readASICreg()
  return response

def loadThrDAC(Address,chip,Range,Value):
  hexAdr = '0%x' % Address
  if chip<16: hexStr = '0%x' % chip  
  else: hexStr = '%x' % chip
  setting = Range*128 + Value
  if setting > 15: hexStr2 = '%x' % setting
  else: hexStr2 = '0%x' % setting
  send([binascii.unhexlify(hexAdr),"\x11","\x02",binascii.unhexlify(hexStr),binascii.unhexlify(hexStr2)])
  return readReg()

def readThrDAC(Address,chip):
  hexAdr = '0%x' % Address
  hexStr = '0%x' % chip  
  send([binascii.unhexlify(hexAdr),"\x21","\x01",binascii.unhexlify(hexStr)])
  response = readASICreg()
  return response

def loadCalMask(Address,Chip,Mask):
  hexAdr = '0%x' % Address
  if Chip<16: hexStr = '0%x' % Chip  
  else: hexStr = '%x' % Chip
  hexByte = []
  for i in range(8):
    Byte = int(Mask[i*8:i*8+8],2)
    if Byte >15: hexByte.append('%x' % Byte)
    else: hexByte.append('0%x' % Byte)
  send([binascii.unhexlify(hexAdr),"\x15","\x09",binascii.unhexlify(hexStr),
          binascii.unhexlify(hexByte[0]),binascii.unhexlify(hexByte[1]),binascii.unhexlify(hexByte[2]),binascii.unhexlify(hexByte[3]),
          binascii.unhexlify(hexByte[4]),binascii.unhexlify(hexByte[5]),binascii.unhexlify(hexByte[6]),binascii.unhexlify(hexByte[7])])
  return readReg()

def readCalMask(Address,Chip):
  hexAdr = '0%x' % Address
  hexStr = '0%x' % Chip  
  send([binascii.unhexlify(hexAdr),"\x25","\x01",binascii.unhexlify(hexStr)])
  response = readASICreg()
  logging.debug("Calibration mask read response=%s",response) 
  return response  

def loadDataMask(Address,Chip,Mask):
  hexAdr = '0%x' % Address
  if Chip<16: hexStr = '0%x' % Chip  
  else: hexStr = '%x' % Chip  
  hexByte = []
  for i in range(8):
    Byte = int(Mask[i*8:i*8+8],2)
    if Byte >15: hexByte.append('%x' % Byte)
    else: hexByte.append('0%x' % Byte)
  send([binascii.unhexlify(hexAdr),"\x13","\x09",binascii.unhexlify(hexStr),
          binascii.unhexlify(hexByte[0]),binascii.unhexlify(hexByte[1]),binascii.unhexlify(hexByte[2]),binascii.unhexlify(hexByte[3]),
          binascii.unhexlify(hexByte[4]),binascii.unhexlify(hexByte[5]),binascii.unhexlify(hexByte[6]),binascii.unhexlify(hexByte[7])])
  return readReg()

def readDataMask(Address,Chip):
  hexAdr = '0%x' % Address
  hexStr = '0%x' % Chip  
  send([binascii.unhexlify(hexAdr),"\x23","\x01",binascii.unhexlify(hexStr)])
  response = readASICreg()
  logging.debug("Data mask read response=%s",response) 
  return response    
  
def loadTrgMask(Address,Chip,Mask):
  hexAdr = '0%x' % Address
  if Chip<16: hexStr = '0%x' % Chip  
  else: hexStr = '%x' % Chip
  hexByte = []
  for i in range(8):
    Byte = int(Mask[i*8:i*8+8],2)
    if Byte >15: hexByte.append('%x' % Byte)
    else: hexByte.append('0%x' % Byte)
  send([binascii.unhexlify(hexAdr),"\x14","\x09",binascii.unhexlify(hexStr),
          binascii.unhexlify(hexByte[0]),binascii.unhexlify(hexByte[1]),binascii.unhexlify(hexByte[2]),binascii.unhexlify(hexByte[3]),
          binascii.unhexlify(hexByte[4]),binascii.unhexlify(hexByte[5]),binascii.unhexlify(hexByte[6]),binascii.unhexlify(hexByte[7])])
  return readReg()

def readTrgMask(Address,Chip):
  hexAdr = '0%x' % Address
  hexStr = '0%x' % Chip  
  send([binascii.unhexlify(hexAdr),"\x24","\x01",binascii.unhexlify(hexStr)])
  response = readASICreg()
  logging.debug("Trigger mask read response=%s",response) 
  return response    
  
def loadConfigReg(Address,Chip,Polarity,oneShot,Gain,shapingTime,bufSpeed,TrgDly,TrgWin,Drive,MxClus):
  # All of the arguments are assumed to be integers within the proper ranges
  hexAdr = '0%x' % Address
  if Chip<16: hexStr = '0%x' % Chip  
  else: hexStr = '%x' % Chip  
  sPolarity=bin(Polarity)[2]
  sOneShot=bin(oneShot)[2]
  sGain=bin(Gain)[2]
  sShapingTime=bin(shapingTime)[2]
  sBufSpeed=binNumb[bufSpeed][2:5]
  sTrgDly=binNumb[TrgDly]
  sTrgWin=bin(TrgWin)[2]
  sDrive=binNumb[Drive][3:5]
  sMxClus=binNumb[MxClus][1:5]
  #Concatenate all the parts into one 24-bit string (the last 5 bits are not used)
  Setting=sMxClus + sDrive + sTrgWin + sTrgDly + sBufSpeed + sShapingTime + sGain + sOneShot + sPolarity + '00000'
  logging.info('   Setting the configuration of chip %d to ' + Setting, Chip)
  s1hx=hex(int(Setting[0:8],2))[2:4]
  if len(s1hx)<2: s1hx= '0' + s1hx
  s2hx=hex(int(Setting[8:16],2))[2:4]
  if len(s2hx)<2: s2hx= '0' + s2hx
  s3hx=hex(int(Setting[16:24],2))[2:4]
  if len(s3hx)<2: s3hx= '0' + s3hx
  logging.debug("   s1=" + s1hx + " s2=" + s2hx + " s3=" + s3hx)
  s1=binascii.unhexlify(s1hx)   #First byte
  s2=binascii.unhexlify(s2hx)   #Second byte
  s3=binascii.unhexlify(s3hx)   #Third byte
  send([binascii.unhexlify(hexAdr),"\x12","\x04",binascii.unhexlify(hexStr),s1,s2,s3])
  readReg()
  return Setting

def readConfigReg(Address,Chip):
  hexAdr = '0%x' % Address
  hexStr = '0%x' % Chip  
  send([binascii.unhexlify(hexAdr),"\x22","\x01",binascii.unhexlify(hexStr)])
  response = readASICreg()
  #The first 3 bits of the register are the error bits
  logging.debug("Configuration register read response=%s",response) 
  return response      
  
binNumb = {   #Dumb translation from decimal to 5-bit binary string
0 : '00000', 1 : '00001', 2 : '00010', 3 : '00011', 4 : '00100', 5 : '00101', 6 : '00110', 7 : '00111', 
8 : '01000', 9 : '01001', 10 : '01010', 11 : '01011', 12 : '01100', 13 : '01101', 14 : '01110', 15 : '01111',
16 : '10000', 17 : '10001', 18 : '10010', 19 : '10011', 20 : '10100', 21 : '10101', 22 : '10110', 23 : '10111', 
24 : '11000', 25 : '11001', 26 : '11010', 27 : '11011', 28 : '11100', 29 : '11101', 30 : '11110', 31 : '11111'}
