#!/usr/bin/python3
import hashlib
import rospy
from std_msgs.msg import String
import serial

def Length3Digit(Length):
    if Length > 100:
        return (str(Length))
    elif Length > 10:
        return ('0' + str(Length))
    elif Length > 0:
        return ('00' + str(Length))

def createMessage(messageList):
    targetNodeType    = str(messageList[0])
    targetNodeID      = str(messageList[1])
    sourceNodeType    = str(messageList[2])
    sourceNodeID      = str(messageList[3])
    commandType       = str(messageList[4])
    commandData       = str(messageList[5])
    commandDataLength = Length3Digit(len(commandData))
    dataToCheckSum = targetNodeType + targetNodeID + sourceNodeType + sourceNodeID + commandType + commandDataLength + commandData
    m = hashlib.sha256()
    m.update(dataToCheckSum.encode("utf-8"))
    checksum = str(m.hexdigest())
    dataToCheckSum += checksum
    return dataToCheckSum


rospy.init_node("controlNode",anonymous=True)
pub = rospy.Publisher("/process", String, queue_size=10)
sub = rospy.Subscriber("/process", String, messageCallback)
port = serial.Serial('/dev/ttyACM0',9600,timeout=5)

def turnOn():
    port.write("on\n")

def turnOn():
    port.write("off\n")


def messageCallback(data):
    inputString = data.data
    targetNodeType = inputString[0]
    targetNodeID = inputString[1]
    sourceNodeType = inputString[2]
    sourceNodeID = inputString[3]
    commandType = inputString[4:7]
    commandDataLength = int(inputString[7:10])
    commandData = inputString[10:(10+commandDataLength)]
    dataToCheckSum = inputString[:(10+commandDataLength)]
    checksum = inputString[(10+commandDataLength):]
    # get data, validate
    m = hashlib.sha256()
    m.update(dataToCheckSum.encode("utf-8"))
    hashResult = str(m.hexdigest())
    if(hashResult == checksum and (targetNodeType=="9" or targetNodeType=="0")): # is it adddrssed to RFID node or any node
        if commandType=="042": # ie rfid asked to read
            turnOn()
            time.sleep(5)
            turnOff()
            messageData = [sourceNodeType,sourceNodeID,targetNodeType,targetNodeID,"046"," "]
            messageString = createMessage(messageData)
            pub.publish(messageString)
