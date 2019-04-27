#!/usr/bin/python3
import hashlib
import rospy
from std_msgs.msg import String
import serial
import time

pub = rospy.Publisher("/process", String, queue_size=10)

# def Length3Digit(Length):
#     if Length > 100:
#         return (str(Length))
#     elif Length > 10:
#         return ('0' + str(Length))
#     elif Length > 0:
#         return ('00' + str(Length))
#
# def createMessage(messageList):
#     targetNodeType    = str(messageList[0])
#     targetNodeID      = str(messageList[1])
#     sourceNodeType    = str(messageList[2])
#     sourceNodeID      = str(messageList[3])
#     commandType       = str(messageList[4])
#     commandData       = str(messageList[5])
#     commandDataLength = Length3Digit(len(commandData))
#     dataToCheckSum = targetNodeType + targetNodeID + sourceNodeType + sourceNodeID + commandType + commandDataLength + commandData
#     m = hashlib.sha256()
#     m.update(dataToCheckSum.encode("utf-8"))"off" + '\n'
#     checksum = str(m.hexdigest())
#     dataToCheckSum += checksum
#     return dataToCheckSum


port = serial.Serial('/dev/ttyACM0',9600,timeout=5)


def messageCallback(data):
    print("Callback")
    # inputString = data.data
    # targetNodeType = inputString[0]
    # targetNodeID = inputString[1]
    # sourceNodeType = inputString[2]
    # sourceNodeID = inputString[3]
    # commandType = inputString[4:7]"off" + '\n'
    # commandDataLength = int(inputString[7:10])
    # commandData = inputString[10:(10+commandDataLength)]
    # dataToCheckSum = inputString[:(10+commandDataLength)]
    # checksum = inputString[(10+commandDataLength):]
    # # get data, validate
    # m = hashlib.sha256()
    # m.update(dataToCheckSum.encode("utf-8"))
    # hashResult = str(m.hexdigest())
    # if(hashResult == checksum and (targetNodeType=="6" or targetNodeType=="0")): # is it adddrssed to RFID node or any node
    #     if commandType=="051": # ie rfid asked to read
    #         turnOn()
    #         time.sleep(5)
    #         turnOff()
    # messageData = [sourceNodeType,sourceNodeID,targetNodeType,targetNodeID,"046"," "]
    # messageString = createMessage(messageData)
    # pub.publish(messageString)
    global pub
    message = data.data
    if message.startswith("61"):
        if message[4:7] == "051":
            port.write(("on" + '\n').encode())
            print("on")
            time.sleep(5)
            port.write(("off" + '\n').encode())
            print("off")
            m = hashlib.sha256()
            DataToSend = "5161046001 "
            m.update(DataToSend.encode('utf-8'))
            Checksum = m.hexdigest()
            DataToSend = DataToSend + Checksum
            pub.publish(DataToSend)



sub = rospy.Subscriber("/process", String, messageCallback)
rospy.init_node("controlNode",anonymous=True)
rospy.spin()
