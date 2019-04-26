#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import sys
import serial
from std_msgs.msg import String
import math
import numpy as np
x = []
y = []
serialList = []
PtIndex = 0
pub = None
#print("this is a test")
rospy.init_node('TransportNode', anonymous=True)
pub = rospy.Publisher('/PlatformError', String, queue_size=10)
time.sleep(2)
#pub.publish("Starting up")
#print("starting up")
try:
	serialList.append(serial.Serial('/dev/ttyACM0', 115200,timeout = 1))
	#pub.publish("Flag 1")
except:
	pass

try:
	serialList.append(serial.Serial('/dev/ttyACM1', 115200,timeout = 1))
	#pub.publish("Flag 2")
except:
	pass
#Arm = serial.Serial('/dev/ttyACM0',115200,timeout = 1)
time.sleep(2)
inputString = ""

Arm = None
arduino = None

for port in serialList:
	try:
		returnValue = port.read(3)
		#pub.publish("returnValue is: {}".format(returnValue))
		if returnValue == "arm":
			Arm = port
			pub.publish("Arm found")
		elif returnValue == "pla":
			arduino = port
			pub.publish("Platform found")
	except serial.SerialException:
		pass

PathSent = False
def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "Iarduino heard %s", data.data)
	global PathSent
	global pub
	global x
	global y
	global PtIndex
	message = str(data.data)
	#pub.publish(data.data)
	if message.startswith("31"):
		if message[4:7] == "022" and PathSent == True:
			#pub.publish("022")
			info = message.split("(")
			message = info[1]
			message = (message[:-65]).split(",")
			PosX = int(message[1])
			PosY = int(message[2])
			alpha = int(message[3])
			pub.publish("    ")
			# create complex next position
			nextPos = x[PtIndex]+y[PtIndex]*1j
			pub.publish("next pos:" + str(nextPos))
			# create complex current position
			currentPos = PosX+PosY*1j
			pub.publish("currentPos:" + str(currentPos))
			# create error vector --> this gives Xerr and Yerr
			currentPosError = nextPos-currentPos
			pub.publish("currentPosError:" + str(currentPosError))
			Xerr,Yerr = currentPosError.real,currentPosError.imag

			# take arg of error vector and subtract alpha
			desiredAlpha = np.angle(currentPosError)*100
			pub.publish("InitialDesiredAlpha: " + str(desiredAlpha))
			#remap desiredAlpha
			if(desiredAlpha<0):
				desiredAlpha =   (math.pi)*200.0 - abs(desiredAlpha)

			#desiredAlpha = (math.pi)*200.0  - desiredAlpha

			alphaError = (desiredAlpha)-alpha #+ (2*math.pi)*100.0
			if(alphaError > (math.pi)*100.0):
				alphaError = alphaError - (math.pi)*200.0
			elif(alphaError < -(math.pi)*100.0):
				alphaError = alphaError + (math.pi)*200.0
			pub.publish("DesiredAlpha: " + str(desiredAlpha))

			#desiredAlpha = (desiredAlpha + (math.pi)*100.0) % ((math.pi)*200.0)

			#pub.publish("final DesiredAlpha: " + str(desiredAlpha))
			# if(desiredAlpha<0):
			# 	desiredAlpha = desiredAlpha + 2*math.pi
			# pub.publish("alpha: " + str(alpha))
			# if alphaError > (math.pi*100):
			# 	alphaError = (200*math.pi) -  alphaError
			# elif alphaError < -math.pi:
			# 	alphaError = 2*math.pi + alphaError

			ErrorVector = "(" + str(int(Xerr))+","+str(int(Yerr))+","+str(int(alphaError)) + ")"
			pub.publish(ErrorVector)
			if arduino != None:
				try:
					arduino.write(ErrorVector)
					#Display.write(ErrorVector)
					#pass
				except serial.SerialException:
					pass
				#pub.publish("No Serial")
				#exit()


		elif message[4:7] == "019":
			pub.publish("019")
			info = message.split("(")
			message = info[1]
			message = message[:-65].split(",")
			length = int(message[0])
			x = []
			y = []
			for i in range(length):
				x.append(int(message[(2*i)+1]))
				y.append(int(message[(2*i)+2]))
			PathSent = True

		elif message[4:7] == "041":
			if Arm != None:
				pub.publish("041")
				info = message.split("(")
				message = info[1]
				message = message[:-1].split(",")
				x = int(message[0])
				y = int(message[1])
				z = int(message[2])
				Suction = int(message[3])
				try:
					pub.publish("G1 X" + str(x) + " Y" + str(y) + " Z" + str(z) +" F100")
					Arm.write("G1 X" + str(x) + " Y" + str(y) + " Z" + str(z) +" F100\n")
					time.sleep(3)
					pub.publish("M2231 V"+str(Suction))
					Arm.write("M2231 V"+str(Suction)+"\n")
				except serial.SerialException:
					pass

rospy.Subscriber('/transport', String, callback)
try:
	rospy.spin()

except rospy.ROSInterruptException:
    pass

#!/usr/bin/env python
#import rospy
#import sys
#import time
#import serial
#from std_msgs.msg import String

#arduino = serial.Serial('/dev/ttyACM0', 9600,timeout = 1)
#arduino = serial.Serial('/dev/ttyUSB0', 9600,timeout = 1)
#time.sleep(2)

# L = 3.4
# W = 1.6
#i = 0
#PathSent = False
#path2 = ['<'+'P','047','100','100','200','200','300','300','400','400','500','500'+'>'+"\n"]
#pastring2 = ",".join(path)

#rospy.init_node('TransportNode', anonymous=True)
#pub = rospy.Publisher('/PlatformError', String, queue_size=10)
#r = rospy.Rate(10)

#def convert(input,old_min,old_max,new_min,new_max):
        # take original value rangeand map it to a new range
#        old_value =  input
#        new_value = ( ( old_value - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
#        return(new_value)

#def serialprint(data):
#        global PathSent

#        message = str(data.data)

#        if message[4:7] == "022" and PathSent == True:
#                info = message.split("(")
#                message = info[1]
#                message = (message[:-65]).split(",")
#                NID = message[0]
#                X  = message[1]
#                Y  = message[2]
#                a  = message[3]

#                pos = ['<'+ NID,X,Y,a +'>']
#                postring = ",".join(pos)
#                print(postring)
#                arduino.write(postring)
#                rospy.loginfo(postring)
	        #while not rospy.is_shutdown():
               	#	pub.publish(postring)
                #	r.sleep()


#        elif message[4:7] == "019":
#                PathSent = True
#                NID = 'P'
#                Length = message[11:14]
#                info = message.split("(")
#                message = info[1]
#                message = message[:-65].split(",")
#                x = message[0]
#                xy = ' '

#                for i in range(1,x):
#                        xy += message[i]
#
#                path = ['<'+ NID,Length,xy+'>']
#                pathstring = ",".join(path)
#                print(pathring)
#                arduino.write(pathstring)
#                rospy.loginfo(pathring)
	       # while not rospy.is_shutdown():
               # 	pub.publish(pathstring)
               # 	r.sleep()


#def listener():
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
#        rospy.init_node('TransportNode', anonymous=True)
#        rospy.Subscriber("/Transport", String, serialprint)
#	rospy.spin() #simply keeps python from exiting until this node is stopped

#if __name__ == '__main__':
#	try:
#		listener()
#        except rospy.ROSInterruptException:
#                pass
