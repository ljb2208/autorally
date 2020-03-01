import numpy as np
from PIL import Image
import argparse
import rosbag
import cv2
import math
import rospy
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError


# state = nav_msgs/Odometry
# image_raw = sensor_msgs/Image
x_min = 0
x_max = 0
x_range = 0
y_min = 0
y_max = 0
y_range = 0
ppm = 0
width = 0
height = 0
costMapImage = 0
r = 0
trs = 0
debugImage = 0
counter = 0
lastHeading = 0.
headingMultiplier = 0

def loadCostMap(costMap):
	global x_min
	global x_max
	global x_range
	global y_min
	global y_max
	global y_range
	global ppm
	global width
	global height
	global costMapImage
	global r
	global trs
	global debugImage
	data = np.load(costMap)  

	x_min = data["xBounds"][0]
	x_max = data["xBounds"][1]

	x_range = x_max - x_min

	y_min = data["yBounds"][0]
	y_max = data["yBounds"][1]

	y_range = y_max - y_min

	ppm = data["pixelsPerMeter"][0]
	width = int((x_max-x_min) * ppm)
	height = int((y_max - y_min) * ppm)

	r = np.zeros((3,3))
	r[0, 0] = 1./(x_max - x_min)
	r[1, 1] = 1./(y_max - y_min)
	r[2, 2] = 1.
	
	trs = np.zeros((1,3))
	trs[0,0] =  -x_min/(x_max - x_min)
	trs[0,1] = -y_min/(y_max - y_min)
	trs[0,2] = 1.

	channel0 = np.array(data["channel0"], dtype = np.float32)	

	costMapImage = channel0.reshape((width,height))
	costMapImage = costMapImage[::-1,:]

	for x in range(width):
		for y in range(height):			
			if costMapImage[x, y] > 1.0:
				costMapImage[x, y] = 1.0


	costMapImage = np.array(costMapImage*255.0, dtype=np.uint8)

	debugImage = cv2.cvtColor(costMapImage, cv2.COLOR_GRAY2RGB)
	cv2.imwrite("debug_pre.png", debugImage)

	print "val: " + str(debugImage[450, 248])
	print "valalt: " + str(debugImage[248, 450])

	print "Cost map loaded."
	print "X  : " + str(x_min) + "/" + str(x_max)
	print "Y  : " + str(y_min) + "/" + str(y_max)
	print "PPM: " + str(ppm)
	print "sz : " + str(width) + "/" + str(height)
	print "rmat"
	print r
	print "trans"
	print trs

def getAvgCost(x, y):
	global costMapImage
	global width
	global height

	cnt = 0
	cost = 0

	for x_val in range(3):
		for y_val in range(3):
			if (x_val < 0 or x_val >= width):
				continue
			if (y_val < 0 or y_val >= height):
				continue

			cost += costMapImage[y, x]
			cnt += 1
	
	return int(cost/cnt)

def generateCostMap(x, y, heading, outputPath):
	global costMapImage
	global width
	global height
	global counter

	cmWidth = 160
	cmHeight = 128

	print "x,y: " + str(x) + "/" + str(y)

	ang = heading - math.pi /2

	x_new = x + (cmWidth/2) * math.cos(ang)
	y_new = y + (cmWidth/2) * -math.sin(ang)	

	bottomRight = (int(x_new), int(y_new))

	x_new = x_new + cmHeight * math.cos(heading)
	y_new = y_new + (cmHeight) * -math.sin(heading)	

	topRight = (int(x_new), int(y_new))

	ang = heading + math.pi / 2
	print "heading/ang: " + str(heading) + "/" + str(ang)

	x_new = x + (cmWidth/2) * math.cos(ang)
	y_new = y + (cmWidth/2) * -math.sin(ang)	

	bottomLeft = (int(x_new),int(y_new))	

	x_new = x_new + cmHeight * math.cos(heading)		
	y_new = y_new + (cmHeight) * -math.sin(heading)		

	topLeft = (int(x_new), int(y_new))

	
	ang2 = heading - math.pi / 2
	x_delta = math.cos(ang2)
	y_delta = -math.sin(ang2)

	ang3 = ang2 - math.pi /2
	print bottomLeft
	print topLeft
	print topRight
	print bottomRight

	x_delta = float(topRight[0] - topLeft[0]) / float(cmWidth)
	y_delta = float(topRight[1] - topLeft[1]) / float(cmHeight)
	x_delta_step = float(bottomLeft[0] - topLeft[0]) / float(cmWidth)
	y_delta_step = float(bottomLeft[1] - topLeft[1]) / float(cmHeight)

	print "x/y delta: " + str(x_delta) + "/" + str(y_delta)

	cm = np.zeros((cmWidth, cmHeight), dtype=np.uint8)

	print "x/y delta step: " + str(x_delta_step) + "/" + str(y_delta_step)

	max_x = 0
	min_x = width

	max_y = 0
	min_y = height
	
	outputCM = debugImage
	cv2.line(outputCM, topLeft, topRight, (0,0,255), 2)
	cv2.line(outputCM, topRight, bottomRight, (0,0,255), 2)
	cv2.line(outputCM, bottomRight, bottomLeft, (0,0,255), 2)
	cv2.line(outputCM, bottomLeft, topLeft, (0,0,255), 2)	

	for u in range(cmWidth):
		x_c = topLeft[0] + (u * x_delta_step)		
		y_c = topLeft[1] + (u * y_delta_step)		

		# print ("row reset: " + str(x_c) + "/" + str(y_c))

		# print "u: " + str(u)

	 	for v in range(cmHeight):
			ix_c = int(x_c)
			iy_c = int(y_c)

			if (u == 0 and v == 0):
				topLeftA = (ix_c, iy_c)
			
			if (u == 0 and v == (cmHeight - 1)):
				topRightA =  (ix_c, iy_c)

			if (u == (cmWidth - 1) and v == 0):
				bottomLeftA = (ix_c, iy_c)

			if (u == (cmWidth - 1) and v == (cmHeight - 1)):
				bottomRightA = (ix_c, iy_c)

			if (ix_c > max_x):
				max_x = ix_c
			
			if (ix_c < min_x):
				min_x = ix_c

			if (iy_c > max_y):
				max_y = iy_c
			
			if (iy_c < min_y):
				min_y = iy_c

			if (ix_c < 0 or iy_c < 0):
				cm[u, v] = 255
			elif (ix_c >= width or iy_c >= height):
				cm[u, v] = 255
			else:
				cm[u, v] = getAvgCost(ix_c, iy_c)
				# costMapImage[ix_c, iy_c]

			# print "ix/y: " + str(ix_c) + "/" + str(iy_c) + "/" + str(cm[u,v]) + "/" + str(costMapImage[iy_c, ix_c])

			x_c += x_delta
			y_c += y_delta




	cv2.imwrite(outputPath + "/output/" + str(counter) + ".png", cm)
	# print "x/y/x new/y new: " + str(x) + "/" + str(y) + "/" + str(x_new) + "/" + str(y_new)

	# print "min/max: " + str(min_x) + "/" + str(max_x) + "    " + str(min_y) + "/" + str(max_y)
	
	# cv2.line(outputCM, topLeftA, topRightA, (255,0,0), 2)
	# cv2.line(outputCM, topRightA, bottomRightA, (255,0,0), 2)
	# cv2.line(outputCM, bottomRightA, bottomLeftA, (255,0,0), 2)
	# cv2.line(outputCM, bottomLeftA, topLeftA, (255,0,0), 2)	

	# print bottomLeftA
	# print topLeftA
	# print topRightA
	# print bottomRightA


	# cv2.imwrite("box.png", outputCM)

def processImage(odom, image, outputPath):
	global x_min
	global x_max
	global ppm
	global x_range
	global y_range
	global width
	global height
	global counter
	global lastHeading
	global headingMultiplier

	# if (counter > 40):
	# 	return

	# print "process image"
	q1 = odom.pose.pose.orientation.x
	q2 = odom.pose.pose.orientation.y
	q3 = odom.pose.pose.orientation.z
	q0 = odom.pose.pose.orientation.w

	x_pos = odom.pose.pose.position.x
	y_pos = -odom.pose.pose.position.y
	z_pos = odom.pose.pose.position.z

	x_vel = odom.twist.twist.linear.x
	y_vel = odom.twist.twist.linear.y
	z_vel = odom.twist.twist.linear.z

	if (x_pos > x_max or x_pos < x_min):
		print "outside of cost map x axis"

	if (y_pos > y_max or y_pos < y_min):
		print "outside of cost map y axis"

	# calculate relative x, y positions
	x_rel  = (x_pos - x_min) / x_range
	y_rel  = (y_pos - y_min) / y_range

	x_coord = x_rel * width
	y_coord = y_rel * height

	# Update euler angles. These use the 1-2-3 Euler angle convention.
	roll = math.atan2(2*q2*q3 + 2*q0*q1, q3*q3 - q2*q2 - q1*q1 + q0*q0)
	pitch = -math.asin(2*q1*q3 - 2*q0*q2)
  	yaw = math.atan2(2*q1*q2 + 2*q0*q3, q1*q1 + q0*q0 - q3*q3 - q2*q2)

  	#Don't allow heading to wrap around
	if (lastHeading > 3.0 and yaw < -3.0):
		headingMultiplier += 1	
	elif (lastHeading < -3.0 and yaw > 3.0):
		headingMultiplier -= 1
	
  	lastHeading = yaw
  	yaw = yaw + headingMultiplier*2*3.14159265359
	
	
	# print "counter: " + str(counter)
	# print "x/y: " + str(x_pos) + "/" + str(y_pos) + ": " + str(counter)
	# print "x/y coords: " + str(x_coord) + "/" + str(y_coord)
	# print "Pose: " + str(q0) + "/" + str(q1) + "/" + str(q2) + "/" + str(q3)
	# print "R/P/Y: " + str(roll) + "/" + str(pitch) + "/" + str(yaw)
	# print "LH/HM: " + str(lastHeading) + "/" + str(headingMultiplier)	
	
	x_endcoord = x_coord + 10 * math.cos(yaw)
	y_endcoord = y_coord - 10 * math.sin(yaw)

	# print "x/y end coords: " + str(x_endcoord) + "/" + str(y_endcoord)
	
	# if (counter <= 68):
	#  	cv2.circle(debugImage, (int(x_coord), int(y_coord)), 3, (255, 0, 0), -1)

	cv2.arrowedLine(debugImage, (int(x_coord), int(y_coord)), (int(x_endcoord), int(y_endcoord)), (0,0,255), tipLength=0.2)
	
	# if (counter == 68):
	generateCostMap(x_coord, y_coord, yaw, outputPath)


	inputImage = CvBridge().imgmsg_to_cv2(image, "bgr8")	
	inputImage = cv2.resize(inputImage, (160,128))
	cv2.imwrite(outputPath + "/input/" + str(counter) + ".png", inputImage)

	# print "Position x: " + str(x_pos) + " y: " + str(y_pos) + " z: " + str(z_pos)
	counter += 1
	

def outputData(inputBag, costMap, outputPath):
	loadCostMap(costMap)
	bag = rosbag.Bag(inputBag)		

	for topic, msg, t in bag.read_messages(topics=['/left_camera/image_raw', '/ground_truth/state']):		
		if (topic == "/ground_truth/state"):
			odom = msg

		if (topic == "/left_camera/image_raw"):
			processImage(odom, msg, outputPath)


	cv2.imwrite("debug_path.png", debugImage)

	return

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-i", "--input", type = str, help = "ros bag file")
  parser.add_argument("-c", "--costmap", type = str, help = "cost map file")
  parser.add_argument("-o", "--output", type = str, help = "Output path")  
  args = vars(parser.parse_args())
  outputData(args["input"], args["costmap"], args["output"])
