import numpy as np
from PIL import Image
import argparse

def output_costmap(costmap, output_name):	
	data = np.load(costmap)  

	x_min = data["xBounds"][0]
	x_max = data["xBounds"][1]

	y_min = data["yBounds"][0]
	y_max = data["yBounds"][1]

	ppm = data["pixelsPerMeter"][0]
	width = int((x_max-x_min) * ppm)
	height = int((y_max - y_min) * ppm)

	print "width: " + str(width)
	print "Height: " + str(height)

	for item in data:
		print item + ": " + str(len(data[item]))
	# 	print data[item]

	print "channel0"
	print data["channel0"]

	print "channel1"
	print data["channel1"]

	print "channel2"
	print data["channel2"]

	print "channel3"
	print data["channel3"]
	
	print "filterChannel"
	print data["filterChannel"]

	print "max/min"
	print max(data["channel0"])
	print min(data["channel0"])
	print sum(data["channel0"])/len(data["channel0"])

	print "max/min"
	print max(data["channel1"])
	print min(data["channel1"])
	print sum(data["channel1"])/len(data["channel1"])

	r = np.zeros((3,3))
	r[0, 0] = 1./(x_max - x_min)
	r[1, 1] = 1./(y_max - y_min)
	r[2, 2] = 1.


	print "Rotation Matrix"
	print r
	trs = np.zeros((1,3))
	trs[0,0] =  -x_min/(x_max - x_min)
	trs[0,1] = -y_min/(y_max - y_min)
	trs[0,2] = 1.

	print "Translation Matrix"
	print trs  
	
	channel0 = np.array(data["channel0"], dtype = np.float32)	
	img = channel0.reshape((width,height))
	img = img[::-1,:]

	for x in range(width):
		for y in range(height):			
			if img[x, y] > 1.0:
				img[x, y] = 1.0


	img = np.array(img*255.0, dtype=np.uint8)
	img = Image.fromarray(img)
	img.save(output_name)

	channel1 = np.array(data["channel1"], dtype = np.float32)	
	img1 = channel1.reshape((width,height))
	img1 = img1[::-1,:]

	for x in range(width):
		for y in range(height):			
			if img1[x, y] > 1.0:
				img1[x, y] = 1.0

	img1 = np.array(img1*255.0, dtype=np.uint8)
	img1 = Image.fromarray(img1)
	img1.save("ch1" + output_name)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-i", "--input", type = str, help = "Costmap in old .txt format")
  parser.add_argument("-d", "--display", type = str, help = "Name of image to save costmap as", default="display_image.jpg")  
  args = vars(parser.parse_args())
  output_costmap(args["input"], args["display"])
