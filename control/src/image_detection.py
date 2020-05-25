
#!/usr/bin/env python

from __future__ import division

import jetson.inference
import jetson.utils
import rospy
from std_msgs.msg import Float32, Bool, UInt16
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesis, BoundingBox2D

class Detection_Monitor(object):
    def __init__(self):
        super(Detection_Monitor, self).__init__()
        rospy.init_node("detection_monitor")

        #variables
        self.last_detection_array = Detection2DArray()

        # publishers
        self.detection_publisher = rospy.Publisher("detections/humans", Bool, queue_size=1)
       
        # ros setup
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

		net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
		camera = jetson.utils.gstCamera(640, 480, "/dev/video0")
		display = jetson.utils.glDisplay()

		while display.IsOpen():
			img, width, height = camera.CaptureRGBA()
			detections = net.Detect(img, width, height)

			#uncomment these lines to show video feed
			# display.RenderOnce(img, width, height)
			# display.SetTitle("Object Detection | {:.0f} FPS".format(net.GetNetworkFPS()))
			
			for detection in detections:
				# print(detection)
				print(net.GetClassDesc(detection.ClassID),detection.Confidence)

				#print(detection.ClassID)    #to output the contents of the detection obj
				#net.GetClassDesc(detection.ClassID) will give type of object seen

				# ClassID: 65
				# Confidence: 0.771499
				# Left:    85.914
				# Top:     67.8477
				# Right:   634.4
				# Bottom:  318.183
				# Width:   548.486
				# Height:  250.335
				# Area:    137305
				# Center:  (360.157, 193.015)









