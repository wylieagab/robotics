#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from math import pi


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber(
            "camera/rgb/image_raw", Image, self.imageCallback
        )
        self.cmd_vel_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1
        )
        self.twist = Twist()
        self.lastColor = None
        self.h = 0
        self.w = 0
        self.d = 0

    def imageCallback(self, msg):

        # Detect color
        color, blobs_list, hsv, image = self.detectColor(msg)

        if color == "green" or color == "blue":
            self.followBlob(color, blobs_list, image)
            self.lastColor = color
        elif color == "red":
            # stop and terminate
            print("Red - stopping")
            self.cmd_vel_pub.publish(Twist())
            rospy.signal_shutdown("Terminate at Red")
        elif color == "yellow":
            # No colors were detected, we will follow yellow based on toggle
            self.followYellow(hsv, image)

    def detectColor(self, msg):
        """
        Detects RBG colors and returns image and hsv
        """

        # Convert image to HSV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.h, self.w, self.d = image.shape
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for RBG, check color map in documentation
        maskGreen = cv2.inRange(hsv, (50, 50, 50), (70, 255, 255))
        maskBlue = cv2.inRange(hsv, (110, 50, 50), (130, 250, 250))
        maskRed = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255)) + cv2.inRange(
            hsv, (0, 100, 100), (10, 255, 255)
        )

        # List of color mask in priority
        colorMask = {"green": maskGreen, "blue": maskBlue, "red": maskRed}

        # Green, Blue, Red
        for color, mask in colorMask.items():

            # Filter color
            search_top = 3 * self.h / 4
            search_bot = self.h
            mask[0:search_top, 0 : self.w] = 0
            mask[search_bot : self.h, 0 : self.w] = 0
            M = cv2.moments(mask)

            # Print out each color's area for debugging
            print(color + ": " + str(M["m00"]))
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(color + " x value: " + str(cx))

                # Only return the color if the centroid is around the center of the image
                colorCentroid = (cx, cy)
                blob = self.blobsAboveColor(colorCentroid, hsv)
                if blob != None and color != "red":
                    return color, blob, hsv, image
                if color == "red" and cx < (self.w / 2 + 50) and cx > (self.w / 2 - 50):
                    # Only return red if it is within the middle of the frame
                    # move 4 more times to land exactly on red
                    for x in range(60):
                        self.move(cx)
                    return color, blob, hsv, image

        # None of RBG detected, return yellow
        blob = []
        return "yellow", blob, hsv, image

        # What if there is no color detected?
        raise Exception("No colors detected!")

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(1)

    def rotate(self, direction):
        linear_speed = 0.2
        rate = 50
        linear_duration = distance / linear_speed
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        ticks = int(rate * linear_duration)
        for t in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)

    def move(self, cx):
        print("Moving...")
        err = cx - (self.w / 2)
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 150
        self.cmd_vel_pub.publish(self.twist)

    def getYellowMask(self, hsv, threshold):
        # Create mask for yellow
        mask = cv2.inRange(hsv, (15, 50, 50), (50, 255, 255))
        search_top = (3 * self.h / 4) + threshold
        search_bot = (3 * self.h / 4) + threshold * 2
        mask[0:search_top, 0 : self.w] = 0
        mask[search_bot : self.h, 0 : self.w] = 0
        return mask

    def blobsAboveColor(self, color_centroid, hsv):
        mask = self.getYellowMask(hsv, 20)
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        centroids = []

        if len(contours) > 0:
            # Find blobs
            for c in contours:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if cy < color_centroid[1]:
                        centroids.append((cx, cy))
        if len(centroids) > 0:
            # Display blobs
            cv2.imshow("window_blob", mask)
            cv2.waitKey(3)
            return centroids
        else:
            return None

    def followBlob(self, color, blobs_list, image):
        print("Following the blob, stopping the robot")

        # Find center based on toggle

        centroids = blobs_list

        # filter out only the top 2
        centroids_y = [x[1] for x in centroids]
        while len(centroids) > 2:
            centroids.remove(centroids_y.index(max(centroids_y)))

        centroids_x = [x[0] for x in centroids]
        print(centroids)
        if color == "blue":
            # Blue
            followCenter = centroids[centroids_x.index(max(centroids_x))]
        else:
            # Green
            followCenter = centroids[centroids_x.index(min(centroids_x))]
        print(followCenter)

        # Move towards the centroid
        cv2.circle(image, followCenter, 10, (0, 0, 255), -1)
        self.move(followCenter[0])
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def followYellow(self, hsv, image):
        # Filter out based on last color
        mask = self.getYellowMask(hsv, 20)
        if self.lastColor == "blue":
            mask[0 : (self.w / 4) + 20, :] = 0
        elif self.lastColor == "green":
            mask[(3 * self.w / 4) + 20 :, :] = 0

        # Follow the biggest yellow blob
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
            self.move(cx)

        # Display
        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node("follower")
follower = Follower()
rospy.spin()
